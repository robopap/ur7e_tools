#!/usr/bin/env python3
import socket
import threading
import time
import sys
import os
from collections import deque
from pathlib import Path

import numpy as np

from PySide6.QtCore import QTimer
from PySide6.QtGui import QVector3D
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox,
    QCheckBox, QFrame, QGridLayout
)

import pyqtgraph.opengl as gl

UDP_IP = "0.0.0.0"
UDP_PORT = 33333

CALIBRATION_DEFAULTS = {
    "x_m": 0.0,
    "y_m": -0.75,
    "z_m": 0.45,
    "yaw_deg": 0.0,
}

# Fixed physical seat location in the ROS workcell world.  The participant's
# NANSENSE Studio Hips origin is intentionally kept at (0, 0, 0), so these
# values are a visual placement convenience only.  Relative hand motion is
# unaffected by this translation.
SEAT_PLACEMENT_M = (0.0, -0.75, 0.45)

# Fixed axis convention used by the ROS/RViz representation before session
# yaw is applied.  mirror_lateral controls the sign of PX.
NANSENSE_TO_ROS_AXIS_CONVENTION = (
    "ROS X = s*PX, ROS Y = PZ, ROS Z = PY; "
    "s = -1 when mirror_lateral=true, else +1"
)

# Kept for backward compatibility with older helper code; the UI no longer
# exposes the old feet-target placement workflow.
ZERO_TARGET_M = (0.60, -0.60, 0.0)
LEFT_FOOT_JOINTS = (
    "LeftFoot", "LeftToeBase", "LeftFootToe", "LeftFootToeTip",
)
RIGHT_FOOT_JOINTS = (
    "RightFoot", "RightToeBase", "RightFootToe", "RightFootToeTip",
)

BODY_JOINTS = {
    "Hips","Spine","Spine1","Spine2","Spine3","Neck","Head","HeadTip",
    "LeftShoulder","LeftShoulder2","LeftArm","LeftForeArm","LeftHand",
    "RightShoulder","RightShoulder2","RightArm","RightForeArm","RightHand",
    "LeftUpLeg","LeftLeg","LeftFoot","LeftToeBase","LeftFootToe","LeftFootToeTip",
    "RightUpLeg","RightLeg","RightFoot","RightToeBase","RightFootToe","RightFootToeTip",
}

TRUNK_CHAIN = ["Hips","Spine","Spine1","Spine2","Spine3","Neck","Head","HeadTip"]
LEFT_ARM_CHAIN = ["Spine3","LeftShoulder","LeftShoulder2","LeftArm","LeftForeArm","LeftHand"]
RIGHT_ARM_CHAIN = ["Spine3","RightShoulder","RightShoulder2","RightArm","RightForeArm","RightHand"]
LEFT_LEG_CHAIN = ["Hips","LeftUpLeg","LeftLeg","LeftFoot","LeftToeBase","LeftFootToe","LeftFootToeTip"]
RIGHT_LEG_CHAIN = ["Hips","RightUpLeg","RightLeg","RightFoot","RightToeBase","RightFootToe","RightFootToeTip"]

VIEW_CHAINS = {
    "Full Body": [TRUNK_CHAIN, LEFT_ARM_CHAIN, RIGHT_ARM_CHAIN, LEFT_LEG_CHAIN, RIGHT_LEG_CHAIN],
    "Upper Body": [TRUNK_CHAIN, LEFT_ARM_CHAIN, RIGHT_ARM_CHAIN],
    "Both Arms": [LEFT_ARM_CHAIN, RIGHT_ARM_CHAIN],
    "Left Arm": [LEFT_ARM_CHAIN],
    "Right Arm": [RIGHT_ARM_CHAIN],
}

SKELETON_COLOR = (0.2, 0.75, 1.0, 1.0)
JOINT_COLOR = (1.0, 0.65, 0.15, 1.0)
LEFT_COLOR = (1.0, 0.20, 0.20, 1.0)
RIGHT_COLOR = (0.20, 1.0, 0.40, 1.0)


def normalize_joint_name(name):
    name = name.strip()
    if name.startswith("mixamorig:"):
        name = name[len("mixamorig:"):]
    return name

def parse_triplet(parts, start):
    return float(parts[start]), float(parts[start+1]), float(parts[start+2])

def parse_packet(data):
    text = data.decode("utf-8", errors="ignore")
    receive_time_ns = time.time_ns()
    frame = {
        "receive_time": receive_time_ns / 1e9,
        "receive_time_ns": receive_time_ns,
        "displacement_cm": None,
        "joints": {},
    }

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        parts = [p.strip() for p in line.split(",")]
        key = normalize_joint_name(parts[0])

        if key.lower() == "displacement" and len(parts) >= 4:
            try:
                frame["displacement_cm"] = parse_triplet(parts, 1)
            except ValueError:
                pass
            continue

        if key not in BODY_JOINTS or len(parts) < 11:
            continue

        try:
            p_world = parse_triplet(parts, 2)
            r_local = parse_triplet(parts, 5)
            r_world = parse_triplet(parts, 8)
        except ValueError:
            continue

        frame["joints"][key] = {
            "parent": normalize_joint_name(parts[1]),
            "position_world_cm": p_world,
            "rotation_local_deg": r_local,
            "rotation_world_deg": r_world,
        }

    return frame

class LatestFrameReceiver:
    def __init__(self, ip=UDP_IP, port=UDP_PORT, packet_callback=None):
        self.ip = ip
        self.port = port
        self.packet_callback = packet_callback
        self.sock = None
        self.thread = None
        self.running = False
        self.lock = threading.Lock()
        self.latest_frame = None
        self.last_frame_monotonic = None
        self.rate = 0.0
        self.rate_count = 0
        self.rate_t0 = time.monotonic()

    def start(self):
        if self.running:
            return
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.2)

        self.running = True
        self.rate = 0.0
        self.rate_count = 0
        self.rate_t0 = time.monotonic()
        with self.lock:
            self.latest_frame = None
            self.last_frame_monotonic = None

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            frame = parse_packet(data)
            if "Hips" not in frame["joints"]:
                continue

            now = time.monotonic()
            with self.lock:
                self.latest_frame = frame
                self.last_frame_monotonic = now

            if self.packet_callback is not None:
                self.packet_callback(frame)

            self.rate_count += 1
            dt = now - self.rate_t0
            if dt >= 1.0:
                self.rate = self.rate_count / dt
                self.rate_count = 0
                self.rate_t0 = now

    def get_latest(self):
        with self.lock:
            return self.latest_frame

    def stream_health(self, max_age_s=0.5):
        """Return native UDP freshness without depending on render FPS."""
        now = time.monotonic()
        with self.lock:
            frame = self.latest_frame
            last = self.last_frame_monotonic
        age_s = None if last is None else max(0.0, now - last)
        ready = bool(
            self.running
            and frame is not None
            and age_s is not None
            and age_s <= float(max_age_s)
        )
        return {
            "ready": ready,
            "running": bool(self.running),
            "age_s": age_s,
            "rate_hz": float(self.rate),
        }

    def is_streaming(self, max_age_s=0.5):
        return bool(self.stream_health(max_age_s)["ready"])

    def stop(self):
        self.running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

        with self.lock:
            self.latest_frame = None
            self.last_frame_monotonic = None
        self.rate = 0.0

def display_position(frame, joint_name, mirror_lateral=False):
    """
    EXACT same viewer orientation as the last working standalone plot:
      viewer X = NANSENSE PX - Hips.PX
      viewer Y = NANSENSE PZ - Hips.PZ
      viewer Z = -(NANSENSE PY - Hips.PY)

    Raw NANSENSE data are not modified.
    """
    joints = frame["joints"]
    p = joints[joint_name]["position_world_cm"]
    hips = joints["Hips"]["position_world_cm"]

    x = p[0] - hips[0]
    if mirror_lateral:
        x = -x
    y = p[2] - hips[2]
    z = -(p[1] - hips[1])
    return x, y, z

def opengl_scene_positions(xyz):
    """Map viewer coordinates into the upright OpenGL scene."""
    scene_xyz = np.array(xyz, dtype=float, copy=True)
    if scene_xyz.size:
        scene_xyz[:, 2] *= -1.0
    return scene_xyz


def seat_calibration_values(yaw_deg):
    """Return the fixed visual seat placement while preserving session yaw."""
    return {
        "x_m": float(SEAT_PLACEMENT_M[0]),
        "y_m": float(SEAT_PLACEMENT_M[1]),
        "z_m": float(SEAT_PLACEMENT_M[2]),
        "yaw_deg": float(yaw_deg),
    }


def scientific_alignment_snapshot(values):
    """Freeze the orientation fields that affect relative hand displacement."""
    return {
        "locked": True,
        "x_m": float(values["x_m"]),
        "y_m": float(values["y_m"]),
        "z_m": float(values["z_m"]),
        "yaw_deg": float(values["yaw_deg"]),
        "mirror_lateral": bool(values["mirror_lateral"]),
        "axis_convention": NANSENSE_TO_ROS_AXIS_CONVENTION,
    }


def scientific_alignment_matches(locked, current, yaw_tol_deg=1e-9):
    """Return whether current scientific orientation still matches the lock.

    XYZ translation is intentionally ignored: it only places the skeleton in
    RViz and cancels when hand motion is expressed relative to its first
    sample.  Yaw and mirror affect the displacement direction and therefore
    invalidate the scientific lock when changed.
    """
    if not locked:
        return False
    return (
        abs(float(locked["yaw_deg"]) - float(current["yaw_deg"]))
        <= float(yaw_tol_deg)
        and bool(locked["mirror_lateral"])
        == bool(current["mirror_lateral"])
    )

def zero_calibration_from_feet(
    frame, target_m, yaw_deg, mirror_lateral=False
):
    """Return XYZ offsets that place the two lowest foot points at target."""
    joints = frame["joints"]

    def lowest_available(names):
        available = [
            joints[name]["position_world_cm"]
            for name in names
            if name in joints
        ]
        if not available:
            return None
        # NANSENSE PY is the upright ROS Z coordinate.
        return min(available, key=lambda position: position[1])

    left = lowest_available(LEFT_FOOT_JOINTS)
    right = lowest_available(RIGHT_FOOT_JOINTS)
    if left is None or right is None:
        raise ValueError("both left and right foot joints are required")

    foot_x_m = (left[0] + right[0]) * 0.005
    if mirror_lateral:
        foot_x_m = -foot_x_m
    foot_y_m = (left[2] + right[2]) * 0.005
    floor_z_m = min(left[1], right[1]) * 0.01

    yaw = np.deg2rad(yaw_deg)
    rotated_x = np.cos(yaw) * foot_x_m - np.sin(yaw) * foot_y_m
    rotated_y = np.sin(yaw) * foot_x_m + np.cos(yaw) * foot_y_m
    return {
        "x_m": float(target_m[0] - rotated_x),
        "y_m": float(target_m[1] - rotated_y),
        "z_m": float(target_m[2] - floor_z_m),
        "yaw_deg": float(yaw_deg),
    }

class RenderRateMeter:
    """Rolling rate of frames confirmed as rendered by the GUI backend."""
    def __init__(self, window_sec=2.0):
        self.window_sec = window_sec
        self.timestamps = deque()

    def record(self, *_args):
        now = time.monotonic()
        self.timestamps.append(now)
        cutoff = now - self.window_sec
        while self.timestamps and self.timestamps[0] < cutoff:
            self.timestamps.popleft()

    def rate(self):
        if len(self.timestamps) < 2:
            return 0.0
        elapsed = self.timestamps[-1] - self.timestamps[0]
        return (len(self.timestamps) - 1) / elapsed if elapsed > 0 else 0.0

    def reset(self):
        self.timestamps.clear()

class NansenseLiveWidget(QWidget):
    def __init__(
        self,
        parent=None,
        frame_callback=None,
        calibration_callback=None,
        packet_callback=None,
    ):
        super().__init__(parent)
        self.receiver = LatestFrameReceiver(
            packet_callback=packet_callback
        )
        self.frame_callback = frame_callback
        self.calibration_callback = calibration_callback
        self._locked_alignment = None
        config_home = Path(
            os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")
        )
        self.calibration_path = (
            config_home / "ur7e_tools" / "nansense_calibration.yaml"
        )

        # Keep numeric controls readable both inside the dark workcell UI and
        # when this widget is launched on its own.
        self.setStyleSheet(
            """
            QDoubleSpinBox {
                background: #303134;
                color: #ffffff;
                border: 1px solid #7a7f85;
                border-radius: 4px;
                padding: 3px 20px 3px 6px;
                selection-background-color: #5f6368;
                selection-color: #ffffff;
            }
            QDoubleSpinBox:focus { border-color: #8ab4f8; }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 16px;
                background: #3c4043;
                border-left: 1px solid #5f6368;
            }
            QFrame#calibrationCard {
                background: #292a2d;
                border: 1px solid #3c4043;
                border-radius: 6px;
            }
            QLabel#calibrationHint { color: #9aa0a6; }
            QLabel#telemetryBadge {
                background: #303134;
                color: #bdc1c6;
                border-radius: 4px;
                padding: 3px 7px;
            }
            QLabel#nansenseOnline {
                background: #254c32; color: #81c995;
                border-radius: 4px; padding: 3px 7px; font-weight: 700;
            }
            QLabel#nansenseOffline {
                background: #542b29; color: #f28b82;
                border-radius: 4px; padding: 3px 7px; font-weight: 700;
            }
            QLabel#calibrationSaved { color: #81c995; font-weight: 700; }
            QLabel#calibrationPending { color: #fdd663; font-weight: 700; }
            QLabel#calibrationError { color: #f28b82; font-weight: 700; }
            QPushButton#placementButton {
                background: #34506f;
                border: 1px solid #5f83aa;
            }
            QPushButton#placementButton:hover { background: #41658c; }
            QPushButton#saveCalibrationButton {
                background: #245c34;
                border: 1px solid #3f8051;
            }
            QPushButton#saveCalibrationButton:hover { background: #2f7543; }
            QPushButton#lockAlignmentButton {
                background: #34506f;
                border: 1px solid #5f83aa;
                font-weight: 700;
            }
            QPushButton#lockAlignmentButton:hover { background: #41658c; }
            """
        )

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(6, 6, 6, 6)
        self.main_layout.setSpacing(6)

        connection_row = QHBoxLayout()

        self.connect_button = QPushButton("CONNECT NANSENSE")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_row.addWidget(self.connect_button)
        self.connection_badge = QLabel("DISCONNECTED")
        self.connection_badge.setObjectName("nansenseOffline")
        connection_row.addWidget(self.connection_badge)
        connection_row.addStretch(1)

        self.udp_badge = QLabel("UDP -- Hz")
        self.render_badge = QLabel("Render -- FPS")
        self.age_badge = QLabel("Age -- ms")
        for badge in (self.udp_badge, self.render_badge, self.age_badge):
            badge.setObjectName("telemetryBadge")
            connection_row.addWidget(badge)
        self.main_layout.addLayout(connection_row)

        controls_row = QHBoxLayout()

        controls_row.addWidget(QLabel("View:"))

        self.view_combo = QComboBox()
        self.view_combo.addItems(VIEW_CHAINS.keys())
        self.view_combo.setCurrentText("Full Body")
        controls_row.addWidget(self.view_combo)

        self.front_view_button = QPushButton("FRONT")
        self.front_view_button.clicked.connect(
            lambda: self.set_view_preset("front")
        )
        controls_row.addWidget(self.front_view_button)
        self.side_view_button = QPushButton("SIDE")
        self.side_view_button.clicked.connect(
            lambda: self.set_view_preset("side")
        )
        controls_row.addWidget(self.side_view_button)
        self.top_view_button = QPushButton("TOP")
        self.top_view_button.clicked.connect(
            lambda: self.set_view_preset("top")
        )
        controls_row.addWidget(self.top_view_button)

        self.reset_view_button = QPushButton("FIT")
        self.reset_view_button.setToolTip(
            "Fit the skeleton without changing the selected view direction."
        )
        self.reset_view_button.clicked.connect(self.reset_view)
        controls_row.addWidget(self.reset_view_button)

        self.swap_sides_checkbox = QCheckBox("Mirror lateral axis")
        self.swap_sides_checkbox.setChecked(True)
        self.swap_sides_checkbox.setToolTip(
            "Reverse the NANSENSE lateral coordinate while preserving the "
            "true Left*/Right* joint identities."
        )
        self.swap_sides_checkbox.toggled.connect(
            lambda _checked: self.mark_calibration_modified("mirror_lateral")
        )
        controls_row.addWidget(self.swap_sides_checkbox)
        controls_row.addStretch(1)
        self.main_layout.addLayout(controls_row)

        calibration_card = QFrame()
        calibration_card.setObjectName("calibrationCard")
        calibration_grid = QGridLayout(calibration_card)
        calibration_grid.setContentsMargins(9, 7, 9, 7)
        calibration_grid.setHorizontalSpacing(7)
        calibration_grid.addWidget(QLabel("NANSENSE / Workcell Alignment"), 0, 0, 1, 2)
        target_label = QLabel(
            "Seat: X 0.00  Y -0.75  Z +0.45 m | XYZ visual; Yaw/Mirror lock"
        )
        target_label.setObjectName("calibrationHint")
        calibration_grid.addWidget(target_label, 0, 2, 1, 4)
        self.calibration_spins = {}
        for column, (key, label, minimum, maximum) in enumerate((
            ("x_m", "X [m]", -20.0, 20.0),
            ("y_m", "Y [m]", -20.0, 20.0),
            ("z_m", "Z [m]", -5.0, 5.0),
            ("yaw_deg", "Yaw [deg]", -360.0, 360.0),
        )):
            calibration_grid.addWidget(QLabel(label), 1, column)
            spin = QDoubleSpinBox()
            spin.setRange(minimum, maximum)
            spin.setDecimals(3 if key != "yaw_deg" else 1)
            spin.setSingleStep(0.05 if key != "yaw_deg" else 5.0)
            spin.setValue(CALIBRATION_DEFAULTS[key])
            spin.setMinimumWidth(88 if key != "yaw_deg" else 96)
            spin.valueChanged.connect(
                lambda _value, key=key: self.mark_calibration_modified(key)
            )
            self.calibration_spins[key] = spin
            calibration_grid.addWidget(spin, 2, column)

        self.apply_calibration_button = QPushButton("APPLY")
        self.apply_calibration_button.clicked.connect(self.apply_calibration)
        calibration_grid.addWidget(self.apply_calibration_button, 2, 4)

        self.save_calibration_button = QPushButton("SAVE")
        self.save_calibration_button.setObjectName("saveCalibrationButton")
        self.save_calibration_button.setToolTip(
            "Save the current UI/RViz defaults for the next launch. "
            "SAVE does not lock the scientific session alignment."
        )
        self.save_calibration_button.clicked.connect(self.save_calibration)
        calibration_grid.addWidget(self.save_calibration_button, 2, 5)

        self.place_at_seat_button = QPushButton("PLACE AT SEAT")
        self.place_at_seat_button.setObjectName("placementButton")
        self.place_at_seat_button.setToolTip(
            "Set visual placement to world X=0.00 m, Y=-0.75 m, Z=+0.45 m. "
            "Current yaw and mirror setting are preserved."
        )
        self.place_at_seat_button.clicked.connect(self.place_at_seat)
        calibration_grid.addWidget(self.place_at_seat_button, 3, 0, 1, 3)

        self.lock_alignment_button = QPushButton("LOCK ALIGNMENT")
        self.lock_alignment_button.setObjectName("lockAlignmentButton")
        self.lock_alignment_button.setToolTip(
            "Freeze the current Yaw + Mirror convention as the scientific "
            "alignment for the upcoming experiment session. XYZ remain visual."
        )
        self.lock_alignment_button.clicked.connect(self.lock_alignment)
        calibration_grid.addWidget(self.lock_alignment_button, 3, 3, 1, 3)

        self.calibration_status_label = QLabel("")
        self.calibration_status_label.setToolTip(str(self.calibration_path))
        calibration_grid.addWidget(self.calibration_status_label, 4, 0, 1, 6)
        self.main_layout.addWidget(calibration_card)

        side_key = QLabel("Identity markers:  L = red   |   R = green")
        side_key.setToolTip(
            "The labels follow the Left*/Right* joint names received in the "
            "NANSENSE packet; they are not inferred from the camera view."
        )
        self.main_layout.addWidget(side_key)

        self._loading_calibration = True
        self.load_calibration()
        self._loading_calibration = False
        self.apply_calibration(show_status=False)

        self.opengl_view = None
        self.opengl_grid = None
        self.opengl_lines = []
        self.opengl_points = []
        self.opengl_hand_points = {}
        self.opengl_render_rate = RenderRateMeter()
        self.opengl_render_pending = False
        self.current_view_preset = "front"
        if gl is not None:
            self.opengl_view = gl.GLViewWidget()
            self.opengl_view.setBackgroundColor((24, 26, 28, 255))
            self.opengl_view.opts["center"] = QVector3D(0, 0, 0)
            self.opengl_view.setCameraPosition(
                distance=350, elevation=5, azimuth=-90
            )

            self.opengl_grid = gl.GLGridItem()
            self.opengl_grid.setSize(220, 220)
            self.opengl_grid.setSpacing(40, 40)
            self.opengl_view.addItem(self.opengl_grid)

            max_chains = max(len(chains) for chains in VIEW_CHAINS.values())
            for _ in range(max_chains):
                line = gl.GLLinePlotItem(
                    pos=np.empty((0, 3), dtype=float),
                    color=(0.2, 0.75, 1.0, 1.0),
                    width=3,
                    antialias=True,
                    mode="line_strip",
                )
                points = gl.GLScatterPlotItem(
                    pos=np.empty((0, 3), dtype=float),
                    color=(1.0, 0.65, 0.15, 1.0),
                    size=7,
                    pxMode=True,
                )
                self.opengl_lines.append(line)
                self.opengl_points.append(points)
                self.opengl_view.addItem(line)
                self.opengl_view.addItem(points)

            for joint_name, color in (
                ("LeftHand", LEFT_COLOR),
                ("RightHand", RIGHT_COLOR),
            ):
                hand_point = gl.GLScatterPlotItem(
                    pos=np.empty((0, 3), dtype=float),
                    color=color,
                    size=16,
                    pxMode=True,
                )
                self.opengl_hand_points[joint_name] = hand_point
                self.opengl_view.addItem(hand_point)

            self.main_layout.addWidget(self.opengl_view, stretch=1)
            self.opengl_view.frameSwapped.connect(
                self.record_opengl_frame
            )
        self.reset_view()

        self.timer = QTimer(self)
        self.timer.setInterval(33)  # ~30 FPS visualization
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def calibration_values(self):
        values = {
            key: spin.value()
            for key, spin in self.calibration_spins.items()
        }
        values["mirror_lateral"] = self.swap_sides_checkbox.isChecked()
        return values

    def set_calibration_status(self, text, state="pending"):
        object_names = {
            "saved": "calibrationSaved",
            "pending": "calibrationPending",
            "error": "calibrationError",
        }
        self.calibration_status_label.setText(text)
        self.calibration_status_label.setObjectName(object_names[state])
        self.calibration_status_label.style().unpolish(
            self.calibration_status_label
        )
        self.calibration_status_label.style().polish(
            self.calibration_status_label
        )

    def mark_calibration_modified(self, key=None):
        if getattr(self, "_loading_calibration", False):
            return

        if key in ("yaw_deg", "mirror_lateral") and self._locked_alignment:
            if not scientific_alignment_matches(
                self._locked_alignment, self.calibration_values()
            ):
                self._locked_alignment = None
                self.lock_alignment_button.setText("LOCK ALIGNMENT")
                self.set_calibration_status(
                    "Orientation modified — LOCK AGAIN",
                    "error",
                )
                return

        if key in ("x_m", "y_m", "z_m"):
            suffix = " | alignment LOCKED" if self.is_alignment_locked() else ""
            self.set_calibration_status(
                f"Placement modified — press APPLY{suffix}",
                "pending",
            )
            return

        self.set_calibration_status("Modified — press APPLY", "pending")

    def apply_calibration(self, _checked=False, show_status=True):
        if self.calibration_callback is not None:
            self.calibration_callback(self.calibration_values())
        if show_status:
            if self.is_alignment_locked():
                self.set_calibration_status(
                    "Applied — scientific alignment LOCKED",
                    "saved",
                )
            else:
                self.set_calibration_status(
                    "Applied — press LOCK ALIGNMENT before experiment",
                    "pending",
                )

    def load_calibration(self):
        self._locked_alignment = None
        if hasattr(self, "lock_alignment_button"):
            self.lock_alignment_button.setText("LOCK ALIGNMENT")

        if not self.calibration_path.exists():
            self.set_calibration_status(
                "Using seat defaults — alignment NOT LOCKED",
                "pending",
            )
            return

        values = dict(CALIBRATION_DEFAULTS)
        try:
            for raw_line in self.calibration_path.read_text().splitlines():
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                key, separator, raw_value = line.partition(":")
                parsed_key = key.strip()
                if separator and parsed_key in values:
                    values[parsed_key] = float(raw_value.strip())
                elif separator and parsed_key in (
                    "mirror_lateral", "swap_left_right"
                ):
                    enabled = raw_value.strip().lower() in ("true", "1", "yes")
                    self.swap_sides_checkbox.setChecked(enabled)
        except (OSError, ValueError) as exc:
            self.set_calibration_status(f"Load error: {exc}", "error")
            return

        for key, value in values.items():
            self.calibration_spins[key].setValue(value)
        self.set_calibration_status(
            "UI defaults loaded — alignment NOT LOCKED",
            "pending",
        )

    def save_calibration(self):
        self.apply_calibration(show_status=False)
        values = self.calibration_values()
        text = (
            "# NANSENSE UI/RViz defaults in the ROS world frame\n"
            "# SAVE is persistent UI state; it does not lock a scientific session.\n"
            f"x_m: {values['x_m']:.6f}\n"
            f"y_m: {values['y_m']:.6f}\n"
            f"z_m: {values['z_m']:.6f}\n"
            f"yaw_deg: {values['yaw_deg']:.6f}\n"
            f"mirror_lateral: {str(self.swap_sides_checkbox.isChecked()).lower()}\n"
        )
        temporary_path = self.calibration_path.with_suffix(".yaml.tmp")
        try:
            self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path.write_text(text)
            os.replace(temporary_path, self.calibration_path)
        except OSError as exc:
            self.set_calibration_status(f"Save error: {exc}", "error")
            return

        if self.is_alignment_locked():
            self.set_calibration_status(
                "UI defaults saved — scientific alignment LOCKED",
                "saved",
            )
        else:
            self.set_calibration_status(
                "UI defaults saved — alignment NOT LOCKED",
                "pending",
            )

    def place_at_seat(self):
        values = seat_calibration_values(
            self.calibration_spins["yaw_deg"].value()
        )
        self._loading_calibration = True
        try:
            for key, value in values.items():
                self.calibration_spins[key].setValue(value)
        finally:
            self._loading_calibration = False

        self.apply_calibration(show_status=False)
        if self.is_alignment_locked():
            self.set_calibration_status(
                "Seat placement applied — scientific alignment LOCKED",
                "saved",
            )
        else:
            self.set_calibration_status(
                "Seat placement applied — press LOCK ALIGNMENT when oriented",
                "pending",
            )

    def lock_alignment(self):
        self.apply_calibration(show_status=False)
        self._locked_alignment = scientific_alignment_snapshot(
            self.calibration_values()
        )
        self.lock_alignment_button.setText("ALIGNMENT LOCKED")
        mirror = "ON" if self._locked_alignment["mirror_lateral"] else "OFF"
        self.set_calibration_status(
            f"ALIGNMENT LOCKED — Yaw {self._locked_alignment['yaw_deg']:.1f}° "
            f"| Mirror {mirror}",
            "saved",
        )

    def is_alignment_locked(self):
        return scientific_alignment_matches(
            self._locked_alignment, self.calibration_values()
        )

    def locked_alignment(self):
        if not self.is_alignment_locked():
            return None
        return dict(self._locked_alignment)

    def current_frame(self):
        return self.receiver.get_latest()

    def stream_health(self, max_age_s=0.5):
        return self.receiver.stream_health(max_age_s)

    def is_streaming(self, max_age_s=0.5):
        return self.receiver.is_streaming(max_age_s)

    def toggle_connection(self):
        if self.receiver.running:
            self.disconnect_nansense()
        else:
            self.connect_nansense()

    def connect_nansense(self):
        # Re-apply the values visible in the controls. Connecting never resets
        # or replaces the calibration loaded during widget construction.
        self.apply_calibration(show_status=False)
        try:
            self.receiver.start()
        except OSError as exc:
            self.connection_badge.setText(f"ERROR: {exc}")
            return

        self.connect_button.setText("DISCONNECT")
        self.connection_badge.setText("CONNECTED")
        self.connection_badge.setObjectName("nansenseOnline")
        self.connection_badge.style().unpolish(self.connection_badge)
        self.connection_badge.style().polish(self.connection_badge)

    def disconnect_nansense(self):
        self.receiver.stop()
        if self.frame_callback is not None:
            self.frame_callback(None)
        self.connect_button.setText("CONNECT NANSENSE")
        self.connection_badge.setText("DISCONNECTED")
        self.connection_badge.setObjectName("nansenseOffline")
        self.connection_badge.style().unpolish(self.connection_badge)
        self.connection_badge.style().polish(self.connection_badge)
        self.clear_plot()

    def record_opengl_frame(self, *_args):
        if self.opengl_render_pending:
            self.opengl_render_rate.record()
            self.opengl_render_pending = False

    def set_view_preset(self, preset):
        self.current_view_preset = preset
        self.reset_view()

    def view_angles(self):
        return {
            "front": (5, -90),
            "side": (5, 0),
            "top": (90, -90),
        }[self.current_view_preset]

    def reset_view(self):
        frame = self.current_frame()
        elevation, azimuth = self.view_angles()
        if self.opengl_view:
            center = QVector3D(0, 0, 0)
            distance = 350.0
            if frame is not None:
                xyz = np.asarray([
                    display_position(
                        frame, joint,
                        self.swap_sides_checkbox.isChecked(),
                    )
                    for joint in BODY_JOINTS
                    if joint in frame["joints"]
                ], dtype=float)
                scene_xyz = opengl_scene_positions(xyz)
                if len(scene_xyz):
                    xyz_min = scene_xyz.min(axis=0)
                    xyz_max = scene_xyz.max(axis=0)
                    midpoint = (xyz_min + xyz_max) / 2.0
                    center = QVector3D(*midpoint.tolist())
                    distance = max(220.0, float(np.ptp(scene_xyz, axis=0).max()) * 2.2)
            self.opengl_view.opts["center"] = center
            self.opengl_view.setCameraPosition(
                distance=distance, elevation=elevation, azimuth=azimuth
            )
            self.opengl_view.update()

    def clear_plot(self):
        for line, points in zip(self.opengl_lines, self.opengl_points):
            empty = np.empty((0, 3), dtype=float)
            line.setData(pos=empty)
            points.setData(pos=empty)
        for points in self.opengl_hand_points.values():
            points.setData(pos=np.empty((0, 3), dtype=float))

    def update_plot(self):
        if not self.receiver.running:
            return

        frame = self.current_frame()
        if frame is None:
            self.connection_badge.setText(f"WAITING UDP :{UDP_PORT}")
            return

        self.connection_badge.setText("LIVE")
        self.connection_badge.setObjectName("nansenseOnline")
        self.connection_badge.style().unpolish(self.connection_badge)
        self.connection_badge.style().polish(self.connection_badge)

        if self.frame_callback is not None:
            self.frame_callback(frame)

        chains = VIEW_CHAINS[self.view_combo.currentText()]
        self.clear_plot()

        positions = []
        for chain in chains:
            available_chain = [
                joint for joint in chain if joint in frame["joints"]
            ]
            if len(available_chain) < 2:
                positions.append(np.empty((0, 3), dtype=float))
                continue

            positions.append(np.asarray([
                display_position(
                    frame, joint,
                    self.swap_sides_checkbox.isChecked(),
                )
                for joint in available_chain
            ], dtype=float))

        if self.opengl_view:
            self.opengl_render_pending = True
            for line, points, xyz in zip(
                self.opengl_lines, self.opengl_points, positions
            ):
                scene_xyz = opengl_scene_positions(xyz)
                line.setData(pos=scene_xyz, color=SKELETON_COLOR)
                points.setData(pos=scene_xyz, color=JOINT_COLOR)

            for joint_name, points in self.opengl_hand_points.items():
                if joint_name in frame["joints"]:
                    xyz = np.asarray([
                        display_position(
                            frame, joint_name,
                            self.swap_sides_checkbox.isChecked(),
                        )
                    ], dtype=float)
                    points.setData(pos=opengl_scene_positions(xyz))

            all_xyz = np.asarray([
                display_position(
                    frame, joint,
                    self.swap_sides_checkbox.isChecked(),
                )
                for joint in BODY_JOINTS
                if joint in frame["joints"]
            ], dtype=float)
            scene_xyz = opengl_scene_positions(all_xyz)
            if len(scene_xyz) and self.opengl_grid is not None:
                floor_z = float(scene_xyz[:, 2].min()) - 5.0
                self.opengl_grid.resetTransform()
                self.opengl_grid.translate(0, 0, floor_z)
        age_ms = (time.time() - frame["receive_time"]) * 1000.0
        render_rate = self.opengl_render_rate.rate()
        self.udp_badge.setText(f"UDP {self.receiver.rate:.1f} Hz")
        self.render_badge.setText(f"Render {render_rate:.1f} FPS")
        self.age_badge.setText(f"Age {age_ms:.1f} ms")

    def shutdown(self):
        self.timer.stop()
        self.receiver.stop()
        if self.frame_callback is not None:
            self.frame_callback(None)

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)

def main():
    app = QApplication(sys.argv)
    window = NansenseLiveWidget()
    window.setWindowTitle("NANSENSE QWidget Integration Test")
    window.resize(1000, 720)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
