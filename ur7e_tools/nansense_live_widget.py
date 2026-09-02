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
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QStackedWidget
)

from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

try:
    import pyqtgraph.opengl as gl
except ImportError:
    gl = None

UDP_IP = "0.0.0.0"
UDP_PORT = 33333

CALIBRATION_DEFAULTS = {
    "x_m": 0.0,
    "y_m": 0.0,
    "z_m": 0.0,
    "yaw_deg": 0.0,
}

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

def normalize_joint_name(name):
    name = name.strip()
    if name.startswith("mixamorig:"):
        name = name[len("mixamorig:"):]
    return name

def parse_triplet(parts, start):
    return float(parts[start]), float(parts[start+1]), float(parts[start+2])

def parse_packet(data):
    text = data.decode("utf-8", errors="ignore")
    frame = {"receive_time": time.time(), "displacement_cm": None, "joints": {}}

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
    def __init__(self, ip=UDP_IP, port=UDP_PORT):
        self.ip = ip
        self.port = port
        self.sock = None
        self.thread = None
        self.running = False
        self.lock = threading.Lock()
        self.latest_frame = None
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

            with self.lock:
                self.latest_frame = frame

            self.rate_count += 1
            now = time.monotonic()
            dt = now - self.rate_t0
            if dt >= 1.0:
                self.rate = self.rate_count / dt
                self.rate_count = 0
                self.rate_t0 = now

    def get_latest(self):
        with self.lock:
            return self.latest_frame

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
        self.rate = 0.0

def display_position(frame, joint_name):
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
    y = p[2] - hips[2]
    z = -(p[1] - hips[1])
    return x, y, z

def opengl_scene_positions(xyz):
    """Match Matplotlib's reversed display Z axis without changing source data."""
    scene_xyz = np.array(xyz, dtype=float, copy=True)
    if scene_xyz.size:
        scene_xyz[:, 2] *= -1.0
    return scene_xyz

def zero_calibration_from_feet(frame, target_m, yaw_deg):
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
    ):
        super().__init__(parent)
        self.receiver = LatestFrameReceiver()
        self.frame_callback = frame_callback
        self.calibration_callback = calibration_callback
        config_home = Path(
            os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")
        )
        self.calibration_path = (
            config_home / "ur7e_tools" / "nansense_calibration.yaml"
        )

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(6, 6, 6, 6)
        self.main_layout.setSpacing(6)

        top = QHBoxLayout()

        self.connect_button = QPushButton("CONNECT NANSENSE")
        self.connect_button.clicked.connect(self.toggle_connection)
        top.addWidget(self.connect_button)

        top.addWidget(QLabel("View:"))

        self.view_combo = QComboBox()
        self.view_combo.addItems(VIEW_CHAINS.keys())
        self.view_combo.setCurrentText("Full Body")
        top.addWidget(self.view_combo)

        top.addWidget(QLabel("Renderer:"))

        self.renderer_combo = QComboBox()
        self.renderer_combo.addItem("OpenGL", "opengl")
        self.renderer_combo.addItem("Matplotlib", "matplotlib")
        if gl is None:
            self.renderer_combo.model().item(0).setEnabled(False)
            self.renderer_combo.setCurrentIndex(1)
            self.renderer_combo.setToolTip(
                "Install pyqtgraph and PyOpenGL to enable OpenGL rendering"
            )
        self.renderer_combo.currentIndexChanged.connect(self.change_renderer)
        top.addWidget(self.renderer_combo)

        self.reset_view_button = QPushButton("RESET VIEW")
        self.reset_view_button.clicked.connect(self.reset_view)
        top.addWidget(self.reset_view_button)

        top.addStretch(1)

        self.status_label = QLabel("Disconnected")
        top.addWidget(self.status_label)

        self.main_layout.addLayout(top)

        calibration_row = QHBoxLayout()
        calibration_row.addWidget(QLabel("RViz calibration:"))
        self.calibration_spins = {}
        for key, label, minimum, maximum in (
            ("x_m", "X [m]", -20.0, 20.0),
            ("y_m", "Y [m]", -20.0, 20.0),
            ("z_m", "Z [m]", -5.0, 5.0),
            ("yaw_deg", "Yaw [deg]", -360.0, 360.0),
        ):
            calibration_row.addWidget(QLabel(label))
            spin = QDoubleSpinBox()
            spin.setRange(minimum, maximum)
            spin.setDecimals(3 if key != "yaw_deg" else 1)
            spin.setSingleStep(0.05 if key != "yaw_deg" else 5.0)
            spin.setValue(CALIBRATION_DEFAULTS[key])
            self.calibration_spins[key] = spin
            calibration_row.addWidget(spin)

        self.apply_calibration_button = QPushButton("APPLY")
        self.apply_calibration_button.clicked.connect(self.apply_calibration)
        calibration_row.addWidget(self.apply_calibration_button)

        self.save_calibration_button = QPushButton("SAVE")
        self.save_calibration_button.clicked.connect(self.save_calibration)
        calibration_row.addWidget(self.save_calibration_button)

        self.zero_here_button = QPushButton("ZERO HERE")
        self.zero_here_button.setToolTip(
            "Place the feet at world X=0.60 m, Y=-0.60 m, Z=0.00 m; "
            "the current yaw is preserved."
        )
        self.zero_here_button.clicked.connect(self.zero_here)
        calibration_row.addWidget(self.zero_here_button)

        self.calibration_status_label = QLabel("")
        calibration_row.addWidget(self.calibration_status_label)
        calibration_row.addStretch(1)
        self.main_layout.addLayout(calibration_row)

        self.load_calibration()
        self.apply_calibration(show_status=False)

        self.render_stack = QStackedWidget()
        self.main_layout.addWidget(self.render_stack, stretch=1)

        self.opengl_view = None
        self.opengl_grid = None
        self.opengl_lines = []
        self.opengl_points = []
        self.opengl_render_rate = RenderRateMeter()
        self.matplotlib_render_rate = RenderRateMeter()
        self.opengl_render_pending = False
        self.matplotlib_render_pending = False
        if gl is not None:
            self.opengl_view = gl.GLViewWidget()
            self.opengl_view.setBackgroundColor((24, 26, 28, 255))
            self.opengl_view.opts["center"] = QVector3D(0, 0, 0)
            self.opengl_view.setCameraPosition(
                distance=350, elevation=12, azimuth=-70
            )

            self.opengl_grid = gl.GLGridItem()
            self.opengl_grid.setSize(220, 220)
            self.opengl_grid.setSpacing(20, 20)
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

            self.render_stack.addWidget(self.opengl_view)
            self.opengl_view.frameSwapped.connect(
                self.record_opengl_frame
            )

        self.matplotlib_widget = QWidget()
        matplotlib_layout = QVBoxLayout(self.matplotlib_widget)
        matplotlib_layout.setContentsMargins(0, 0, 0, 0)
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        matplotlib_layout.addWidget(self.canvas)
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.canvas.mpl_connect(
            "draw_event", self.record_matplotlib_frame
        )
        self.render_stack.addWidget(self.matplotlib_widget)

        self.ax.set_xlim(-110, 110)
        self.ax.set_ylim(-110, 110)
        self.ax.set_zlim(110, -110)
        self.ax.set_xlabel("X [cm]")
        self.ax.set_ylabel("Y [cm]")
        self.ax.set_zlabel("Z [cm]")
        self.ax.set_title("NANSENSE Live Skeleton")

        try:
            self.ax.set_box_aspect((1, 1, 1))
        except AttributeError:
            pass

        # Keep the same camera direction as the screenshot you approved.
        self.ax.view_init(elev=12, azim=-70)

        max_chains = max(len(chains) for chains in VIEW_CHAINS.values())
        self.line_artists = []
        for _ in range(max_chains):
            line, = self.ax.plot([], [], [], marker="o", linewidth=2, markersize=4)
            self.line_artists.append(line)

        self.canvas.draw_idle()

        self.change_renderer()

        self.timer = QTimer(self)
        self.timer.setInterval(33)  # ~30 FPS visualization
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def calibration_values(self):
        return {
            key: spin.value()
            for key, spin in self.calibration_spins.items()
        }

    def apply_calibration(self, _checked=False, show_status=True):
        if self.calibration_callback is not None:
            self.calibration_callback(self.calibration_values())
        if show_status:
            self.calibration_status_label.setText("Applied")

    def load_calibration(self):
        if not self.calibration_path.exists():
            self.calibration_status_label.setText("Not saved")
            return

        values = dict(CALIBRATION_DEFAULTS)
        try:
            for raw_line in self.calibration_path.read_text().splitlines():
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                key, separator, raw_value = line.partition(":")
                if separator and key.strip() in values:
                    values[key.strip()] = float(raw_value.strip())
        except (OSError, ValueError) as exc:
            self.calibration_status_label.setText(f"Load error: {exc}")
            return

        for key, value in values.items():
            self.calibration_spins[key].setValue(value)
        self.calibration_status_label.setText("Loaded")

    def save_calibration(self):
        self.apply_calibration(show_status=False)
        values = self.calibration_values()
        text = (
            "# NANSENSE origin pose in the ROS world frame\n"
            f"x_m: {values['x_m']:.6f}\n"
            f"y_m: {values['y_m']:.6f}\n"
            f"z_m: {values['z_m']:.6f}\n"
            f"yaw_deg: {values['yaw_deg']:.6f}\n"
        )
        temporary_path = self.calibration_path.with_suffix(".yaml.tmp")
        try:
            self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path.write_text(text)
            os.replace(temporary_path, self.calibration_path)
        except OSError as exc:
            self.calibration_status_label.setText(f"Save error: {exc}")
            return
        self.calibration_status_label.setText("Saved")

    def zero_here(self):
        frame = self.receiver.get_latest()
        if frame is None:
            self.calibration_status_label.setText("No NANSENSE frame")
            return

        try:
            values = zero_calibration_from_feet(
                frame,
                ZERO_TARGET_M,
                self.calibration_spins["yaw_deg"].value(),
            )
        except ValueError:
            self.calibration_status_label.setText("Both feet required")
            return

        for key, value in values.items():
            self.calibration_spins[key].setValue(value)
        self.apply_calibration(show_status=False)
        self.calibration_status_label.setText("Zero applied; press SAVE")

    def toggle_connection(self):
        if self.receiver.running:
            self.disconnect_nansense()
        else:
            self.connect_nansense()

    def connect_nansense(self):
        try:
            self.receiver.start()
        except OSError as exc:
            self.status_label.setText(f"Connection error: {exc}")
            return

        self.connect_button.setText("DISCONNECT")
        self.status_label.setText(f"Listening UDP :{UDP_PORT}")

    def disconnect_nansense(self):
        self.receiver.stop()
        if self.frame_callback is not None:
            self.frame_callback(None)
        self.connect_button.setText("CONNECT NANSENSE")
        self.status_label.setText("Disconnected")
        self.clear_plot()
        if self.current_renderer() == "matplotlib":
            self.canvas.draw_idle()

    def current_renderer(self):
        return self.renderer_combo.currentData()

    def record_opengl_frame(self, *_args):
        if self.opengl_render_pending:
            self.opengl_render_rate.record()
            self.opengl_render_pending = False

    def record_matplotlib_frame(self, *_args):
        if self.matplotlib_render_pending:
            self.matplotlib_render_rate.record()
            self.matplotlib_render_pending = False

    def change_renderer(self, _index=None):
        use_opengl = self.current_renderer() == "opengl" and self.opengl_view
        self.render_stack.setCurrentWidget(
            self.opengl_view if use_opengl else self.matplotlib_widget
        )
        self.clear_plot()
        self.opengl_render_rate.reset()
        self.matplotlib_render_rate.reset()
        self.opengl_render_pending = False
        self.matplotlib_render_pending = False
        self.reset_view()
        if not use_opengl:
            self.canvas.draw_idle()

    def reset_view(self):
        frame = self.receiver.get_latest()
        if self.current_renderer() == "opengl" and self.opengl_view:
            center = QVector3D(0, 0, 0)
            distance = 350.0
            if frame is not None:
                xyz = np.asarray([
                    display_position(frame, joint)
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
                distance=distance, elevation=12, azimuth=-70
            )
            self.opengl_view.update()
        else:
            self.ax.set_xlim(-110, 110)
            self.ax.set_ylim(-110, 110)
            self.ax.set_zlim(110, -110)
            self.ax.view_init(elev=12, azim=-70)
            self.canvas.draw_idle()

    def clear_plot(self):
        for line in self.line_artists:
            line.set_data([], [])
            line.set_3d_properties([])
        for line, points in zip(self.opengl_lines, self.opengl_points):
            empty = np.empty((0, 3), dtype=float)
            line.setData(pos=empty)
            points.setData(pos=empty)

    def update_plot(self):
        if not self.receiver.running:
            return

        frame = self.receiver.get_latest()
        if frame is None:
            self.status_label.setText(f"Waiting for UDP :{UDP_PORT}...")
            return

        if self.frame_callback is not None:
            self.frame_callback(frame)

        chains = VIEW_CHAINS[self.view_combo.currentText()]
        self.clear_plot()

        positions = []
        for chain in chains:
            available_chain = [joint for joint in chain if joint in frame["joints"]]
            if len(available_chain) < 2:
                positions.append(np.empty((0, 3), dtype=float))
                continue

            positions.append(np.asarray([
                display_position(frame, joint) for joint in available_chain
            ], dtype=float))

        if self.current_renderer() == "opengl" and self.opengl_view:
            self.opengl_render_pending = True
            for line, points, xyz in zip(
                self.opengl_lines, self.opengl_points, positions
            ):
                scene_xyz = opengl_scene_positions(xyz)
                line.setData(pos=scene_xyz)
                points.setData(pos=scene_xyz)

            all_xyz = np.asarray([
                display_position(frame, joint)
                for joint in BODY_JOINTS
                if joint in frame["joints"]
            ], dtype=float)
            scene_xyz = opengl_scene_positions(all_xyz)
            if len(scene_xyz) and self.opengl_grid is not None:
                floor_z = float(scene_xyz[:, 2].min()) - 5.0
                self.opengl_grid.resetTransform()
                self.opengl_grid.translate(0, 0, floor_z)
        else:
            self.matplotlib_render_pending = True
            for line, xyz in zip(self.line_artists, positions):
                if len(xyz) < 2:
                    continue
                line.set_data(xyz[:, 0], xyz[:, 1])
                line.set_3d_properties(xyz[:, 2])

        age_ms = (time.time() - frame["receive_time"]) * 1000.0
        render_rate = (
            self.opengl_render_rate.rate()
            if self.current_renderer() == "opengl"
            else self.matplotlib_render_rate.rate()
        )
        self.status_label.setText(
            f"UDP {self.receiver.rate:.1f} Hz | "
            f"render {render_rate:.1f} FPS | frame age {age_ms:.1f} ms"
        )
        if self.current_renderer() == "matplotlib":
            self.canvas.draw_idle()

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
