#!/usr/bin/env python3

import csv
import ipaddress
import glob
import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Point, WrenchStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from ur_dashboard_msgs.msg import RobotMode
from visualization_msgs.msg import Marker, MarkerArray

from PySide6.QtCore import QProcess, QSettings, QTimer, Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import (
    QApplication,
    QAbstractScrollArea,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QStyle,
    QVBoxLayout,
    QWidget,
)

from ur7e_tools.nansense_live_widget import (
    BODY_JOINTS,
    VIEW_CHAINS,
    NansenseLiveWidget,
)


FORCE_BAR_LIMIT = 50.0
TORQUE_BAR_LIMIT = 5.0
WRENCH_UI_REFRESH_MS = 50
WRENCH_STALE_SEC = 0.5

NANSENSE_MARKER_TOPIC = "/nansense/skeleton_markers"
NANSENSE_RAW_TOPIC = "/nansense/raw_frame"
NANSENSE_MARKER_FRAME = "world"
NANSENSE_MARKER_LIFETIME_SEC = 0.2

# Initial, deliberately neutral calibration. These values describe the
# NANSENSE origin in the ROS world frame and will become user-adjustable in
# the calibration step after the first RViz geometry/orientation check.
NANSENSE_DEFAULT_CALIBRATION = {
    "x_m": 0.0,
    "y_m": 0.0,
    "z_m": 0.0,
    "yaw_deg": 0.0,
    "mirror_lateral": True,
}

# Workcell health-gate timing.
HEALTH_PROBE_PERIOD_SEC = 0.5
HEALTH_SNAPSHOT_STALE_SEC = 2.0
CONTROLLER_RESPONSE_STALE_SEC = 8.0
PROGRAM_CONTROLLER_TRANSITION_GRACE_SEC = 2.0
CONTROLLER_PROBE_INTERVAL_SEC = 0.75
HEALTH_STARTUP_GRACE_SEC = 20.0

EXPECTED_GRAPH_NODES = (
    "/workcell/robot_state_publisher",
    "/robot1/robot_state_publisher",
    "/robot2/robot_state_publisher",
)

TF_BASE_FRAMES = {
    "robot1": "robot1_base",
    "robot2": "robot2_base",
}

SUPPORT_CONTROLLERS = (
    "joint_state_broadcaster",
    "io_and_status_controller",
    "speed_scaling_state_broadcaster",
    "force_torque_sensor_broadcaster",
    "tcp_pose_broadcaster",
    "ur_configuration_controller",
    "friction_model_controller",
)

MOTION_CONTROLLERS = (
    "joint_trajectory_controller",
    "scaled_joint_trajectory_controller",
    "forward_velocity_controller",
    "forward_position_controller",
)

PRIMARY_MOTION_CONTROLLER = "joint_trajectory_controller"


# -------------------------------------------------------------------------
# Workcell supervisor preflight
# -------------------------------------------------------------------------
#
# The UI itself owns one ROS 2 participant (the wrench/program listener).
# Therefore FastDDS cleanup must happen BEFORE rclpy.init().
#
# We only remove FastDDS shared-memory files when no workcell ROS processes
# are running. This prevents a stale previous run from contaminating the next
# UI session while avoiding deletion of resources used by a live workcell.
#
WORKCELL_PROCESS_MARKERS = (
    "dual_ur7e.launch.py",
    "single_ur5.launch.py",
    "ur_ros2_control_node",
    "controller_stopper_node",
    "robot_state_publisher",
    "dashboard_client",
    "urscript_interface",
    "trajectory_until_node",
    "gripper_visualizer",
    "rviz2",
    "ft_sensor",
    "/controller_manager/spawner",
)


def _process_cmdline(pid):
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (OSError, PermissionError):
        return ""

    return raw.replace(b"\0", b" ").decode(
        "utf-8",
        errors="replace",
    ).strip()


def find_running_workcell_processes():
    """Return live ROS/workcell processes, excluding this UI process."""

    current_pid = os.getpid()
    found = []

    for proc_path in Path("/proc").iterdir():
        if not proc_path.name.isdigit():
            continue

        pid = int(proc_path.name)
        if pid == current_pid:
            continue

        cmdline = _process_cmdline(pid)
        if not cmdline:
            continue

        if any(marker in cmdline for marker in WORKCELL_PROCESS_MARKERS):
            found.append((pid, cmdline))

    return sorted(found, key=lambda item: item[0])


def _fastdds_shared_memory_paths():
    patterns = (
        "/dev/shm/fastrtps_*",
        "/dev/shm/sem.fastrtps_*",
        "/dev/shm/fastdds_*",
        "/dev/shm/sem.fastdds_*",
    )

    paths = set()
    for pattern in patterns:
        paths.update(glob.glob(pattern))

    return sorted(paths)


def stop_ros2_daemon_quietly():
    """Stop only the ROS 2 CLI daemon; never kill arbitrary ROS processes."""

    ros2_executable = shutil.which("ros2")
    if not ros2_executable:
        return False, "ros2 executable not found"

    try:
        result = subprocess.run(
            [ros2_executable, "daemon", "stop"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)

    return result.returncode == 0, result.stdout.strip()


def perform_startup_fastdds_preflight():
    """Safely clean stale FastDDS SHM before the UI creates its ROS node."""

    report = {
        "state": "CLEAN",
        "message": "No stale FastDDS state detected.",
        "removed": 0,
        "blocked_processes": [],
    }

    # The ros2 CLI daemon is not part of the robot workcell. Stop it first so
    # it cannot keep old FastDDS shared-memory ports locked during cleanup.
    stop_ros2_daemon_quietly()
    time.sleep(0.2)

    # Cleanup is allowed only when no workcell ROS process is alive.
    blockers = find_running_workcell_processes()
    if blockers:
        report["state"] = "BLOCKED"
        report["blocked_processes"] = blockers
        report["message"] = (
            "A workcell ROS process is running; "
            "FastDDS cleanup was not attempted."
        )
        return report

    removed = 0
    errors = []

    for path_string in _fastdds_shared_memory_paths():
        path = Path(path_string)

        try:
            # Only remove files owned by the current Linux user.
            if path.stat().st_uid != os.getuid():
                continue
            path.unlink()
            removed += 1
        except FileNotFoundError:
            pass
        except OSError as exc:
            errors.append(f"{path.name}: {exc}")

    report["removed"] = removed

    if errors:
        report["state"] = "WARNING"
        report["message"] = (
            f"Removed {removed} stale FastDDS file(s), "
            f"but {len(errors)} item(s) could not be removed."
        )
    elif removed:
        report["state"] = "CLEANED"
        report["message"] = (
            f"Removed {removed} stale FastDDS shared-memory file(s)."
        )

    return report


class CenteredBar(QWidget):
    """Simple center-zero bar used for live signed wrench values."""

    def __init__(self, max_abs, parent=None):
        super().__init__(parent)
        self.max_abs = float(max_abs)
        self.value = 0.0

        self.setMinimumHeight(16)
        self.setMinimumWidth(110)
        self.setToolTip(f"Visual scale: ±{self.max_abs:g}")

    def set_value(self, value):
        self.value = float(value)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)

        rect = self.rect().adjusted(1, 2, -1, -2)

        # Keep the bars explicitly dark regardless of the desktop theme.
        painter.fillRect(
            rect,
            QColor("#303134"),
        )

        painter.setPen(
            QColor("#5f6368")
        )
        painter.drawRect(rect)

        center_x = rect.left() + rect.width() // 2

        painter.setPen(
            QColor("#e8eaed")
        )
        painter.drawLine(
            center_x,
            rect.top(),
            center_x,
            rect.bottom(),
        )

        if self.max_abs <= 0.0:
            return

        fraction = min(
            abs(self.value) / self.max_abs,
            1.0,
        )

        half_width = rect.width() / 2.0
        fill_width = int(half_width * fraction)

        fill_color = QColor("#8ab4f8")

        if self.value >= 0.0:
            fill_rect = rect.adjusted(
                rect.width() // 2,
                2,
                -(rect.width() // 2 - fill_width),
                -2,
            )
        else:
            fill_rect = rect.adjusted(
                rect.width() // 2 - fill_width,
                2,
                -(rect.width() // 2),
                -2,
            )

        if fill_width > 0:
            painter.fillRect(
                fill_rect,
                fill_color,
            )


class CollapsibleSection(QWidget):
    """Compact dark collapsible section with an arrow in its header."""

    def __init__(self, title, expanded=False, parent=None):
        super().__init__(parent)

        self.title = title

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)

        self.toggle_button = QPushButton()
        self.toggle_button.setObjectName("collapseButton")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(bool(expanded))
        self.toggle_button.setToolTip(
            f"Expand or collapse {title}."
        )
        self.toggle_button.toggled.connect(
            self.set_expanded
        )

        layout.addWidget(self.toggle_button)

        self.body = QFrame()
        self.body.setObjectName("collapsibleBody")
        self.body_layout = QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(8, 6, 8, 7)
        self.body_layout.setSpacing(5)

        layout.addWidget(self.body)

        self.set_expanded(bool(expanded))

    def set_expanded(self, expanded):
        self.toggle_button.blockSignals(True)
        self.toggle_button.setChecked(bool(expanded))
        self.toggle_button.blockSignals(False)

        self.body.setVisible(bool(expanded))
        self.toggle_button.setText(
            f"▼  {self.title}"
            if expanded
            else f"▶  {self.title}"
        )


class WrenchListenerNode(Node):
    """Background ROS listener and selectable-rate CSV recorder for wrench streams."""

    TOPICS = {
        "robot1": "/robot1/force_torque_sensor_broadcaster/wrench",
        "robot2": "/robot2/force_torque_sensor_broadcaster/wrench",
        "external": "/external_ft",
    }

    PROGRAM_TOPICS = {
        "robot1": "/robot1/io_and_status_controller/robot_program_running",
        "robot2": "/robot2/io_and_status_controller/robot_program_running",
    }

    ROBOT_MODE_TOPICS = {
        "robot1": "/robot1/io_and_status_controller/robot_mode",
        "robot2": "/robot2/io_and_status_controller/robot_mode",
    }

    # All three currently verified wrench streams run at approximately 100 Hz.
    # Lower CSV rates are obtained by deterministic sample decimation while
    # leaving the ROS acquisition/control streams untouched at full rate.
    NOMINAL_SOURCE_RATE_HZ = 100

    def __init__(self):
        super().__init__("workcell_ui_wrench_listener")

        self._lock = threading.Lock()
        self._latest = {}
        self._program_states = {}
        self._robot_modes = {}
        self._recordings = {}
        self._subscriptions = []
        self._nansense_lock = threading.Lock()
        self._latest_nansense_frame = None
        self._nansense_calibration = dict(NANSENSE_DEFAULT_CALIBRATION)
        self._nansense_marker_publisher = self.create_publisher(
            MarkerArray,
            NANSENSE_MARKER_TOPIC,
            1,
        )
        self._nansense_raw_publisher = self.create_publisher(
            String,
            NANSENSE_RAW_TOPIC,
            50,
        )

        # Health probes run inside this ROS node's spin thread so the GUI never
        # blocks on controller-manager, graph, or TF calls.
        self._health_enabled = False
        self._health_reset_requested = False
        self._health_state = self._new_health_state()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )

        self._controller_clients = {
            robot: self.create_client(
                ListControllers,
                f"/{robot}/controller_manager/list_controllers",
            )
            for robot in ("robot1", "robot2")
        }
        self._controller_futures = {
            "robot1": None,
            "robot2": None,
        }
        self._controller_last_request = {
            "robot1": 0.0,
            "robot2": 0.0,
        }
        self._controller_request_started = {
            "robot1": None,
            "robot2": None,
        }
        self._controller_cache = {
            robot: self._new_controller_health()
            for robot in ("robot1", "robot2")
        }

        for key, topic in self.TOPICS.items():
            subscription = self.create_subscription(
                WrenchStamped,
                topic,
                lambda msg, sensor_key=key:
                    self._wrench_callback(sensor_key, msg),
                qos_profile_sensor_data,
            )
            self._subscriptions.append(subscription)

        # Use VOLATILE depth-1 subscriptions for runtime robot state.
        # Readiness accepts only samples received after START SYSTEM, so a
        # retained state from an older launch cannot make the UI READY.
        for key, topic in self.PROGRAM_TOPICS.items():
            subscription = self.create_subscription(
                Bool,
                topic,
                lambda msg, robot_key=key:
                    self._program_running_callback(robot_key, msg),
                1,
            )
            self._subscriptions.append(subscription)

        # robot_program_running may not publish a sample until the PolyScope
        # program is actually started. robot_mode is available earlier, so it
        # lets the supervisor distinguish "driver/controller is alive and
        # waiting for PLAY" from "robot state has not appeared yet".
        for key, topic in self.ROBOT_MODE_TOPICS.items():
            subscription = self.create_subscription(
                RobotMode,
                topic,
                lambda msg, robot_key=key:
                    self._robot_mode_callback(robot_key, msg),
                1,
            )
            self._subscriptions.append(subscription)

        self._health_timer = self.create_timer(
            HEALTH_PROBE_PERIOD_SEC,
            self._health_timer_callback,
        )
        self._nansense_marker_timer = self.create_timer(
            1.0 / 30.0,
            self._publish_latest_nansense_markers,
        )

    def publish_nansense_raw_frame(self, frame):
        """Publish one complete parsed NANSENSE UDP frame for rosbag logging."""
        if frame is None:
            return

        import json

        msg = String()
        msg.data = json.dumps(
            frame,
            separators=(",", ":"),
            ensure_ascii=False,
        )
        self._nansense_raw_publisher.publish(msg)

    def update_nansense_frame(self, frame):
        with self._nansense_lock:
            self._latest_nansense_frame = frame

    def update_nansense_calibration(self, calibration):
        with self._nansense_lock:
            self._nansense_calibration = {
                "x_m": float(calibration["x_m"]),
                "y_m": float(calibration["y_m"]),
                "z_m": float(calibration["z_m"]),
                "yaw_deg": float(calibration["yaw_deg"]),
                "mirror_lateral": bool(calibration["mirror_lateral"]),
            }

    def _publish_latest_nansense_markers(self):
        with self._nansense_lock:
            frame = self._latest_nansense_frame
            calibration = dict(self._nansense_calibration)
        if frame is not None:
            self.publish_nansense_markers(frame, calibration)

    @staticmethod
    def _nansense_point_in_world(position_world_cm, calibration):
        """Map NANSENSE viewer axes to ROS metres, then apply calibration."""
        # The accepted upright viewer convention is lateral PX, depth PZ,
        # vertical PY. This changes only the published representation; the
        # parsed NANSENSE values remain untouched.
        lateral_sign = -1.0 if calibration["mirror_lateral"] else 1.0
        x_m = lateral_sign * position_world_cm[0] * 0.01
        y_m = position_world_cm[2] * 0.01
        z_m = position_world_cm[1] * 0.01

        yaw = math.radians(calibration["yaw_deg"])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return Point(
            x=(cos_yaw * x_m - sin_yaw * y_m)
              + calibration["x_m"],
            y=(sin_yaw * x_m + cos_yaw * y_m)
              + calibration["y_m"],
            z=z_m + calibration["z_m"],
        )

    def publish_nansense_markers(self, frame, calibration):
        joints = frame.get("joints", {})
        if not joints:
            return

        stamp = self.get_clock().now().to_msg()
        lifetime = Duration(
            seconds=NANSENSE_MARKER_LIFETIME_SEC
        ).to_msg()

        joint_marker = Marker()
        joint_marker.header.frame_id = NANSENSE_MARKER_FRAME
        joint_marker.header.stamp = stamp
        joint_marker.ns = "nansense_joints"
        joint_marker.id = 0
        joint_marker.type = Marker.SPHERE_LIST
        joint_marker.action = Marker.ADD
        joint_marker.pose.orientation.w = 1.0
        joint_marker.scale.x = 0.04
        joint_marker.scale.y = 0.04
        joint_marker.scale.z = 0.04
        joint_marker.color.r = 1.0
        joint_marker.color.g = 0.65
        joint_marker.color.b = 0.15
        joint_marker.color.a = 1.0
        joint_marker.lifetime = lifetime
        joint_marker.points = [
            self._nansense_point_in_world(
                joints[name]["position_world_cm"], calibration
            )
            for name in BODY_JOINTS
            if name in joints
        ]

        bone_marker = Marker()
        bone_marker.header.frame_id = NANSENSE_MARKER_FRAME
        bone_marker.header.stamp = stamp
        bone_marker.ns = "nansense_bones"
        bone_marker.id = 1
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.pose.orientation.w = 1.0
        bone_marker.scale.x = 0.025
        bone_marker.color.r = 0.2
        bone_marker.color.g = 0.75
        bone_marker.color.b = 1.0
        bone_marker.color.a = 1.0
        bone_marker.lifetime = lifetime

        for chain in VIEW_CHAINS["Full Body"]:
            for parent_name, child_name in zip(chain, chain[1:]):
                if parent_name not in joints or child_name not in joints:
                    continue
                bone_marker.points.extend([
                    self._nansense_point_in_world(
                        joints[parent_name]["position_world_cm"], calibration
                    ),
                    self._nansense_point_in_world(
                        joints[child_name]["position_world_cm"], calibration
                    ),
                ])

        self._nansense_marker_publisher.publish(
            MarkerArray(markers=[
                joint_marker,
                bone_marker,
                *self._nansense_hand_markers(
                    joints, stamp, lifetime, calibration
                ),
            ])
        )

    def _nansense_hand_markers(self, joints, stamp, lifetime, calibration):
        """Add unambiguous semantic L/R cues independent of camera angle."""
        markers = []
        for marker_id, (joint_name, label, color) in enumerate((
            ("LeftHand", "L", (1.0, 0.15, 0.15)),
            ("RightHand", "R", (0.15, 1.0, 0.35)),
        ), start=10):
            if joint_name not in joints:
                continue
            point = self._nansense_point_in_world(
                joints[joint_name]["position_world_cm"], calibration
            )

            hand = Marker()
            hand.header.frame_id = NANSENSE_MARKER_FRAME
            hand.header.stamp = stamp
            hand.ns = "nansense_hand_identity"
            hand.id = marker_id
            hand.type = Marker.SPHERE
            hand.action = Marker.ADD
            hand.pose.position = point
            hand.pose.orientation.w = 1.0
            hand.scale.x = hand.scale.y = hand.scale.z = 0.09
            hand.color.r, hand.color.g, hand.color.b = color
            hand.color.a = 1.0
            hand.lifetime = lifetime
            markers.append(hand)

            text = Marker()
            text.header.frame_id = NANSENSE_MARKER_FRAME
            text.header.stamp = stamp
            text.ns = "nansense_hand_labels"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = point.x
            text.pose.position.y = point.y
            text.pose.position.z = point.z + 0.12
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r, text.color.g, text.color.b = color
            text.color.a = 1.0
            text.text = label
            text.lifetime = lifetime
            markers.append(text)
        return markers

    def _program_running_callback(self, key, msg):
        with self._lock:
            self._program_states[key] = (
                time.monotonic(),
                bool(msg.data),
            )

    def program_snapshot(self):
        with self._lock:
            return dict(self._program_states)

    def _robot_mode_callback(self, key, msg):
        with self._lock:
            self._robot_modes[key] = (
                time.monotonic(),
                int(msg.mode),
            )

    def robot_mode_snapshot(self):
        with self._lock:
            return dict(self._robot_modes)

    # -----------------------------------------------------
    # Workcell health probes
    # -----------------------------------------------------

    @staticmethod
    def _new_controller_health():
        return {
            "service_ready": False,
            "response_stamp": None,
            "states": {},
            "error": "",
            "pending_since": None,
        }

    @classmethod
    def _new_health_state(cls):
        return {
            "stamp": None,
            "graph_ok": False,
            "graph_missing": list(EXPECTED_GRAPH_NODES),
            "graph_duplicates": [],
            "tf_ok": {
                "robot1": False,
                "robot2": False,
            },
            "controllers": {
                "robot1": cls._new_controller_health(),
                "robot2": cls._new_controller_health(),
            },
        }

    def request_health_reset(self):
        # The actual reset is performed by the ROS spin thread on the next
        # health timer tick. This also clears the TF buffer so old transforms
        # from a previous UI-owned launch cannot satisfy a new session.
        with self._lock:
            self._health_enabled = False
            self._health_reset_requested = True
            self._health_state = self._new_health_state()

    def set_health_monitor_enabled(self, enabled):
        with self._lock:
            self._health_enabled = bool(enabled)

    def health_snapshot(self):
        with self._lock:
            state = self._health_state
            return {
                "stamp": state["stamp"],
                "graph_ok": bool(state["graph_ok"]),
                "graph_missing": list(state["graph_missing"]),
                "graph_duplicates": list(state["graph_duplicates"]),
                "tf_ok": dict(state["tf_ok"]),
                "controllers": {
                    robot: {
                        "service_ready": bool(
                            state["controllers"][robot]["service_ready"]
                        ),
                        "response_stamp": (
                            state["controllers"][robot]["response_stamp"]
                        ),
                        "states": dict(
                            state["controllers"][robot]["states"]
                        ),
                        "error": state["controllers"][robot]["error"],
                        "pending_since": (
                            state["controllers"][robot]["pending_since"]
                        ),
                    }
                    for robot in ("robot1", "robot2")
                },
            }

    def _apply_health_reset(self):
        try:
            clear = getattr(self._tf_buffer, "clear", None)
            if callable(clear):
                clear()
        except Exception:
            # The state is still reset below. A missing/unclear TF path will
            # remain fail-closed until current-session TF is observed again.
            pass

        self._controller_futures = {
            "robot1": None,
            "robot2": None,
        }
        self._controller_last_request = {
            "robot1": 0.0,
            "robot2": 0.0,
        }
        self._controller_request_started = {
            "robot1": None,
            "robot2": None,
        }
        self._controller_cache = {
            robot: self._new_controller_health()
            for robot in ("robot1", "robot2")
        }

    def _health_timer_callback(self):
        with self._lock:
            reset_requested = self._health_reset_requested
            enabled = self._health_enabled
            if reset_requested:
                self._health_reset_requested = False

        if reset_requested:
            self._apply_health_reset()

        if not enabled:
            return

        now = time.monotonic()

        # ROS graph: every required robot_state_publisher must appear exactly
        # once. This deliberately rejects duplicate stale graph entries.
        node_counts = {}
        try:
            for node_name, namespace in self.get_node_names_and_namespaces():
                namespace = namespace or "/"
                if namespace == "/":
                    full_name = f"/{node_name}"
                else:
                    full_name = (
                        f"{namespace.rstrip('/')}/{node_name}"
                    )
                node_counts[full_name] = node_counts.get(full_name, 0) + 1
        except Exception:
            node_counts = {}

        graph_missing = [
            name
            for name in EXPECTED_GRAPH_NODES
            if node_counts.get(name, 0) == 0
        ]
        graph_duplicates = [
            name
            for name in EXPECTED_GRAPH_NODES
            if node_counts.get(name, 0) > 1
        ]
        graph_ok = not graph_missing and not graph_duplicates

        # TF: require a current transform path for each robot base.
        tf_ok = {}
        for robot, base_frame in TF_BASE_FRAMES.items():
            try:
                tf_ok[robot] = bool(
                    self._tf_buffer.can_transform(
                        "world",
                        base_frame,
                        Time(),
                    )
                )
            except Exception:
                tf_ok[robot] = False

        # Controller managers: issue one non-blocking list_controllers call per
        # robot and never stack another request while one is still pending.
        for robot in ("robot1", "robot2"):
            self._update_controller_probe(robot, now)

        controller_snapshot = {
            robot: {
                "service_ready": bool(
                    self._controller_cache[robot]["service_ready"]
                ),
                "response_stamp": (
                    self._controller_cache[robot]["response_stamp"]
                ),
                "states": dict(
                    self._controller_cache[robot]["states"]
                ),
                "error": self._controller_cache[robot]["error"],
                "pending_since": (
                    self._controller_cache[robot]["pending_since"]
                ),
            }
            for robot in ("robot1", "robot2")
        }

        with self._lock:
            self._health_state = {
                "stamp": now,
                "graph_ok": graph_ok,
                "graph_missing": graph_missing,
                "graph_duplicates": graph_duplicates,
                "tf_ok": tf_ok,
                "controllers": controller_snapshot,
            }

    def _update_controller_probe(self, robot, now):
        client = self._controller_clients[robot]
        future = self._controller_futures[robot]
        cache = self._controller_cache[robot]

        if future is not None and future.done():
            try:
                response = future.result()
                cache["states"] = {
                    controller.name: controller.state
                    for controller in response.controller
                }
                cache["response_stamp"] = now
                cache["error"] = ""
            except Exception as exc:
                cache["error"] = str(exc)

            cache["pending_since"] = None
            self._controller_futures[robot] = None
            future = None

        if future is not None:
            cache["service_ready"] = True
            cache["pending_since"] = self._controller_request_started[robot]
            return

        if (
            now - self._controller_last_request[robot]
            < CONTROLLER_PROBE_INTERVAL_SEC
        ):
            return

        self._controller_last_request[robot] = now
        cache["service_ready"] = bool(client.service_is_ready())

        if not cache["service_ready"]:
            cache["pending_since"] = None
            if cache["response_stamp"] is None:
                cache["error"] = "list_controllers service unavailable"
            return

        try:
            future = client.call_async(ListControllers.Request())
            self._controller_futures[robot] = future
            self._controller_request_started[robot] = now
            cache["pending_since"] = now
        except Exception as exc:
            cache["error"] = str(exc)
            cache["pending_since"] = None

    def _wrench_callback(self, key, msg):
        values = (
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        )

        stamp_sec = int(msg.header.stamp.sec)
        stamp_nanosec = int(msg.header.stamp.nanosec)

        if stamp_sec == 0 and stamp_nanosec == 0:
            now_ns = self.get_clock().now().nanoseconds
            stamp_sec = int(now_ns // 1_000_000_000)
            stamp_nanosec = int(now_ns % 1_000_000_000)

        stamp_float = stamp_sec + stamp_nanosec * 1e-9

        with self._lock:
            self._latest[key] = (
                time.monotonic(),
                values,
            )

            recording = self._recordings.get(key)

            if recording is not None:
                record_rate_hz = recording["record_rate_hz"]
                should_record = True

                # Deterministic decimation from the verified ~100 Hz source.
                # This changes only the CSV rate; live ROS data stay untouched.
                if record_rate_hz < self.NOMINAL_SOURCE_RATE_HZ:
                    recording["rate_accumulator"] += record_rate_hz

                    if (
                        recording["rate_accumulator"]
                        >= self.NOMINAL_SOURCE_RATE_HZ
                    ):
                        recording["rate_accumulator"] -= (
                            self.NOMINAL_SOURCE_RATE_HZ
                        )
                    else:
                        should_record = False

                if should_record:
                    if recording["start_stamp"] is None:
                        recording["start_stamp"] = stamp_float

                    elapsed = (
                        stamp_float
                        - recording["start_stamp"]
                    )

                    row = [
                        stamp_sec,
                        stamp_nanosec,
                        f"{elapsed:.9f}",
                        f"{values[0]:.9f}",
                        f"{values[1]:.9f}",
                        f"{values[2]:.9f}",
                    ]

                    if recording["include_torque"]:
                        row.extend([
                            f"{values[3]:.9f}",
                            f"{values[4]:.9f}",
                            f"{values[5]:.9f}",
                        ])

                    try:
                        recording["writer"].writerow(row)
                        recording["samples"] += 1

                        # Flush approximately once per second.
                        if (
                            recording["samples"]
                            % max(1, record_rate_hz)
                            == 0
                        ):
                            recording["file"].flush()

                    except Exception as exc:
                        self.get_logger().error(
                            f"CSV recording error for {key}: {exc}"
                        )

    def snapshot(self):
        with self._lock:
            return dict(self._latest)

    def start_recording(self, key, path, include_torque, record_rate_hz):
        if key not in self.TOPICS:
            raise ValueError(f"Unknown wrench sensor: {key}")

        record_rate_hz = int(record_rate_hz)
        allowed_rates = (10, 20, 30, 50, 100)
        if record_rate_hz not in allowed_rates:
            raise ValueError(
                f"Unsupported recording rate: {record_rate_hz} Hz"
            )

        with self._lock:
            if key in self._recordings:
                raise RuntimeError(
                    f"{key} is already recording"
                )

            handle = open(
                path,
                "w",
                newline="",
                encoding="utf-8",
            )
            writer = csv.writer(handle)

            header = [
                "stamp_sec",
                "stamp_nanosec",
                "elapsed_s",
                "Fx_N",
                "Fy_N",
                "Fz_N",
            ]

            if include_torque:
                header.extend([
                    "Mx_Nm",
                    "My_Nm",
                    "Mz_Nm",
                ])

            writer.writerow(header)
            handle.flush()

            self._recordings[key] = {
                "file": handle,
                "writer": writer,
                "include_torque": bool(include_torque),
                "record_rate_hz": record_rate_hz,
                # Initialize so that the first received sample is kept.
                "rate_accumulator": (
                    self.NOMINAL_SOURCE_RATE_HZ - record_rate_hz
                ),
                "start_stamp": None,
                "samples": 0,
                "path": path,
            }

    def stop_recording(self, key):
        with self._lock:
            recording = self._recordings.pop(
                key,
                None,
            )

            if recording is None:
                return None

            path = recording["path"]
            recording["file"].flush()
            recording["file"].close()
            return path

    def is_recording(self, key):
        with self._lock:
            return key in self._recordings

    def stop_all_recordings(self):
        for key in tuple(self.TOPICS):
            self.stop_recording(key)


class WorkcellUI(QMainWindow):

    def __init__(self, startup_preflight_report=None):
        super().__init__()

        self.startup_preflight_report = (
            startup_preflight_report
            if startup_preflight_report is not None
            else {
                "state": "UNKNOWN",
                "message": "Startup preflight was not run.",
                "removed": 0,
                "blocked_processes": [],
            }
        )

        self.setWindowTitle("Robot Workcell Control")
        self.resize(1340, 780)

        # Persistent UI preferences.
        # QSettings stores these outside the ROS/Git workspace
        # (normally under ~/.config on Ubuntu).
        self.settings = QSettings(
            "CAOR",
            "RobotWorkcellControl",
        )
        self.default_recording_folder = os.path.expanduser(
            "~/Robot_Recordings"
        )

        # -----------------------------------------------------
        # Main ROS launch process
        # -----------------------------------------------------

        self.ros_process = QProcess(self)
        self.ros_process.setProcessChannelMode(QProcess.MergedChannels)

        self.ros_process.readyReadStandardOutput.connect(
            self.read_ros_output
        )
        self.ros_process.started.connect(
            self.on_process_started
        )
        self.ros_process.finished.connect(
            self.on_process_finished
        )

        # -----------------------------------------------------
        # Ping processes
        # -----------------------------------------------------

        self.ur5_ping_process = QProcess(self)
        self.robot1_ping_process = QProcess(self)
        self.robot2_ping_process = QProcess(self)

        self.ur5_ping_process.finished.connect(
            self.ur5_ping_finished
        )

        self.robot1_ping_process.finished.connect(
            self.robot1_ping_finished
        )

        self.robot2_ping_process.finished.connect(
            self.robot2_ping_finished
        )

        # -----------------------------------------------------
        # Setup motion process
        #
        # This single process serializes setup motions. For the
        # Dual UR7e setup it runs the saved_pose backend, whose
        # mandatory full-path workcell safety gate cannot be
        # bypassed by the UI. The legacy UR5 HOME path remains
        # available for the Single UR5 setup.
        # -----------------------------------------------------

        self.home_process = QProcess(self)
        self.home_process.setProcessChannelMode(QProcess.MergedChannels)

        self.home_process.readyReadStandardOutput.connect(
            self.read_setup_motion_output
        )
        self.home_process.finished.connect(
            self.setup_motion_finished
        )

        self.active_setup_motion = None
        self.setup_motion_output_buffer = ""

        # -----------------------------------------------------
        # 2FG7 gripper commands
        # -----------------------------------------------------

        self.gripper_process = QProcess(self)
        self.gripper_process.setProcessChannelMode(QProcess.MergedChannels)

        self.gripper_process.readyReadStandardOutput.connect(
            self.read_gripper_output
        )
        self.gripper_process.finished.connect(
            self.gripper_command_finished
        )

        self.active_gripper_command = None

        # -----------------------------------------------------
        # Experiment process
        # -----------------------------------------------------

        self.experiment_process = QProcess(self)
        self.experiment_process.setProcessChannelMode(QProcess.MergedChannels)
        self.experiment_process.readyReadStandardOutput.connect(
            self.read_experiment_output
        )
        self.experiment_process.finished.connect(
            self.experiment_finished
        )
        self.active_experiment = None

        # -----------------------------------------------------
        # Experiment analyzer process
        # -----------------------------------------------------

        self.analysis_process = QProcess(self)
        self.analysis_process.setProcessChannelMode(QProcess.MergedChannels)
        self.analysis_process.readyReadStandardOutput.connect(
            self.read_analysis_output
        )
        self.analysis_process.finished.connect(
            self.analysis_finished
        )
        self.analysis_output_buffer = ""
        self.analysis_report_path = None

        # -----------------------------------------------------
        # Per-robot readiness state for the current UI-owned launch
        # -----------------------------------------------------

        self.system_session_started_at = None
        self.robot_ready = {
            "robot1": False,
            "robot2": False,
        }
        self.robot_reverse_ready_seen = {
            "robot1": False,
            "robot2": False,
        }
        self.robot_core_health = {
            "robot1": "checking",
            "robot2": "checking",
        }
        self._ros_output_parse_buffer = ""

        # -----------------------------------------------------
        # External Robotiq F/T process
        # -----------------------------------------------------

        self.ft_process = QProcess(self)
        self.ft_process.setProcessChannelMode(
            QProcess.MergedChannels
        )
        self.ft_process.readyReadStandardOutput.connect(
            self.read_external_ft_output
        )
        self.ft_process.started.connect(
            self.external_ft_process_started
        )
        self.ft_process.finished.connect(
            self.external_ft_process_finished
        )

        self.ft_zero_process = QProcess(self)
        self.ft_zero_process.setProcessChannelMode(
            QProcess.MergedChannels
        )
        self.ft_zero_process.readyReadStandardOutput.connect(
            self.read_external_ft_zero_output
        )
        self.ft_zero_process.finished.connect(
            self.external_ft_zero_finished
        )

        self.external_ft_stopping = False
        self.external_ft_live = False

        # -----------------------------------------------------
        # Internal UR F/T zero service processes
        # -----------------------------------------------------

        self.internal_ft_zero_processes = {}

        for sensor_key in ("robot1", "robot2"):
            process = QProcess(self)
            process.setProcessChannelMode(
                QProcess.MergedChannels
            )

            process.readyReadStandardOutput.connect(
                lambda sensor_key=sensor_key:
                    self.read_internal_ft_zero_output(
                        sensor_key
                    )
            )

            process.finished.connect(
                lambda exit_code, exit_status, sensor_key=sensor_key:
                    self.internal_ft_zero_finished(
                        sensor_key,
                        exit_code,
                        exit_status,
                    )
            )

            self.internal_ft_zero_processes[
                sensor_key
            ] = process

        # -----------------------------------------------------
        # ROS wrench listener (display only)
        # -----------------------------------------------------

        self._owns_rclpy_context = False

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True

        self.wrench_listener = WrenchListenerNode()

        self.wrench_spin_thread = threading.Thread(
            target=rclpy.spin,
            args=(self.wrench_listener,),
            name="workcell_ui_wrench_spin",
            daemon=True,
        )
        self.wrench_spin_thread.start()

        self.build_ui()
        self.apply_style()
        self.update_setup_view()
        self.apply_startup_preflight_report()

        # UI display is intentionally throttled to 20 Hz.
        # The ROS topics themselves remain at their native rates (~100 Hz).
        self.wrench_refresh_timer = QTimer(self)
        self.wrench_refresh_timer.setInterval(
            WRENCH_UI_REFRESH_MS
        )
        self.wrench_refresh_timer.timeout.connect(
            self.refresh_wrench_display
        )
        self.wrench_refresh_timer.start()

        # Robot READY state is refreshed independently from the wrench UI.
        self.robot_state_timer = QTimer(self)
        self.robot_state_timer.setInterval(100)
        self.robot_state_timer.timeout.connect(
            self.refresh_robot_readiness
        )
        self.robot_state_timer.start()

    # =========================================================
    # UI
    # =========================================================

    def build_ui(self):

        central = QWidget()
        central.setObjectName("centralWidget")
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(10, 8, 10, 8)
        main_layout.setSpacing(8)

        # -----------------------------------------------------
        # Title
        # -----------------------------------------------------

        title = QLabel("Robot Workcell Control")
        title.setObjectName("title")

        subtitle = QLabel(
            "ROS 2 control interface for laboratory robot setups"
        )
        subtitle.setObjectName("subtitle")

        main_layout.addWidget(title)
        main_layout.addWidget(subtitle)

        # -----------------------------------------------------
        # Setup selection + main system controls
        # -----------------------------------------------------

        setup_group = QGroupBox("System configuration")

        setup_group_layout = QVBoxLayout(setup_group)
        setup_group_layout.setSpacing(4)

        setup_layout = QHBoxLayout()
        setup_layout.setSpacing(8)

        setup_group_layout.addLayout(setup_layout)

        setup_layout.addWidget(QLabel("Setup:"))

        self.setup_combo = QComboBox()
        self.setup_combo.addItems([
            "Single UR5",
            "Dual UR7e",
        ])
        self.setup_combo.setCurrentText("Dual UR7e")

        self.setup_combo.currentIndexChanged.connect(
            self.update_setup_view
        )

        setup_layout.addWidget(self.setup_combo)

        setup_layout.addWidget(QLabel("Mode:"))

        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "Simulation",
            "Real Robot(s)",
        ])
        self.mode_combo.setCurrentText("Real Robot(s)")

        self.mode_combo.currentIndexChanged.connect(
            self.update_setup_view
        )

        setup_layout.addWidget(self.mode_combo)
        setup_layout.addSpacing(8)

        self.start_button = QPushButton(
            "START SYSTEM"
        )
        self.start_button.setObjectName(
            "startButton"
        )
        self.start_button.clicked.connect(
            self.start_system
        )

        self.stop_button = QPushButton(
            "STOP SYSTEM"
        )
        self.stop_button.setObjectName(
            "stopButton"
        )
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(
            self.stop_system
        )

        setup_layout.addWidget(self.start_button)
        setup_layout.addWidget(self.stop_button)
        setup_layout.addStretch()

        setup_layout.addWidget(QLabel("Status:"))

        self.status_label = QLabel(
            "STOPPED"
        )
        self.status_label.setObjectName(
            "statusStopped"
        )

        setup_layout.addWidget(self.status_label)

        setup_layout.addSpacing(6)
        setup_layout.addWidget(QLabel("Health:"))

        self.health_status_label = QLabel("STOPPED")
        self.health_status_label.setObjectName(
            "connectionUnknown"
        )
        self.health_status_label.setToolTip(
            "ROS graph, TF, controller-manager, controller state, "
            "robot mode, and readiness health gates."
        )
        setup_layout.addWidget(self.health_status_label)

        setup_layout.addSpacing(6)
        setup_layout.addWidget(QLabel("Robots:"))

        self.robots_ready_label = QLabel("NOT READY")
        self.robots_ready_label.setObjectName(
            "robotSummaryUnknown"
        )
        setup_layout.addWidget(self.robots_ready_label)

        setup_layout.addSpacing(6)
        setup_layout.addWidget(QLabel("Preflight:"))

        self.preflight_status_label = QLabel("UNKNOWN")
        self.preflight_status_label.setObjectName(
            "connectionUnknown"
        )
        self.preflight_status_label.setToolTip(
            "Startup check for leftover workcell processes and stale "
            "FastDDS shared-memory state."
        )
        setup_layout.addWidget(self.preflight_status_label)

        self.start_guard_label = QLabel("")
        self.start_guard_label.setObjectName("systemWarning")
        self.start_guard_label.setVisible(False)

        setup_group_layout.addWidget(
            self.start_guard_label
        )

        main_layout.addWidget(setup_group)

        # =====================================================
        # SINGLE UR5
        # =====================================================

        self.ur5_group = QGroupBox(
            "Single UR5 configuration"
        )

        ur5_layout = QVBoxLayout(
            self.ur5_group
        )

        ur5_form = QFormLayout()

        self.ur5_type = QComboBox()
        self.ur5_type.addItems([
            "ur5",
            "ur5e",
        ])

        ur5_form.addRow(
            "Robot type:",
            self.ur5_type
        )

        ur5_layout.addLayout(ur5_form)

        # IP + TEST
        ur5_connection_layout = QHBoxLayout()

        self.ur5_ip = QLineEdit(
            "127.0.0.1"
        )

        self.ur5_test_button = QPushButton(
            "TEST"
        )

        self.ur5_test_button.clicked.connect(
            self.test_ur5_connection
        )

        ur5_connection_layout.addWidget(
            QLabel("Robot IP:")
        )

        ur5_connection_layout.addWidget(
            self.ur5_ip,
            1
        )

        ur5_connection_layout.addWidget(
            self.ur5_test_button
        )

        ur5_layout.addLayout(
            ur5_connection_layout
        )

        # Status
        ur5_status_layout = QHBoxLayout()

        ur5_status_layout.addWidget(
            QLabel("Connection:")
        )

        self.ur5_connection_status = QLabel(
            "NOT TESTED"
        )
        self.ur5_connection_status.setObjectName(
            "connectionUnknown"
        )

        ur5_status_layout.addWidget(
            self.ur5_connection_status
        )

        ur5_status_layout.addStretch()

        ur5_layout.addLayout(
            ur5_status_layout
        )

        # HOME
        self.ur5_home_button = QPushButton(
            "MOVE UR5 TO HOME"
        )

        self.ur5_home_button.setEnabled(
            False
        )

        self.ur5_home_button.clicked.connect(
            lambda: self.move_to_home("ur5")
        )

        self.ur5_home_button.setToolTip(
            "Move the UR5 to its saved HOME joint configuration."
        )

        ur5_layout.addWidget(
            self.ur5_home_button
        )

        main_layout.addWidget(
            self.ur5_group
        )

        # =====================================================
        # DUAL UR7e
        # =====================================================

        self.dual_group = QGroupBox(
            "Dual UR7e configuration"
        )

        dual_layout = QHBoxLayout(
            self.dual_group
        )
        dual_layout.setSpacing(10)

        # -----------------------------------------------------
        # Robot 1
        # -----------------------------------------------------

        robot1_box = QFrame()
        robot1_box.setObjectName("robotCard")
        robot1_layout = QVBoxLayout(robot1_box)
        robot1_layout.setContentsMargins(8, 5, 8, 5)
        robot1_layout.setSpacing(5)

        robot1_header = QHBoxLayout()

        robot1_title = QLabel("Robot 1")
        robot1_title.setObjectName(
            "robotSectionTitle"
        )
        robot1_header.addWidget(robot1_title)

        robot1_header.addWidget(QLabel("IP:"))

        self.robot1_ip = QLineEdit(
            "10.0.0.1"
        )
        robot1_header.addWidget(
            self.robot1_ip,
            1,
        )

        self.robot1_test_button = QPushButton(
            "TEST"
        )
        self.robot1_test_button.clicked.connect(
            self.test_robot1_connection
        )
        robot1_header.addWidget(
            self.robot1_test_button
        )

        robot1_header.addWidget(QLabel("Connection:"))

        self.robot1_connection_status = QLabel(
            "NOT TESTED"
        )
        self.robot1_connection_status.setObjectName(
            "connectionUnknown"
        )
        robot1_header.addWidget(
            self.robot1_connection_status
        )

        robot1_header.addWidget(QLabel("Robot:"))
        self.robot1_ready_status = QLabel("NOT STARTED")
        self.robot1_ready_status.setObjectName(
            "robotStateStopped"
        )
        robot1_header.addWidget(
            self.robot1_ready_status
        )

        robot1_layout.addLayout(robot1_header)

        robot1_actions = QHBoxLayout()

        self.robot1_home_button = QPushButton(
            "MOVE ROBOT"
        )
        self.robot1_home_button.setEnabled(False)
        self.robot1_pose_menu = QMenu(
            self.robot1_home_button
        )
        self.robot1_pose_menu.aboutToShow.connect(
            lambda: self.refresh_robot_pose_menu("robot1")
        )
        self.robot1_home_button.setMenu(
            self.robot1_pose_menu
        )
        self.robot1_home_button.setToolTip(
            "Choose a saved Robot 1 setup pose. "
            "Every real move is checked against the table and Robot 2 "
            "before any trajectory is sent."
        )
        robot1_actions.addWidget(
            self.robot1_home_button
        )

        robot1_actions.addWidget(
            QLabel("Gripper:")
        )
        robot1_actions.addWidget(
            QLabel("0.0")
        )

        self.robot1_gripper_slider = QSlider()
        self.robot1_gripper_slider.setOrientation(
            Qt.Horizontal
        )
        self.robot1_gripper_slider.setRange(0, 100)
        self.robot1_gripper_slider.setValue(100)
        self.robot1_gripper_slider.setSingleStep(1)
        self.robot1_gripper_slider.setPageStep(10)
        self.robot1_gripper_slider.setToolTip(
            "2FG7 command value: 0.0 = closed, 1.0 = open."
        )
        robot1_actions.addWidget(
            self.robot1_gripper_slider,
            1,
        )

        robot1_actions.addWidget(
            QLabel("1.0")
        )

        self.robot1_gripper_value = QLabel("1.00")
        self.robot1_gripper_value.setMinimumWidth(38)

        self.robot1_gripper_slider.valueChanged.connect(
            lambda value: self.robot1_gripper_value.setText(
                f"{value / 100.0:.2f}"
            )
        )

        robot1_actions.addWidget(
            self.robot1_gripper_value
        )

        self.robot1_gripper_move_button = QPushButton(
            "MOVE GRIP"
        )
        self.robot1_gripper_move_button.setEnabled(False)
        self.robot1_gripper_move_button.clicked.connect(
            lambda: self.command_gripper_position(
                "robot1",
                self.robot1_gripper_slider.value() / 100.0,
            )
        )
        self.robot1_gripper_move_button.setToolTip(
            "Send the selected 0.0-1.0 command to Robot 1 2FG7. "
            "Simulation: RViz. Real: physical gripper + RViz."
        )
        robot1_actions.addWidget(
            self.robot1_gripper_move_button
        )

        robot1_layout.addLayout(
            robot1_actions
        )

        dual_layout.addWidget(
            robot1_box,
            1,
        )

        # -----------------------------------------------------
        # Robot 2
        # -----------------------------------------------------

        robot2_box = QFrame()
        robot2_box.setObjectName("robotCard")
        robot2_layout = QVBoxLayout(robot2_box)
        robot2_layout.setContentsMargins(8, 5, 8, 5)
        robot2_layout.setSpacing(5)

        robot2_header = QHBoxLayout()

        robot2_title = QLabel("Robot 2")
        robot2_title.setObjectName(
            "robotSectionTitle"
        )
        robot2_header.addWidget(robot2_title)

        robot2_header.addWidget(QLabel("IP:"))

        self.robot2_ip = QLineEdit(
            "10.0.0.2"
        )
        robot2_header.addWidget(
            self.robot2_ip,
            1,
        )

        self.robot2_test_button = QPushButton(
            "TEST"
        )
        self.robot2_test_button.clicked.connect(
            self.test_robot2_connection
        )
        robot2_header.addWidget(
            self.robot2_test_button
        )

        robot2_header.addWidget(QLabel("Connection:"))

        self.robot2_connection_status = QLabel(
            "NOT TESTED"
        )
        self.robot2_connection_status.setObjectName(
            "connectionUnknown"
        )
        robot2_header.addWidget(
            self.robot2_connection_status
        )

        robot2_header.addWidget(QLabel("Robot:"))
        self.robot2_ready_status = QLabel("NOT STARTED")
        self.robot2_ready_status.setObjectName(
            "robotStateStopped"
        )
        robot2_header.addWidget(
            self.robot2_ready_status
        )

        robot2_layout.addLayout(robot2_header)

        robot2_actions = QHBoxLayout()

        self.robot2_home_button = QPushButton(
            "MOVE ROBOT"
        )
        self.robot2_home_button.setEnabled(False)
        self.robot2_pose_menu = QMenu(
            self.robot2_home_button
        )
        self.robot2_pose_menu.aboutToShow.connect(
            lambda: self.refresh_robot_pose_menu("robot2")
        )
        self.robot2_home_button.setMenu(
            self.robot2_pose_menu
        )
        self.robot2_home_button.setToolTip(
            "Choose a saved Robot 2 setup pose. "
            "Every real move is checked against the table and Robot 1 "
            "before any trajectory is sent."
        )
        robot2_actions.addWidget(
            self.robot2_home_button
        )

        robot2_actions.addWidget(
            QLabel("Gripper:")
        )
        robot2_actions.addWidget(
            QLabel("0.0")
        )

        self.robot2_gripper_slider = QSlider()
        self.robot2_gripper_slider.setOrientation(
            Qt.Horizontal
        )
        self.robot2_gripper_slider.setRange(0, 100)
        self.robot2_gripper_slider.setValue(100)
        self.robot2_gripper_slider.setSingleStep(1)
        self.robot2_gripper_slider.setPageStep(10)
        self.robot2_gripper_slider.setToolTip(
            "2FG7 command value: 0.0 = closed, 1.0 = open."
        )
        robot2_actions.addWidget(
            self.robot2_gripper_slider,
            1,
        )

        robot2_actions.addWidget(
            QLabel("1.0")
        )

        self.robot2_gripper_value = QLabel("1.00")
        self.robot2_gripper_value.setMinimumWidth(38)

        self.robot2_gripper_slider.valueChanged.connect(
            lambda value: self.robot2_gripper_value.setText(
                f"{value / 100.0:.2f}"
            )
        )

        robot2_actions.addWidget(
            self.robot2_gripper_value
        )

        self.robot2_gripper_move_button = QPushButton(
            "MOVE GRIP"
        )
        self.robot2_gripper_move_button.setEnabled(False)
        self.robot2_gripper_move_button.clicked.connect(
            lambda: self.command_gripper_position(
                "robot2",
                self.robot2_gripper_slider.value() / 100.0,
            )
        )
        self.robot2_gripper_move_button.setToolTip(
            "Send the selected 0.0-1.0 command to Robot 2 2FG7. "
            "Simulation: RViz. Real: physical gripper + RViz."
        )
        robot2_actions.addWidget(
            self.robot2_gripper_move_button
        )

        robot2_layout.addLayout(
            robot2_actions
        )

        dual_layout.addWidget(
            robot2_box,
            1,
        )

        main_layout.addWidget(
            self.dual_group
        )

        # =====================================================
        # EXPERIMENT
        # =====================================================

        self.experiment_group = QGroupBox("Experiment")
        experiment_layout = QVBoxLayout(self.experiment_group)
        experiment_layout.setSpacing(8)

        # -----------------------------------------------------
        # Experiment execution row
        # -----------------------------------------------------

        experiment_controls_layout = QHBoxLayout()
        experiment_controls_layout.setSpacing(8)

        experiment_controls_layout.addWidget(QLabel("Task:"))
        self.experiment_task_combo = QComboBox()
        self.experiment_task_combo.addItems([
            "Compound",
            "Polishing",
        ])
        experiment_controls_layout.addWidget(
            self.experiment_task_combo
        )

        experiment_controls_layout.addWidget(QLabel("Hand:"))
        self.experiment_hand_combo = QComboBox()
        self.experiment_hand_combo.addItems([
            "Right",
            "Left",
        ])
        experiment_controls_layout.addWidget(
            self.experiment_hand_combo
        )

        experiment_controls_layout.addWidget(QLabel("Control:"))
        self.experiment_control_combo = QComboBox()
        self.experiment_control_combo.addItems([
            "Open-loop",
            "Reactive",
            "Proactive",
        ])
        experiment_controls_layout.addWidget(
            self.experiment_control_combo
        )

        experiment_controls_layout.addWidget(QLabel("Motion:"))
        self.experiment_motion_combo = QComboBox()
        self.experiment_motion_combo.addItems([
            "Task Space",
            "Joint Space",
        ])
        experiment_controls_layout.addWidget(
            self.experiment_motion_combo
        )

        experiment_controls_layout.addWidget(QLabel("Speed:"))
        self.experiment_speed_spin = QDoubleSpinBox()
        self.experiment_speed_spin.setRange(0.05, 1.00)
        self.experiment_speed_spin.setSingleStep(0.05)
        self.experiment_speed_spin.setDecimals(2)
        self.experiment_speed_spin.setValue(0.20)
        self.experiment_speed_spin.setSuffix(" ×")
        self.experiment_speed_spin.setFixedWidth(90)
        self.experiment_speed_spin.setToolTip(
            "Experiment trajectory speed scale. "
            "1.00 = expert timing; 0.20 = five times slower."
        )
        experiment_controls_layout.addWidget(
            self.experiment_speed_spin
        )

        self.run_experiment_button = QPushButton(
            "RUN EXPERIMENT"
        )
        self.run_experiment_button.setObjectName("startButton")
        self.run_experiment_button.setEnabled(False)
        self.run_experiment_button.clicked.connect(
            self.run_selected_experiment
        )
        experiment_controls_layout.addWidget(
            self.run_experiment_button
        )

        self.experiment_status_label = QLabel("IDLE")
        self.experiment_status_label.setObjectName(
            "connectionUnknown"
        )
        self.experiment_status_label.setMinimumWidth(82)
        experiment_controls_layout.addWidget(
            self.experiment_status_label
        )

        experiment_controls_layout.addStretch()
        experiment_layout.addLayout(
            experiment_controls_layout
        )

        # -----------------------------------------------------
        # Analysis row
        # -----------------------------------------------------

        analysis_layout = QHBoxLayout()
        analysis_layout.setSpacing(8)

        analysis_layout.addWidget(QLabel("Analysis trial:"))

        self.analysis_trial_combo = QComboBox()
        self.analysis_trial_combo.setObjectName(
            "analysisTrialCombo"
        )
        self.analysis_trial_combo.setMinimumContentsLength(38)
        self.analysis_trial_combo.setToolTip(
            "Choose a completed experiment trial to analyze."
        )
        analysis_layout.addWidget(
            self.analysis_trial_combo,
            1,
        )

        self.refresh_analysis_button = QPushButton("REFRESH")
        self.refresh_analysis_button.setToolTip(
            "Reload the list of completed experiment trials."
        )
        self.refresh_analysis_button.clicked.connect(
            lambda: self.refresh_analysis_trials(
                preserve_selection=True
            )
        )
        analysis_layout.addWidget(
            self.refresh_analysis_button
        )

        self.analyze_trial_button = QPushButton("ANALYZE")
        self.analyze_trial_button.setEnabled(False)
        self.analyze_trial_button.clicked.connect(
            self.analyze_selected_trial
        )
        analysis_layout.addWidget(
            self.analyze_trial_button
        )

        self.analysis_status_label = QLabel("IDLE")
        self.analysis_status_label.setObjectName(
            "connectionUnknown"
        )
        self.analysis_status_label.setMinimumWidth(82)
        analysis_layout.addWidget(
            self.analysis_status_label
        )

        experiment_layout.addLayout(analysis_layout)

        for combo in (
            self.experiment_task_combo,
            self.experiment_hand_combo,
            self.experiment_control_combo,
            self.experiment_motion_combo,
        ):
            combo.currentIndexChanged.connect(
                self.update_experiment_controls
            )

        self.analysis_trial_combo.currentIndexChanged.connect(
            self.update_experiment_controls
        )

        main_layout.addWidget(self.experiment_group)

        # Populate the selector once at UI startup.
        # Completed trials are listed newest first.
        self.refresh_analysis_trials(
            preserve_selection=False
        )

        self.robot1_ip.textChanged.connect(
            lambda _text: self.set_connection_status(
                self.robot1_connection_status,
                "NOT TESTED",
            )
        )

        self.robot2_ip.textChanged.connect(
            lambda _text: self.set_connection_status(
                self.robot2_connection_status,
                "NOT TESTED",
            )
        )

        # =====================================================
        # LOWER WORKSPACE
        # Left: F/T + ROS output
        # Right: live NANSENSE skeleton
        # =====================================================

        self.lower_workspace_splitter = QSplitter(Qt.Horizontal)
        self.lower_workspace_splitter.setChildrenCollapsible(False)
        self.lower_workspace_splitter.setHandleWidth(7)

        sensor_column = QWidget()
        sensor_layout = QVBoxLayout(sensor_column)
        sensor_layout.setContentsMargins(0, 0, 0, 0)
        sensor_layout.setSpacing(7)

        # Keep expanding sensor/ROS sections from changing the
        # top-level window minimum height. If the left column needs
        # more vertical space, it scrolls inside its own half instead.
        sensor_scroll = QScrollArea()
        sensor_scroll.setObjectName("sensorScroll")
        sensor_scroll.setWidget(sensor_column)
        sensor_scroll.setWidgetResizable(True)
        sensor_scroll.setFrameShape(QFrame.NoFrame)
        sensor_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff
        )

        # QScrollArea has its own viewport, which otherwise uses the
        # desktop/default palette (white on this system).
        sensor_scroll.setStyleSheet(
            """
            QScrollArea#sensorScroll {
                background: #202124;
                border: 0px;
            }

            QScrollArea#sensorScroll > QWidget > QWidget {
                background: #202124;
            }
            """
        )
        sensor_scroll.setSizeAdjustPolicy(
            QAbstractScrollArea.AdjustIgnored
        )

        self.nansense_widget = NansenseLiveWidget(
            frame_callback=self.wrench_listener.update_nansense_frame,
            calibration_callback=(
                self.wrench_listener.update_nansense_calibration
            ),
            packet_callback=(
                self.wrench_listener.publish_nansense_raw_frame
            ),
        )
        self.nansense_widget.setMinimumWidth(650)

        sensor_scroll.setMinimumWidth(430)
        self.lower_workspace_splitter.addWidget(sensor_scroll)
        self.lower_workspace_splitter.addWidget(self.nansense_widget)
        self.lower_workspace_splitter.setStretchFactor(0, 42)
        self.lower_workspace_splitter.setStretchFactor(1, 58)
        saved_splitter = self.settings.value("lower_workspace_splitter")
        if saved_splitter is not None:
            self.lower_workspace_splitter.restoreState(saved_splitter)
        else:
            self.lower_workspace_splitter.setSizes([560, 780])

        main_layout.addWidget(
            self.lower_workspace_splitter,
            1,
        )

        # =====================================================
        # F/T RECORDING FOLDER + COLLAPSIBLE MONITORS
        # Visible only for Dual UR7e + Real Robot(s)
        # =====================================================

        self.wrench_panels = {}

        self.recording_folder_frame = QFrame()
        self.recording_folder_frame.setObjectName("recordingBar")
        recording_folder_layout = QHBoxLayout(
            self.recording_folder_frame
        )
        recording_folder_layout.setContentsMargins(8, 4, 8, 4)
        recording_folder_layout.setSpacing(6)

        recording_folder_layout.addWidget(
            QLabel("Recording folder:")
        )

        saved_recording_folder = self.settings.value(
            "recording_folder",
            self.default_recording_folder,
            type=str,
        )

        self.recording_folder_edit = QLineEdit(
            saved_recording_folder
        )
        self.recording_folder_edit.setToolTip(
            "CSV recordings from all three F/T sensors are saved here. "
            "This folder is kept outside the Git workspace by default."
        )
        self.recording_folder_edit.editingFinished.connect(
            self.save_recording_folder
        )
        recording_folder_layout.addWidget(
            self.recording_folder_edit,
            1,
        )

        self.recording_folder_button = QPushButton()
        self.recording_folder_button.setIcon(
            self.style().standardIcon(
                QStyle.SP_DirOpenIcon
            )
        )
        self.recording_folder_button.setFixedWidth(38)
        self.recording_folder_button.setToolTip(
            "Choose recording folder"
        )
        self.recording_folder_button.clicked.connect(
            self.browse_recording_folder
        )
        recording_folder_layout.addWidget(
            self.recording_folder_button
        )

        sensor_layout.addWidget(
            self.recording_folder_frame
        )

        # Internal UR7e sensors.
        self.internal_wrench_section = CollapsibleSection(
            "Force / Torque monitoring",
            expanded=False,
        )

        internal_wrench_layout = QGridLayout()
        internal_wrench_layout.setContentsMargins(0, 0, 0, 0)
        internal_wrench_layout.setHorizontalSpacing(8)
        internal_wrench_layout.setVerticalSpacing(5)

        internal_wrench_layout.addWidget(
            self.build_wrench_sensor_panel(
                "robot1",
                "Robot 1 — Internal F/T",
            ),
            0,
            0,
        )

        internal_wrench_layout.addWidget(
            self.build_wrench_sensor_panel(
                "robot2",
                "Robot 2 — Internal F/T",
            ),
            1,
            0,
        )

        internal_wrench_layout.setColumnStretch(0, 1)

        self.internal_wrench_section.body_layout.addLayout(
            internal_wrench_layout
        )

        sensor_layout.addWidget(
            self.internal_wrench_section
        )

        # External sensor remains a separate unit.
        self.external_wrench_section = CollapsibleSection(
            "External Robotiq FT300-S",
            expanded=False,
        )

        self.external_wrench_section.body_layout.addWidget(
            self.build_wrench_sensor_panel(
                "external",
                None,
                external=True,
            )
        )

        sensor_layout.addWidget(
            self.external_wrench_section
        )

        # -----------------------------------------------------
        # ROS output - secondary information, collapsed by default
        # -----------------------------------------------------

        self.ros_output_section = CollapsibleSection(
            "ROS 2 output",
            expanded=False,
        )

        self.log_output = QPlainTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setPlaceholderText(
            "ROS launch output will appear here..."
        )
        self.log_output.setMinimumHeight(90)
        self.log_output.setMaximumHeight(120)

        self.ros_output_section.body_layout.addWidget(
            self.log_output
        )

        sensor_layout.addWidget(
            self.ros_output_section
        )

        sensor_layout.addStretch(1)

    # =========================================================
    # Wrench monitoring UI
    # =========================================================

    def build_wrench_sensor_panel(
        self,
        key,
        title,
        external=False,
    ):

        if title:
            panel = QGroupBox(title)
        else:
            panel = QFrame()
            panel.setObjectName("sensorPanel")

        outer_layout = QVBoxLayout(panel)
        outer_layout.setContentsMargins(7, 5, 7, 5)
        outer_layout.setSpacing(4)

        header_layout = QHBoxLayout()
        header_layout.setSpacing(5)

        header_layout.addWidget(
            QLabel("Status:")
        )

        status_label = QLabel(
            "STOPPED"
            if external
            else "WAITING"
        )
        status_label.setObjectName(
            "connectionUnknown"
        )
        header_layout.addWidget(status_label)

        recording_label = QLabel("● REC")
        recording_label.setObjectName("recordingActive")
        recording_label.setVisible(False)
        header_layout.addWidget(recording_label)

        header_layout.addStretch()

        internal_zero_button = None

        if external:
            self.external_ft_start_button = QPushButton(
                "START"
            )
            self.external_ft_stop_button = QPushButton(
                "STOP"
            )
            self.external_ft_zero_button = QPushButton(
                "ZERO"
            )

            self.external_ft_start_button.setToolTip(
                "Start the external Robotiq FT300-S stream."
            )
            self.external_ft_stop_button.setToolTip(
                "Stop the external Robotiq FT300-S stream started by this UI."
            )
            self.external_ft_zero_button.setToolTip(
                "Software-zero the external Robotiq FT300-S."
            )

            self.external_ft_stop_button.setEnabled(False)
            self.external_ft_zero_button.setEnabled(False)

            self.external_ft_start_button.clicked.connect(
                self.start_external_ft
            )
            self.external_ft_stop_button.clicked.connect(
                self.stop_external_ft
            )
            self.external_ft_zero_button.clicked.connect(
                self.zero_external_ft
            )

            header_layout.addWidget(
                self.external_ft_start_button
            )
            header_layout.addWidget(
                self.external_ft_stop_button
            )
            header_layout.addWidget(
                self.external_ft_zero_button
            )

        else:
            internal_zero_button = QPushButton(
                "ZERO"
            )
            internal_zero_button.setEnabled(False)
            internal_zero_button.setToolTip(
                f"Zero the {key} internal UR F/T sensor."
            )
            internal_zero_button.clicked.connect(
                lambda checked=False, sensor_key=key:
                    self.zero_internal_ft(sensor_key)
            )
            header_layout.addWidget(
                internal_zero_button
            )

        view_button = QPushButton(
            "HIDE TORQUES"
        )
        view_button.setCheckable(True)
        view_button.setChecked(True)
        view_button.setToolTip(
            "Show or hide torque channels. Recording follows this selection."
        )
        header_layout.addWidget(view_button)

        header_layout.addWidget(QLabel("REC RATE:"))

        rate_combo = QComboBox()
        rate_combo.addItem("100 Hz", 100)
        rate_combo.addItem("50 Hz", 50)
        rate_combo.addItem("30 Hz", 30)
        rate_combo.addItem("20 Hz", 20)
        rate_combo.addItem("10 Hz", 10)
        rate_combo.setCurrentIndex(0)
        rate_combo.setToolTip(
            "CSV recording rate only. The wrench sensor and ROS topic "
            "continue running at the full ~100 Hz source rate."
        )
        header_layout.addWidget(rate_combo)

        start_record_button = QPushButton(
            "START REC"
        )
        start_record_button.setObjectName(
            "recordStartButton"
        )
        start_record_button.setEnabled(False)
        start_record_button.setToolTip(
            "Start CSV recording at the selected rate."
        )
        start_record_button.clicked.connect(
            lambda checked=False, sensor_key=key:
                self.start_wrench_recording(sensor_key)
        )
        header_layout.addWidget(start_record_button)

        stop_record_button = QPushButton(
            "STOP REC"
        )
        stop_record_button.setObjectName(
            "recordStopButton"
        )
        stop_record_button.setEnabled(False)
        stop_record_button.setToolTip(
            "Stop and close this sensor's CSV recording."
        )
        stop_record_button.clicked.connect(
            lambda checked=False, sensor_key=key:
                self.stop_wrench_recording(sensor_key)
        )
        header_layout.addWidget(stop_record_button)

        outer_layout.addLayout(header_layout)

        body_layout = QHBoxLayout()
        body_layout.setSpacing(6)
        body_layout.setContentsMargins(0, 0, 0, 0)

        # ---------------- Forces ----------------

        force_widget = QWidget()
        force_layout = QGridLayout(force_widget)
        force_layout.setContentsMargins(0, 0, 0, 0)
        force_layout.setHorizontalSpacing(5)
        force_layout.setVerticalSpacing(2)

        force_title = QLabel("FORCES [N]")
        force_title.setObjectName("wrenchColumnTitle")
        force_layout.addWidget(
            force_title,
            0,
            0,
            1,
            3,
        )

        # ---------------- Torques ----------------

        torque_widget = QWidget()
        torque_layout = QGridLayout(torque_widget)
        torque_layout.setContentsMargins(0, 0, 0, 0)
        torque_layout.setHorizontalSpacing(5)
        torque_layout.setVerticalSpacing(2)

        torque_title = QLabel("TORQUES [Nm]")
        torque_title.setObjectName("wrenchColumnTitle")
        torque_layout.addWidget(
            torque_title,
            0,
            0,
            1,
            3,
        )

        labels = {}
        bars = {}

        force_components = (
            ("fx", "Fx"),
            ("fy", "Fy"),
            ("fz", "Fz"),
        )
        torque_components = (
            ("mx", "Mx"),
            ("my", "My"),
            ("mz", "Mz"),
        )

        for row, (component, display) in enumerate(
            force_components,
            start=1,
        ):
            name_label = QLabel(display)
            value_label = QLabel("--")
            value_label.setMinimumWidth(64)
            bar = CenteredBar(FORCE_BAR_LIMIT)

            force_layout.addWidget(name_label, row, 0)
            force_layout.addWidget(value_label, row, 1)
            force_layout.addWidget(bar, row, 2)

            labels[component] = value_label
            bars[component] = bar

        for row, (component, display) in enumerate(
            torque_components,
            start=1,
        ):
            name_label = QLabel(display)
            value_label = QLabel("--")
            value_label.setMinimumWidth(72)
            bar = CenteredBar(TORQUE_BAR_LIMIT)

            torque_layout.addWidget(name_label, row, 0)
            torque_layout.addWidget(value_label, row, 1)
            torque_layout.addWidget(bar, row, 2)

            labels[component] = value_label
            bars[component] = bar

        force_layout.setColumnStretch(2, 1)
        torque_layout.setColumnStretch(2, 1)

        body_layout.addWidget(force_widget, 1)
        body_layout.addWidget(torque_widget, 1)
        outer_layout.addLayout(body_layout)

        self.wrench_panels[key] = {
            "panel": panel,
            "status": status_label,
            "recording_label": recording_label,
            "view_button": view_button,
            "rate_combo": rate_combo,
            "start_record_button": start_record_button,
            "stop_record_button": stop_record_button,
            "force_widget": force_widget,
            "torque_widget": torque_widget,
            "labels": labels,
            "bars": bars,
        }

        if internal_zero_button is not None:
            self.wrench_panels[key]["zero_button"] = (
                internal_zero_button
            )

        view_button.toggled.connect(
            lambda checked, sensor_key=key:
                self.set_torque_visibility(
                    sensor_key,
                    checked,
                )
        )

        return panel

    def set_torque_visibility(
        self,
        key,
        show_torques,
    ):

        panel = self.wrench_panels[key]

        panel["torque_widget"].setVisible(
            show_torques
        )

        panel["view_button"].setText(
            "HIDE TORQUES"
            if show_torques
            else "SHOW TORQUES"
        )

    def set_wrench_status(
        self,
        key,
        text,
        state,
    ):

        label = self.wrench_panels[key][
            "status"
        ]

        label.setText(text)

        object_names = {
            "live": "connectionReachable",
            "waiting": "connectionTesting",
            "stopped": "connectionUnknown",
            "error": "connectionOffline",
        }

        label.setObjectName(
            object_names.get(
                state,
                "connectionUnknown",
            )
        )

        label.style().unpolish(
            label
        )
        label.style().polish(
            label
        )

    def set_wrench_values(
        self,
        key,
        values,
    ):

        components = (
            "fx",
            "fy",
            "fz",
            "mx",
            "my",
            "mz",
        )

        panel = self.wrench_panels[key]

        for component, value in zip(
            components,
            values,
        ):

            if component.startswith("f"):
                panel["labels"][component].setText(
                    f"{value:+.2f} N"
                )
            else:
                panel["labels"][component].setText(
                    f"{value:+.3f} Nm"
                )

            panel["bars"][component].set_value(
                value
            )

    def clear_wrench_values(
        self,
        key,
    ):

        panel = self.wrench_panels[key]

        for label in panel[
            "labels"
        ].values():
            label.setText("--")

        for bar in panel[
            "bars"
        ].values():
            bar.set_value(0.0)

    def refresh_wrench_display(self):

        if not hasattr(
            self,
            "wrench_listener",
        ):
            return

        snapshot = (
            self.wrench_listener.snapshot()
        )

        now = time.monotonic()

        dual_real = (
            self.setup_combo.currentText()
            == "Dual UR7e"
            and self.mode_combo.currentText()
            == "Real Robot(s)"
        )

        system_running = (
            self.status_label.text()
            == "RUNNING"
        )

        # Internal Robot 1 / Robot 2 wrench.
        for key in (
            "robot1",
            "robot2",
        ):

            data = snapshot.get(key)
            fresh = (
                data is not None
                and now - data[0]
                <= WRENCH_STALE_SEC
            )

            if (
                dual_real
                and system_running
                and fresh
            ):
                self.set_wrench_values(
                    key,
                    data[1],
                )
                self.set_wrench_status(
                    key,
                    "LIVE",
                    "live",
                )

            elif (
                dual_real
                and system_running
            ):
                self.set_wrench_status(
                    key,
                    "WAITING FOR DATA",
                    "waiting",
                )

            else:
                self.set_wrench_status(
                    key,
                    "STOPPED",
                    "stopped",
                )
                self.clear_wrench_values(
                    key
                )

        # External FT300 is independent from the UR launch.
        external_data = snapshot.get(
            "external"
        )

        self.external_ft_live = (
            external_data is not None
            and now - external_data[0]
            <= WRENCH_STALE_SEC
        )

        if (
            dual_real
            and self.external_ft_live
        ):
            self.set_wrench_values(
                "external",
                external_data[1],
            )
            self.set_wrench_status(
                "external",
                "LIVE",
                "live",
            )

        elif (
            self.ft_process.state()
            != QProcess.NotRunning
        ):
            self.set_wrench_status(
                "external",
                "WAITING FOR DATA",
                "waiting",
            )

        else:
            self.set_wrench_status(
                "external",
                "STOPPED",
                "stopped",
            )

            if not self.external_ft_live:
                self.clear_wrench_values(
                    "external"
                )

        self.update_internal_ft_controls()
        self.update_external_ft_controls()
        self.update_recording_controls()

    # =========================================================
    # Wrench CSV recording
    # =========================================================

    def save_recording_folder(self):
        folder = os.path.expanduser(
            self.recording_folder_edit.text().strip()
        )

        if not folder:
            folder = self.default_recording_folder
            self.recording_folder_edit.setText(folder)

        self.settings.setValue(
            "recording_folder",
            folder,
        )
        self.settings.sync()

    def browse_recording_folder(self):
        current = os.path.expanduser(
            self.recording_folder_edit.text().strip()
        )

        if not os.path.isdir(current):
            current = self.default_recording_folder

        if not os.path.isdir(current):
            current = os.path.expanduser("~")

        folder = QFileDialog.getExistingDirectory(
            self,
            "Select F/T recording folder",
            current,
            QFileDialog.ShowDirsOnly
            | QFileDialog.DontResolveSymlinks,
        )

        if folder:
            self.recording_folder_edit.setText(folder)
            self.save_recording_folder()

    def start_wrench_recording(self, key):
        panel = self.wrench_panels.get(key)
        if panel is None:
            return

        if self.wrench_listener.is_recording(key):
            return

        if panel["status"].text() != "LIVE":
            QMessageBox.warning(
                self,
                "Sensor not live",
                "Start the sensor and wait until its status is LIVE before recording.",
            )
            return

        folder = os.path.expanduser(
            self.recording_folder_edit.text().strip()
        )

        # Persist manual path edits as soon as a recording starts.
        self.save_recording_folder()

        if not folder:
            QMessageBox.warning(
                self,
                "Recording folder",
                "Select a recording folder first.",
            )
            return

        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as exc:
            QMessageBox.warning(
                self,
                "Recording folder",
                f"Could not create/access recording folder:\n{exc}",
            )
            return

        names = {
            "robot1": "robot1_internal_ft",
            "robot2": "robot2_internal_ft",
            "external": "external_ft300",
        }

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        record_rate_hz = int(
            panel["rate_combo"].currentData()
        )

        filename = (
            f"{names[key]}_{timestamp}_{record_rate_hz}Hz.csv"
        )
        path = os.path.join(folder, filename)

        include_torque = bool(
            panel["view_button"].isChecked()
        )

        try:
            self.wrench_listener.start_recording(
                key,
                path,
                include_torque,
                record_rate_hz,
            )
        except Exception as exc:
            QMessageBox.warning(
                self,
                "Recording error",
                f"Could not start CSV recording:\n{exc}",
            )
            return

        panel["recording_label"].setVisible(True)
        panel["view_button"].setEnabled(False)
        panel["rate_combo"].setEnabled(False)

        channels = (
            "Fx, Fy, Fz, Mx, My, Mz"
            if include_torque
            else "Fx, Fy, Fz"
        )

        self.log_output.appendPlainText(
            f"\n[REC started: {path}]\n"
            f"[Rate: {record_rate_hz} Hz | Channels: {channels}]"
        )

        self.update_recording_controls()

    def stop_wrench_recording(self, key, silent=False):
        if not self.wrench_listener.is_recording(key):
            return

        path = self.wrench_listener.stop_recording(key)

        panel = self.wrench_panels.get(key)
        if panel is not None:
            panel["recording_label"].setVisible(False)
            panel["view_button"].setEnabled(True)
            panel["rate_combo"].setEnabled(True)

        if path and not silent:
            self.log_output.appendPlainText(
                f"\n[REC saved: {path}]"
            )

        self.update_recording_controls()

    def stop_all_wrench_recordings(self, silent=False):
        for key in ("robot1", "robot2", "external"):
            if self.wrench_listener.is_recording(key):
                self.stop_wrench_recording(
                    key,
                    silent=silent,
                )

    def update_recording_controls(self):
        if not hasattr(self, "wrench_panels"):
            return

        dual_real = (
            self.setup_combo.currentText() == "Dual UR7e"
            and self.mode_combo.currentText() == "Real Robot(s)"
        )

        for key, panel in self.wrench_panels.items():
            recording = self.wrench_listener.is_recording(key)
            live = panel["status"].text() == "LIVE"

            panel["start_record_button"].setEnabled(
                dual_real
                and live
                and not recording
            )
            panel["stop_record_button"].setEnabled(
                recording
            )
            panel["recording_label"].setVisible(
                recording
            )
            panel["view_button"].setEnabled(
                not recording
            )
            panel["rate_combo"].setEnabled(
                dual_real and not recording
            )

    # =========================================================
    # Internal UR7e F/T zero
    # =========================================================

    def update_internal_ft_controls(self):

        if not hasattr(
            self,
            "internal_ft_zero_processes",
        ):
            return

        dual_real = (
            self.setup_combo.currentText()
            == "Dual UR7e"
            and self.mode_combo.currentText()
            == "Real Robot(s)"
        )

        system_running = (
            self.status_label.text()
            == "RUNNING"
        )

        for key in (
            "robot1",
            "robot2",
        ):

            panel = self.wrench_panels.get(
                key
            )

            if (
                panel is None
                or "zero_button" not in panel
            ):
                continue

            process = (
                self.internal_ft_zero_processes[
                    key
                ]
            )

            busy = (
                process.state()
                != QProcess.NotRunning
            )

            live = (
                panel["status"].text()
                == "LIVE"
            )

            panel["zero_button"].setText(
                "ZEROING..."
                if busy
                else "ZERO"
            )

            panel["zero_button"].setEnabled(
                dual_real
                and system_running
                and live
                and not busy
            )

    def zero_internal_ft(
        self,
        key,
    ):

        if key not in (
            "robot1",
            "robot2",
        ):
            return

        if (
            self.setup_combo.currentText()
            != "Dual UR7e"
            or self.mode_combo.currentText()
            != "Real Robot(s)"
            or self.status_label.text()
            != "RUNNING"
        ):
            return

        panel = self.wrench_panels.get(
            key
        )

        if (
            panel is None
            or panel["status"].text()
            != "LIVE"
        ):
            return

        process = (
            self.internal_ft_zero_processes[
                key
            ]
        )

        if (
            process.state()
            != QProcess.NotRunning
        ):
            return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        service = (
            f"/{key}/io_and_status_controller/"
            "zero_ftsensor"
        )

        command = (
            "timeout 5s ros2 service call "
            f"{service} "
            'std_srvs/srv/Trigger "{}"'
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && exec {command}"
        )

        self.log_output.appendPlainText(
            f"\n$ {command}\n"
        )

        process.start(
            "/bin/bash",
            [
                "-lc",
                full_command,
            ],
        )

        self.update_internal_ft_controls()

    def read_internal_ft_zero_output(
        self,
        key,
    ):

        process = (
            self.internal_ft_zero_processes[
                key
            ]
        )

        text = bytes(
            process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        if text:
            self.log_output.appendPlainText(
                text.rstrip()
            )

            scrollbar = (
                self.log_output.verticalScrollBar()
            )
            scrollbar.setValue(
                scrollbar.maximum()
            )

    def internal_ft_zero_finished(
        self,
        key,
        exit_code,
        exit_status,
    ):

        display_name = (
            "Robot 1"
            if key == "robot1"
            else "Robot 2"
        )

        if exit_code == 0:
            self.log_output.appendPlainText(
                f"\n[{display_name} internal F/T zero completed]"
            )
        else:
            self.log_output.appendPlainText(
                f"\n[{display_name} internal F/T zero failed "
                f"- exit code {exit_code}]"
            )

        self.update_internal_ft_controls()

    # =========================================================
    # External Robotiq FT300-S
    # =========================================================

    def update_external_ft_controls(self):

        if not hasattr(
            self,
            "external_ft_start_button",
        ):
            return

        dual_real = (
            self.setup_combo.currentText()
            == "Dual UR7e"
            and self.mode_combo.currentText()
            == "Real Robot(s)"
        )

        process_running = (
            self.ft_process.state()
            != QProcess.NotRunning
        )

        zero_idle = (
            self.ft_zero_process.state()
            == QProcess.NotRunning
        )

        # If /external_ft is already live from a manually started
        # node, do not start a second process that would compete for
        # /dev/ttyUSB0.
        self.external_ft_start_button.setEnabled(
            dual_real
            and not process_running
            and not self.external_ft_live
        )

        # STOP only controls the F/T process started by this UI.
        self.external_ft_stop_button.setEnabled(
            dual_real
            and process_running
        )

        self.external_ft_zero_button.setEnabled(
            dual_real
            and self.external_ft_live
            and zero_idle
        )

    def start_external_ft(self):

        if (
            self.setup_combo.currentText()
            != "Dual UR7e"
            or self.mode_combo.currentText()
            != "Real Robot(s)"
        ):
            return

        if (
            self.ft_process.state()
            != QProcess.NotRunning
        ):
            return

        if self.external_ft_live:
            return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        command = (
            "ros2 run ur7e_tools ft_sensor"
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && exec setsid {command}"
        )

        self.external_ft_stopping = False

        self.log_output.appendPlainText(
            f"\n$ {command}\n"
        )

        self.set_wrench_status(
            "external",
            "STARTING",
            "waiting",
        )

        self.ft_process.start(
            "/bin/bash",
            [
                "-lc",
                full_command,
            ],
        )

        self.update_external_ft_controls()

    def stop_external_ft(self):

        if (
            self.ft_process.state()
            == QProcess.NotRunning
        ):
            return

        self.external_ft_stopping = True

        pid = int(
            self.ft_process.processId()
        )

        if pid > 0:
            try:
                os.killpg(
                    pid,
                    signal.SIGINT,
                )
            except ProcessLookupError:
                pass

        QTimer.singleShot(
            1500,
            self.force_stop_external_ft_if_needed,
        )

    def force_stop_external_ft_if_needed(self):

        if (
            self.ft_process.state()
            == QProcess.NotRunning
        ):
            return

        pid = int(
            self.ft_process.processId()
        )

        if pid > 0:
            try:
                os.killpg(
                    pid,
                    signal.SIGTERM,
                )
            except ProcessLookupError:
                pass

    def external_ft_process_started(self):

        self.set_wrench_status(
            "external",
            "STARTING",
            "waiting",
        )

        self.update_external_ft_controls()

    def external_ft_process_finished(
        self,
        exit_code,
        exit_status,
    ):

        was_stopping = (
            self.external_ft_stopping
        )

        self.external_ft_stopping = False

        if (
            was_stopping
            or exit_code == 0
        ):
            self.set_wrench_status(
                "external",
                "STOPPED",
                "stopped",
            )
        else:
            self.set_wrench_status(
                "external",
                f"ERROR ({exit_code})",
                "error",
            )

        self.log_output.appendPlainText(
            f"\n[External F/T process stopped - exit code {exit_code}]"
        )

        self.update_external_ft_controls()

    def read_external_ft_output(self):

        text = bytes(
            self.ft_process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        if text:
            self.log_output.appendPlainText(
                text.rstrip()
            )

            scrollbar = (
                self.log_output.verticalScrollBar()
            )
            scrollbar.setValue(
                scrollbar.maximum()
            )

    def zero_external_ft(self):

        if not self.external_ft_live:
            return

        if (
            self.ft_zero_process.state()
            != QProcess.NotRunning
        ):
            return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        command = (
            "timeout 5s ros2 service call "
            "/external_ft/zero "
            'std_srvs/srv/Trigger "{}"'
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && exec {command}"
        )

        self.log_output.appendPlainText(
            f"\n$ {command}\n"
        )

        self.ft_zero_process.start(
            "/bin/bash",
            [
                "-lc",
                full_command,
            ],
        )

        self.update_external_ft_controls()

    def read_external_ft_zero_output(self):

        text = bytes(
            self.ft_zero_process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        if text:
            self.log_output.appendPlainText(
                text.rstrip()
            )

    def external_ft_zero_finished(
        self,
        exit_code,
        exit_status,
    ):

        if exit_code == 0:
            self.log_output.appendPlainText(
                "\n[External F/T zero completed]"
            )
        else:
            self.log_output.appendPlainText(
                f"\n[External F/T zero failed - exit code {exit_code}]"
            )

        self.update_external_ft_controls()

    # =========================================================
    # Supervisor preflight
    # =========================================================

    def set_preflight_status(self, text, state, tooltip=None):
        if not hasattr(self, "preflight_status_label"):
            return

        self.preflight_status_label.setText(text)

        object_names = {
            "clean": "connectionReachable",
            "cleaned": "connectionReachable",
            "checking": "connectionTesting",
            "warning": "connectionTesting",
            "blocked": "connectionOffline",
            "unknown": "connectionUnknown",
        }

        self.preflight_status_label.setObjectName(
            object_names.get(state, "connectionUnknown")
        )

        if tooltip is not None:
            self.preflight_status_label.setToolTip(tooltip)

        self.preflight_status_label.style().unpolish(
            self.preflight_status_label
        )
        self.preflight_status_label.style().polish(
            self.preflight_status_label
        )

    def apply_startup_preflight_report(self):
        report = self.startup_preflight_report
        state = report.get("state", "UNKNOWN")
        message = report.get("message", "")

        if state == "CLEAN":
            self.set_preflight_status(
                "CLEAN",
                "clean",
                message,
            )
        elif state == "CLEANED":
            self.set_preflight_status(
                "CLEANED",
                "cleaned",
                message,
            )
        elif state == "BLOCKED":
            blockers = report.get("blocked_processes", [])
            details = "\n".join(
                f"PID {pid}: {cmdline}"
                for pid, cmdline in blockers[:8]
            )
            tooltip = message
            if details:
                tooltip += "\n\n" + details

            self.set_preflight_status(
                "BLOCKED",
                "blocked",
                tooltip,
            )
        elif state == "WARNING":
            self.set_preflight_status(
                "WARNING",
                "warning",
                message,
            )
        else:
            self.set_preflight_status(
                "UNKNOWN",
                "unknown",
                message,
            )

    def preflight_before_start(self):
        """Fail closed if another workcell ROS stack is already running."""

        self.set_preflight_status(
            "CHECKING...",
            "checking",
            "Checking for existing workcell ROS processes.",
        )
        QApplication.processEvents()

        startup_state = self.startup_preflight_report.get(
            "state",
            "UNKNOWN",
        )

        # If startup cleanup could not establish a clean FastDDS state, fail
        # closed. The safe recovery is to stop the old ROS process(es), close
        # this UI, and reopen it so cleanup happens before rclpy.init().
        if startup_state not in ("CLEAN", "CLEANED"):
            self.set_preflight_status(
                "RESTART REQUIRED",
                "blocked",
                "Startup FastDDS cleanup was not completed safely. "
                "Stop old ROS/workcell processes, close this UI, "
                "and open it again.",
            )
            self.start_guard_label.setText(
                "Preflight requires a clean UI restart before START SYSTEM."
            )
            self.start_guard_label.setVisible(True)
            return False

        blockers = find_running_workcell_processes()

        if blockers:
            details = "\n".join(
                f"PID {pid}: {cmdline}"
                for pid, cmdline in blockers[:8]
            )

            self.set_preflight_status(
                "BLOCKED",
                "blocked",
                "Existing workcell ROS processes detected.\n\n"
                + details,
            )

            self.start_guard_label.setText(
                "Preflight blocked: another workcell ROS process is "
                "already running. Stop it before START SYSTEM."
            )
            self.start_guard_label.setVisible(True)
            return False

        # A ros2 CLI daemon is safe to stop and does not control the robots.
        # Keeping it out of the launch transition also reduces stale FastDDS
        # shared-memory participants between repeated sessions.
        stop_ros2_daemon_quietly()

        self.set_preflight_status(
            "CLEAN",
            "clean",
            "No existing workcell ROS processes detected.",
        )
        return True

    # =========================================================
    # Setup visibility
    # =========================================================

    def update_setup_view(self):

        single = (
            self.setup_combo.currentText()
            == "Single UR5"
        )

        self.ur5_group.setVisible(
            single
        )

        self.dual_group.setVisible(
            not single
        )

        if hasattr(self, "experiment_group"):
            self.experiment_group.setVisible(not single)
            self.update_experiment_controls()

        if hasattr(self, "robots_ready_label"):
            self.robots_ready_label.setVisible(not single)

        if hasattr(self, "health_status_label"):
            self.health_status_label.setVisible(not single)

        if hasattr(self, "start_guard_label"):
            self.start_guard_label.clear()
            self.start_guard_label.setVisible(False)

        dual_real = (
            not single
            and self.mode_combo.currentText()
            == "Real Robot(s)"
        )

        if hasattr(self, "recording_folder_frame"):
            self.recording_folder_frame.setVisible(dual_real)

        if hasattr(self, "internal_wrench_section"):
            self.internal_wrench_section.setVisible(dual_real)

        if hasattr(self, "external_wrench_section"):
            self.external_wrench_section.setVisible(dual_real)

        if not dual_real and hasattr(self, "wrench_listener"):
            self.stop_all_wrench_recordings(silent=True)

        if hasattr(self, "home_process"):
            self.update_home_buttons()

        if hasattr(
            self,
            "internal_ft_zero_processes",
        ):
            self.update_internal_ft_controls()

        if hasattr(
            self,
            "external_ft_start_button",
        ):
            self.update_external_ft_controls()

        if hasattr(self, "wrench_panels"):
            self.update_recording_controls()

    # =========================================================
    # IP validation
    # =========================================================

    def valid_ip(self, address):

        try:
            ipaddress.ip_address(
                address
            )
            return True

        except ValueError:
            return False

    # =========================================================
    # Connection status helper
    # =========================================================

    def set_connection_status(
        self,
        label,
        status
    ):

        label.setText(
            status
        )

        if status == "REACHABLE":

            label.setObjectName(
                "connectionReachable"
            )

        elif status == "OFFLINE":

            label.setObjectName(
                "connectionOffline"
            )

        elif status == "TESTING...":

            label.setObjectName(
                "connectionTesting"
            )

        else:

            label.setObjectName(
                "connectionUnknown"
            )

        label.style().unpolish(
            label
        )

        label.style().polish(
            label
        )

    # =========================================================
    # Ping helper
    # =========================================================

    def start_ping(
        self,
        ip,
        process,
        status_label
    ):

        if not self.valid_ip(ip):

            QMessageBox.warning(
                self,
                "Invalid IP",
                f"Invalid IP address:\n{ip}",
            )

            return

        if (
            process.state()
            != QProcess.NotRunning
        ):
            return

        self.set_connection_status(
            status_label,
            "TESTING..."
        )

        process.start(
            "ping",
            [
                "-c",
                "1",
                "-W",
                "1",
                ip,
            ],
        )

    # =========================================================
    # UR5 ping
    # =========================================================

    def test_ur5_connection(self):

        self.start_ping(
            self.ur5_ip.text().strip(),
            self.ur5_ping_process,
            self.ur5_connection_status,
        )

    def ur5_ping_finished(
        self,
        exit_code,
        exit_status
    ):

        if exit_code == 0:

            self.set_connection_status(
                self.ur5_connection_status,
                "REACHABLE"
            )

        else:

            self.set_connection_status(
                self.ur5_connection_status,
                "OFFLINE"
            )

    # =========================================================
    # Robot 1 ping
    # =========================================================

    def test_robot1_connection(self):

        self.robot1_tested_ip = (
            self.robot1_ip.text().strip()
        )

        self.start_ping(
            self.robot1_tested_ip,
            self.robot1_ping_process,
            self.robot1_connection_status,
        )

    def robot1_ping_finished(
        self,
        exit_code,
        exit_status
    ):

        if (
            self.robot1_ip.text().strip()
            != self.robot1_tested_ip
        ):
            self.set_connection_status(
                self.robot1_connection_status,
                "NOT TESTED"
            )
            return

        if exit_code == 0:

            self.set_connection_status(
                self.robot1_connection_status,
                "REACHABLE"
            )
            self.clear_connection_test_warning_if_ready()

        else:

            self.set_connection_status(
                self.robot1_connection_status,
                "OFFLINE"
            )

    # =========================================================
    # Robot 2 ping
    # =========================================================

    def test_robot2_connection(self):

        self.robot2_tested_ip = (
            self.robot2_ip.text().strip()
        )

        self.start_ping(
            self.robot2_tested_ip,
            self.robot2_ping_process,
            self.robot2_connection_status,
        )

    def robot2_ping_finished(
        self,
        exit_code,
        exit_status
    ):

        if (
            self.robot2_ip.text().strip()
            != self.robot2_tested_ip
        ):
            self.set_connection_status(
                self.robot2_connection_status,
                "NOT TESTED"
            )
            return

        if exit_code == 0:

            self.set_connection_status(
                self.robot2_connection_status,
                "REACHABLE"
            )
            self.clear_connection_test_warning_if_ready()

        else:

            self.set_connection_status(
                self.robot2_connection_status,
                "OFFLINE"
            )

    def clear_connection_test_warning_if_ready(self):

        both_reachable = (
            self.robot1_connection_status.text() == "REACHABLE"
            and self.robot2_connection_status.text() == "REACHABLE"
        )

        if (
            both_reachable
            and self.start_guard_label.text().startswith(
                "Press TEST for both robot connections"
            )
        ):
            self.start_guard_label.clear()
            self.start_guard_label.setVisible(False)

    # =========================================================
    # Robot program / reverse-interface readiness
    # =========================================================

    def set_robot_ready_status(self, robot, status, tooltip=None):

        label = (
            self.robot1_ready_status
            if robot == "robot1"
            else self.robot2_ready_status
        )

        label.setText(status)

        object_names = {
            "READY": "robotStateReady",
            "WAITING FOR PLAY": "robotStateWaiting",
            "CONNECTING...": "robotStateConnecting",
            "HEALTH CHECK...": "robotStateConnecting",
            "HEALTH ERROR": "robotStateDisconnected",
            "DISCONNECTED": "robotStateDisconnected",
            "NOT STARTED": "robotStateStopped",
        }

        label.setObjectName(
            object_names.get(status, "robotStateStopped")
        )

        if tooltip is not None:
            label.setToolTip(tooltip)

        label.style().unpolish(label)
        label.style().polish(label)

    def set_health_status(self, text, state, tooltip=None):

        if not hasattr(self, "health_status_label"):
            return

        self.health_status_label.setText(text)

        object_names = {
            "ok": "connectionReachable",
            "checking": "connectionTesting",
            "fault": "connectionOffline",
            "stopped": "connectionUnknown",
        }

        self.health_status_label.setObjectName(
            object_names.get(state, "connectionUnknown")
        )

        if tooltip is not None:
            self.health_status_label.setToolTip(tooltip)

        self.health_status_label.style().unpolish(
            self.health_status_label
        )
        self.health_status_label.style().polish(
            self.health_status_label
        )

    def update_robot_ready_summary(self):

        if not hasattr(self, "robots_ready_label"):
            return

        ready_count = sum(
            1 for ready in self.robot_ready.values()
            if ready
        )

        if ready_count == 2:
            summary = "WORKCELL READY"
            object_name = "robotSummaryReady"
        elif ready_count == 1:
            summary = "1/2 READY"
            object_name = "robotSummaryPartial"
        else:
            summary = "NOT READY"
            object_name = "robotSummaryUnknown"

        self.robots_ready_label.setText(summary)
        self.robots_ready_label.setObjectName(object_name)
        self.robots_ready_label.style().unpolish(
            self.robots_ready_label
        )
        self.robots_ready_label.style().polish(
            self.robots_ready_label
        )

    def reset_robot_readiness(self, status="NOT STARTED"):

        for robot in ("robot1", "robot2"):
            self.robot_ready[robot] = False
            self.robot_reverse_ready_seen[robot] = False
            self.robot_core_health[robot] = "checking"

            if hasattr(self, f"{robot}_ready_status"):
                self.set_robot_ready_status(
                    robot,
                    status,
                    "Workcell health has not been established for this session.",
                )

        self.update_robot_ready_summary()

        if hasattr(self, "home_process"):
            self.update_home_buttons()

        if hasattr(self, "gripper_process"):
            self.update_gripper_buttons()

    def _evaluate_robot_core_health(
        self,
        robot,
        health,
        now,
        program_sample,
        mode_sample,
    ):
        session_start = self.system_session_started_at
        session_age = (
            now - session_start
            if session_start is not None
            else 0.0
        )
        startup_waiting = (
            session_age < HEALTH_STARTUP_GRACE_SEC
        )

        details = []
        has_error = False
        has_waiting = False

        health_stamp = health.get("stamp")
        health_snapshot_current = (
            health_stamp is not None
            and session_start is not None
            and health_stamp >= session_start
            and now - health_stamp <= HEALTH_SNAPSHOT_STALE_SEC
        )

        if not health_snapshot_current:
            has_waiting = True
            details.append("Health probe: WAITING")
            return "checking", details

        graph_ok = bool(health.get("graph_ok"))
        graph_missing = health.get("graph_missing", [])
        graph_duplicates = health.get("graph_duplicates", [])

        if graph_duplicates:
            has_error = True
            details.append(
                "ROS graph: ERROR (duplicate: "
                + ", ".join(graph_duplicates)
                + ")"
            )
        elif graph_missing:
            if startup_waiting:
                has_waiting = True
                state = "WAITING"
            else:
                has_error = True
                state = "ERROR"
            details.append(
                "ROS graph: "
                + state
                + " (missing: "
                + ", ".join(graph_missing)
                + ")"
            )
        else:
            details.append("ROS graph: OK")

        tf_ok = bool(
            health.get("tf_ok", {}).get(robot, False)
        )
        if tf_ok:
            details.append(
                f"TF world -> {TF_BASE_FRAMES[robot]}: OK"
            )
        else:
            if startup_waiting:
                has_waiting = True
                tf_state = "WAITING"
            else:
                has_error = True
                tf_state = "ERROR"
            details.append(
                f"TF world -> {TF_BASE_FRAMES[robot]}: {tf_state}"
            )

        mode_current = (
            mode_sample is not None
            and session_start is not None
            and mode_sample[0] >= session_start
        )
        robot_mode_running_value = int(
            getattr(RobotMode, "RUNNING", 7)
        )

        if not mode_current:
            if startup_waiting:
                has_waiting = True
                details.append("Robot mode: WAITING")
            else:
                has_error = True
                details.append("Robot mode: ERROR (no current sample)")
        else:
            mode_value = int(mode_sample[1])
            if mode_value == robot_mode_running_value:
                details.append(
                    f"Robot mode: RUNNING ({mode_value})"
                )
            else:
                has_error = True
                details.append(
                    f"Robot mode: ERROR ({mode_value})"
                )

        controller_info = (
            health.get("controllers", {}).get(robot, {})
        )
        response_stamp = controller_info.get(
            "response_stamp"
        )
        states = controller_info.get("states", {})
        controller_error = controller_info.get(
            "error", ""
        )

        response_current = (
            response_stamp is not None
            and session_start is not None
            and response_stamp >= session_start
        )
        response_fresh = (
            response_current
            and now - response_stamp
            <= CONTROLLER_RESPONSE_STALE_SEC
        )

        if not response_fresh:
            if startup_waiting or response_stamp is None:
                has_waiting = True
                manager_state = "WAITING"
            else:
                has_error = True
                manager_state = "ERROR"

            extra = ""
            if controller_error:
                extra = f" ({controller_error})"
            details.append(
                f"controller_manager: {manager_state}{extra}"
            )
        else:
            age = now - response_stamp
            details.append(
                f"controller_manager: OK ({age:.1f}s)"
            )

            support_bad = [
                name
                for name in SUPPORT_CONTROLLERS
                if states.get(name) != "active"
            ]

            if support_bad:
                has_error = True
                details.append(
                    "Support controllers: ERROR ("
                    + ", ".join(
                        f"{name}={states.get(name, 'missing')}"
                        for name in support_bad
                    )
                    + ")"
                )
            else:
                details.append("Support controllers: OK")

            program_current = (
                program_sample is not None
                and session_start is not None
                and program_sample[0] >= session_start
            )

            if program_current:
                program_running = bool(program_sample[1])
                program_transition_stamp = program_sample[0]
            else:
                program_running = False
                program_transition_stamp = session_start

            # Do not compare controller states captured before the latest
            # robot_program_running transition. Wait for a fresh response.
            controller_after_transition = (
                response_stamp is not None
                and program_transition_stamp is not None
                and response_stamp >= program_transition_stamp
            )

            if not controller_after_transition:
                has_waiting = True
                details.append(
                    "Motion controllers: WAITING FOR FRESH STATE"
                )
            else:
                expected = {
                    name: "inactive"
                    for name in MOTION_CONTROLLERS
                }
                if program_running:
                    expected[
                        PRIMARY_MOTION_CONTROLLER
                    ] = "active"

                motion_bad = [
                    name
                    for name, expected_state in expected.items()
                    if states.get(name) != expected_state
                ]

                if motion_bad:
                    transition_recent = (
                        program_current
                        and now - program_sample[0]
                        < PROGRAM_CONTROLLER_TRANSITION_GRACE_SEC
                    )

                    if transition_recent:
                        has_waiting = True
                        details.append(
                            "Motion controllers: SYNCING ("
                            + ", ".join(
                                f"{name}={states.get(name, 'missing')}"
                                for name in motion_bad
                            )
                            + ")"
                        )
                    else:
                        has_error = True
                        details.append(
                            "Motion controllers: ERROR ("
                            + ", ".join(
                                f"{name}={states.get(name, 'missing')}"
                                for name in motion_bad
                            )
                            + ")"
                        )
                else:
                    phase = (
                        "PLAY"
                        if program_running
                        else "WAITING"
                    )
                    details.append(
                        f"Motion controllers: OK ({phase})"
                    )

        program_current = (
            program_sample is not None
            and session_start is not None
            and program_sample[0] >= session_start
        )
        if program_current:
            details.append(
                "Program: "
                + (
                    "RUNNING"
                    if bool(program_sample[1])
                    else "STOPPED"
                )
            )
        else:
            details.append("Program: NO CURRENT SAMPLE")

        details.append(
            "Reverse interface: "
            + (
                "READY"
                if self.robot_reverse_ready_seen[robot]
                else "WAITING"
            )
        )

        if has_error:
            return "error", details
        if has_waiting:
            return "checking", details
        return "ok", details

    def refresh_robot_readiness(self):

        if (
            self.setup_combo.currentText() != "Dual UR7e"
        ):
            return

        system_running = (
            self.status_label.text() == "RUNNING"
        )

        if not system_running:
            return

        # Simulation has no pendant/External Control handshake or hardware
        # controller-manager health path.
        if self.mode_combo.currentText() == "Simulation":
            changed = False
            for robot in ("robot1", "robot2"):
                self.robot_core_health[robot] = "ok"
                if not self.robot_ready[robot]:
                    self.robot_ready[robot] = True
                    changed = True
                self.set_robot_ready_status(
                    robot,
                    "READY",
                    "Simulation mode: hardware health gates are not required.",
                )

            self.set_health_status(
                "SIMULATION",
                "ok",
                "Simulation mode: real-robot health gates are bypassed.",
            )

            if changed:
                self.update_home_buttons()
                self.update_gripper_buttons()

            self.update_robot_ready_summary()
            return

        if self.system_session_started_at is None:
            return

        now = time.monotonic()
        program_states = self.wrench_listener.program_snapshot()
        robot_modes = self.wrench_listener.robot_mode_snapshot()
        health = self.wrench_listener.health_snapshot()
        controls_changed = False
        health_tooltips = []

        for robot in ("robot1", "robot2"):
            program_sample = program_states.get(robot)
            mode_sample = robot_modes.get(robot)

            core_state, details = (
                self._evaluate_robot_core_health(
                    robot,
                    health,
                    now,
                    program_sample,
                    mode_sample,
                )
            )
            self.robot_core_health[robot] = core_state

            program_current = (
                program_sample is not None
                and program_sample[0]
                >= self.system_session_started_at
            )
            program_running = (
                bool(program_sample[1])
                if program_current
                else False
            )

            if core_state == "error":
                desired_ready = False
                desired_status = "HEALTH ERROR"
            elif core_state == "checking":
                desired_ready = False
                desired_status = "HEALTH CHECK..."
            elif not program_current or not program_running:
                desired_ready = False
                desired_status = "WAITING FOR PLAY"
            elif self.robot_reverse_ready_seen[robot]:
                desired_ready = True
                desired_status = "READY"
            else:
                desired_ready = False
                desired_status = "CONNECTING..."

            if self.robot_ready[robot] != desired_ready:
                self.robot_ready[robot] = desired_ready
                controls_changed = True

            display_name = (
                "Robot 1"
                if robot == "robot1"
                else "Robot 2"
            )
            tooltip = (
                display_name
                + " health gates\n"
                + "\n".join(details)
            )
            health_tooltips.append(tooltip)

            self.set_robot_ready_status(
                robot,
                desired_status,
                tooltip,
            )

        core_states = tuple(
            self.robot_core_health[robot]
            for robot in ("robot1", "robot2")
        )

        if all(state == "ok" for state in core_states):
            self.set_health_status(
                "OK",
                "ok",
                "\n\n".join(health_tooltips),
            )
        elif any(state == "error" for state in core_states):
            self.set_health_status(
                "FAULT",
                "fault",
                "\n\n".join(health_tooltips),
            )
        else:
            self.set_health_status(
                "CHECKING...",
                "checking",
                "\n\n".join(health_tooltips),
            )

        self.update_robot_ready_summary()

        if (
            self.robot_ready["robot1"]
            and self.robot_ready["robot2"]
            and self.start_guard_label.isVisible()
        ):
            self.start_guard_label.clear()
            self.start_guard_label.setVisible(False)

        if controls_changed:
            self.update_home_buttons()
            self.update_gripper_buttons()

    def mark_reverse_interface_ready(self, robot):

        if robot not in ("robot1", "robot2"):
            return

        if (
            self.setup_combo.currentText() != "Dual UR7e"
            or self.mode_combo.currentText() != "Real Robot(s)"
            or self.status_label.text() != "RUNNING"
            or self.system_session_started_at is None
        ):
            return

        # This call comes only from the current UI-owned ros_process output,
        # so it is a fresh reverse-interface confirmation for this launch.
        self.robot_reverse_ready_seen[robot] = True
        self.refresh_robot_readiness()

    def mark_reverse_interface_not_ready(self, robot):

        if robot not in ("robot1", "robot2"):
            return

        if (
            self.setup_combo.currentText() != "Dual UR7e"
            or self.mode_combo.currentText() != "Real Robot(s)"
            or self.status_label.text() != "RUNNING"
            or self.system_session_started_at is None
        ):
            return

        # A fresh "connection dropped" or "robot requested program" message
        # belongs to this UI-owned launch and starts a new reverse-interface
        # handshake generation. READY must therefore be earned again.
        self.robot_reverse_ready_seen[robot] = False
        self.refresh_robot_readiness()

    def show_robot_not_ready_warning(self, robot):

        display_name = (
            "Robot 1" if robot == "robot1" else "Robot 2"
        )
        status_label = (
            self.robot1_ready_status
            if robot == "robot1"
            else self.robot2_ready_status
        )
        status = status_label.text()

        if status in ("HEALTH ERROR", "HEALTH CHECK..."):
            message = (
                f"{display_name} health gates are not satisfied. "
                f"Hover the {display_name} status or the Health indicator "
                "for details."
            )
        else:
            message = (
                f"{display_name} is not ready. Press PLAY on the "
                f"{display_name} pendant."
            )

        self.start_guard_label.setText(message)
        self.start_guard_label.setVisible(True)

    # =========================================================
    # Setup / saved-pose motion
    # =========================================================

    @staticmethod
    def _pose_display_name(pose_name):
        if pose_name == "home":
            return "HOME"
        return pose_name.replace("_", " ").replace("-", " ").title()

    def discover_robot_saved_poses(self, robot):
        """Return saved pose names for robot1/robot2 from config/."""

        if robot not in ("robot1", "robot2"):
            return []

        config_dir = (
            Path(__file__).resolve().parents[1]
            / "config"
        )

        poses = []

        home_path = config_dir / f"home_{robot}.yaml"
        if home_path.is_file():
            poses.append("home")

        prefix = f"pose_{robot}_"
        for path in sorted(config_dir.glob(f"{prefix}*.yaml")):
            pose_name = path.stem[len(prefix):]
            if pose_name and pose_name not in poses:
                poses.append(pose_name)

        return poses

    def refresh_robot_pose_menu(self, robot):
        menu = (
            self.robot1_pose_menu
            if robot == "robot1"
            else self.robot2_pose_menu
        )

        menu.clear()
        poses = self.discover_robot_saved_poses(robot)

        if not poses:
            action = menu.addAction("No saved poses")
            action.setEnabled(False)
            return

        for pose_name in poses:
            action = menu.addAction(
                self._pose_display_name(pose_name)
            )
            action.setToolTip(
                f"Saved pose: {pose_name}"
            )
            action.triggered.connect(
                lambda checked=False,
                robot=robot,
                pose_name=pose_name:
                self.move_robot_to_pose(robot, pose_name)
            )

    def update_home_buttons(self):
        """Update UR5 HOME and Dual-UR7e MOVE ROBOT controls."""

        system_running = (
            self.status_label.text() == "RUNNING"
        )

        motion_idle = (
            self.home_process.state()
            == QProcess.NotRunning
            and self.gripper_process.state()
            == QProcess.NotRunning
            and self.experiment_process.state()
            == QProcess.NotRunning
        )

        single = (
            self.setup_combo.currentText()
            == "Single UR5"
        )

        self.ur5_home_button.setEnabled(
            system_running
            and motion_idle
            and single
        )

        dual_real = (
            not single
            and self.mode_combo.currentText() == "Real Robot(s)"
        )

        robot1_ready = (
            self.robot_ready["robot1"]
            if dual_real
            else True
        )
        robot2_ready = (
            self.robot_ready["robot2"]
            if dual_real
            else True
        )

        self.robot1_home_button.setEnabled(
            system_running
            and motion_idle
            and not single
            and robot1_ready
        )

        self.robot2_home_button.setEnabled(
            system_running
            and motion_idle
            and not single
            and robot2_ready
        )

    def move_to_home(self, target):
        """Legacy UR5 HOME entry point; dual robots use saved_pose safely."""

        if target in ("robot1", "robot2"):
            self.move_robot_to_pose(target, "home")
            return

        if target != "ur5":
            return

        if self.status_label.text() != "RUNNING":
            return

        if (
            self.home_process.state()
            != QProcess.NotRunning
        ):
            return

        if self.mode_combo.currentText() == "Real Robot(s)":
            answer = QMessageBox.question(
                self,
                "Move UR5 to HOME",
                "Move UR5 to its saved HOME position?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        command = (
            "ros2 run ur7e_tools home_pose "
            "--target ur5 --move --duration 5.0"
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && exec {command}"
        )

        self.active_setup_motion = (
            "ur5_home",
            "ur5",
            "home",
        )
        self.setup_motion_output_buffer = ""

        self.log_output.appendPlainText(
            f"\n$ {command}\n"
        )

        self.ur5_home_button.setEnabled(False)
        self.robot1_home_button.setEnabled(False)
        self.robot2_home_button.setEnabled(False)

        self.home_process.start(
            "/bin/bash",
            ["-lc", full_command],
        )

    def move_robot_to_pose(self, robot, pose_name):
        """
        Move robot1/robot2 to a named pose through saved_pose.py.

        The UI confirmation is only a user confirmation. The mandatory
        full-path collision/clearance gate remains inside saved_pose.py and
        is run before any trajectory can be submitted.
        """

        if robot not in ("robot1", "robot2"):
            return

        if self.status_label.text() != "RUNNING":
            return

        if self.setup_combo.currentText() != "Dual UR7e":
            return

        if (
            self.home_process.state()
            != QProcess.NotRunning
            or self.gripper_process.state()
            != QProcess.NotRunning
            or self.experiment_process.state()
            != QProcess.NotRunning
        ):
            return

        available = self.discover_robot_saved_poses(robot)
        if pose_name not in available:
            QMessageBox.warning(
                self,
                "Saved pose unavailable",
                f"The saved pose '{pose_name}' is not available for {robot}.",
            )
            return

        dual_real = (
            self.mode_combo.currentText() == "Real Robot(s)"
        )

        if dual_real and not self.robot_ready[robot]:
            self.show_robot_not_ready_warning(robot)
            return

        display_robot = (
            "Robot 1" if robot == "robot1" else "Robot 2"
        )
        display_pose = self._pose_display_name(pose_name)

        # Explicit UI confirmation on real hardware. --yes is passed to the
        # backend only because this dialog has already handled confirmation;
        # it never disables the backend safety gate.
        if dual_real:
            answer = QMessageBox.question(
                self,
                "Move robot to saved pose",
                (
                    f"Move {display_robot} to:\n\n"
                    f"{display_pose}\n\n"
                    "A full-path workcell safety check will run before "
                    "any trajectory is sent."
                ),
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )

            if answer != QMessageBox.Yes:
                return

            # PLAY/readiness may change while the dialog is open.
            if not self.robot_ready[robot]:
                self.show_robot_not_ready_warning(robot)
                return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )
        repo_root = Path(__file__).resolve().parents[1]

        backend_command = (
            "ros2 run ur7e_tools saved_pose "
            f"--target {shlex.quote(robot)} "
            "--move "
            f"--pose {shlex.quote(pose_name)} "
            "--duration 5.0 "
            "--yes"
        )

        # Run from the source repository so newly added saved-pose/safety
        # modules are resolved even before a later install-space refresh.
        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && cd {shlex.quote(str(repo_root))}"
            f" && exec {backend_command}"
        )

        self.active_setup_motion = (
            "saved_pose",
            robot,
            pose_name,
        )
        self.setup_motion_output_buffer = ""

        self.log_output.appendPlainText(
            f"\n$ {backend_command}\n"
        )

        # Serialize setup motion: never allow a second robot move in parallel.
        self.ur5_home_button.setEnabled(False)
        self.robot1_home_button.setEnabled(False)
        self.robot2_home_button.setEnabled(False)

        self.robot1_gripper_move_button.setEnabled(False)
        self.robot2_gripper_move_button.setEnabled(False)
        self.robot1_gripper_slider.setEnabled(False)
        self.robot2_gripper_slider.setEnabled(False)

        self.home_process.start(
            "/bin/bash",
            ["-lc", full_command],
        )

    def read_setup_motion_output(self):
        text = bytes(
            self.home_process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        if not text:
            return

        self.setup_motion_output_buffer += text
        self.log_output.appendPlainText(
            text.rstrip()
        )

        scrollbar = (
            self.log_output.verticalScrollBar()
        )
        scrollbar.setValue(
            scrollbar.maximum()
        )

    def setup_motion_finished(
        self,
        exit_code,
        exit_status,
    ):
        motion = self.active_setup_motion
        output = self.setup_motion_output_buffer

        if motion is None:
            self.update_home_buttons()
            return

        kind, target, pose_name = motion
        display_pose = self._pose_display_name(pose_name)

        if exit_code == 0:
            if kind == "saved_pose":
                self.log_output.appendPlainText(
                    f"\n[POSE completed: {target} -> {display_pose}]"
                )
            else:
                self.log_output.appendPlainText(
                    "\n[HOME completed: ur5]"
                )
        else:
            blocked = "MOVE BLOCKED" in output

            if kind == "saved_pose" and blocked:
                self.log_output.appendPlainText(
                    f"\n[MOVE BLOCKED: {target} -> {display_pose}]"
                )
                QMessageBox.warning(
                    self,
                    "Move blocked",
                    (
                        f"{target} was NOT moved to {display_pose}.\n\n"
                        "The saved-pose safety backend blocked the path.\n"
                        "See ROS 2 output for the collision/clearance details."
                    ),
                )
            else:
                self.log_output.appendPlainText(
                    f"\n[SETUP MOVE failed: {target} -> {display_pose} "
                    f"- exit code {exit_code}]"
                )
                QMessageBox.warning(
                    self,
                    "Robot move failed",
                    (
                        f"The setup move for {target} did not complete.\n\n"
                        "See ROS 2 output for details."
                    ),
                )

        self.active_setup_motion = None
        self.setup_motion_output_buffer = ""
        self.update_home_buttons()
        self.update_gripper_buttons()
        self.update_experiment_controls()

    # =========================================================
    # Experiment execution
    # =========================================================

    def _selected_experiment(self):
        return (
            self.experiment_task_combo.currentText(),
            self.experiment_hand_combo.currentText(),
            self.experiment_control_combo.currentText(),
            self.experiment_motion_combo.currentText(),
        )

    def update_experiment_controls(self):
        if not hasattr(self, "run_experiment_button"):
            return

        dual = (
            self.setup_combo.currentText() == "Dual UR7e"
        )
        system_running = (
            self.status_label.text() == "RUNNING"
        )
        experiment_idle = (
            self.experiment_process.state()
            == QProcess.NotRunning
        )
        analysis_idle = (
            self.analysis_process.state()
            == QProcess.NotRunning
        )
        setup_idle = (
            self.home_process.state()
            == QProcess.NotRunning
            and self.gripper_process.state()
            == QProcess.NotRunning
        )

        real_mode = (
            self.mode_combo.currentText() == "Real Robot(s)"
        )
        robots_ready = (
            self.robot_ready["robot1"]
            and self.robot_ready["robot2"]
        )

        enabled = (
            dual
            and system_running
            and experiment_idle
            and analysis_idle
            and setup_idle
            and (not real_mode or robots_ready)
        )
        self.run_experiment_button.setEnabled(enabled)

        analysis_has_trial = (
            hasattr(self, "analysis_trial_combo")
            and self.analysis_trial_combo.currentData()
            is not None
        )

        analysis_controls_enabled = (
            dual
            and experiment_idle
            and analysis_idle
        )

        if hasattr(self, "analysis_trial_combo"):
            self.analysis_trial_combo.setEnabled(
                analysis_controls_enabled
            )

        if hasattr(self, "refresh_analysis_button"):
            self.refresh_analysis_button.setEnabled(
                analysis_controls_enabled
            )

        self.analyze_trial_button.setEnabled(
            analysis_controls_enabled
            and analysis_has_trial
        )

        if not experiment_idle:
            self.experiment_status_label.setText("RUNNING")
            self.experiment_status_label.setObjectName(
                "connectionTesting"
            )
        elif not system_running:
            self.experiment_status_label.setText("IDLE")
            self.experiment_status_label.setObjectName(
                "connectionUnknown"
            )
        else:
            selection = self._selected_experiment()
            implemented = selection == (
                "Compound",
                "Right",
                "Open-loop",
                "Task Space",
            )
            self.experiment_status_label.setText(
                "READY" if implemented else "NOT READY"
            )
            self.experiment_status_label.setObjectName(
                "connectionReachable"
                if implemented
                else "connectionUnknown"
            )

        self.experiment_status_label.style().unpolish(
            self.experiment_status_label
        )
        self.experiment_status_label.style().polish(
            self.experiment_status_label
        )

    def run_selected_experiment(self):
        if self.status_label.text() != "RUNNING":
            return

        if self.setup_combo.currentText() != "Dual UR7e":
            return

        if (
            self.experiment_process.state()
            != QProcess.NotRunning
            or self.analysis_process.state()
            != QProcess.NotRunning
            or self.home_process.state()
            != QProcess.NotRunning
            or self.gripper_process.state()
            != QProcess.NotRunning
        ):
            return

        selection = self._selected_experiment()
        speed_scale = float(
            self.experiment_speed_spin.value()
        )

        implemented = (
            "Compound",
            "Right",
            "Open-loop",
            "Task Space",
        )

        if selection != implemented:
            QMessageBox.information(
                self,
                "Experiment not implemented",
                (
                    "This experiment combination is not implemented yet:\n\n"
                    f"Task: {selection[0]}\n"
                    f"Hand: {selection[1]}\n"
                    f"Control: {selection[2]}\n"
                    f"Motion: {selection[3]}\n\n"
                    "No robot command was sent."
                ),
            )
            return

        real_mode = (
            self.mode_combo.currentText() == "Real Robot(s)"
        )

        if real_mode:
            if not (
                self.robot_ready["robot1"]
                and self.robot_ready["robot2"]
            ):
                QMessageBox.warning(
                    self,
                    "Robots not ready",
                    "Both robots must be READY before an experiment.",
                )
                return

            answer = QMessageBox.question(
                self,
                "Run experiment",
                (
                    "Run the following REAL experiment?\n\n"
                    "Compound / Right / Open-loop / Task Space\n"
                    f"Speed scale: {speed_scale:.2f} ×\n\n"
                    "The experiment backend will start data acquisition "
                    "and then command Robot 1."
                ),
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return

        project_root = Path(
            os.path.expanduser("~/phd_polishing_experiments")
        )
        backend = (
            project_root
            / "experiment_backend"
            / "compound_right_task_space.py"
        )

        if not backend.is_file():
            QMessageBox.critical(
                self,
                "Experiment backend missing",
                f"Backend not found:\n{backend}",
            )
            return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        backend_module = (
            "experiment_backend.compound_right_task_space"
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && cd {shlex.quote(str(project_root))}"
            " && exec setsid /usr/bin/python3 -u -m "
            f"{shlex.quote(backend_module)}"
            f" --speed-scale {speed_scale:.2f}"
        )

        self.active_experiment = selection

        self.log_output.appendPlainText(
            "\n============================================================\n"
            "RUN EXPERIMENT\n"
            "============================================================\n"
            f"Mode: {self.mode_combo.currentText()}\n"
            f"Task: {selection[0]}\n"
            f"Hand: {selection[1]}\n"
            f"Control: {selection[2]}\n"
            f"Motion: {selection[3]}\n"
            f"Speed scale: {speed_scale:.2f} ×\n"
            f"Backend: {backend}\n"
        )

        self.run_experiment_button.setEnabled(False)
        self.robot1_home_button.setEnabled(False)
        self.robot2_home_button.setEnabled(False)
        self.robot1_gripper_move_button.setEnabled(False)
        self.robot2_gripper_move_button.setEnabled(False)
        self.robot1_gripper_slider.setEnabled(False)
        self.robot2_gripper_slider.setEnabled(False)

        self.experiment_status_label.setText("RUNNING")
        self.experiment_status_label.setObjectName(
            "connectionTesting"
        )

        self.experiment_process.start(
            "/bin/bash",
            ["-lc", full_command],
        )

    def read_experiment_output(self):
        text = bytes(
            self.experiment_process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        if text:
            self.log_output.appendPlainText(
                text.rstrip()
            )
            scrollbar = self.log_output.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

    def experiment_finished(
        self,
        exit_code,
        exit_status,
    ):
        if exit_code == 0:
            self.log_output.appendPlainText(
                "\n[EXPERIMENT COMPLETE]"
            )
            self.experiment_status_label.setText("COMPLETE")
            self.experiment_status_label.setObjectName(
                "connectionReachable"
            )
        else:
            self.log_output.appendPlainText(
                f"\n[EXPERIMENT FAILED - exit code {exit_code}]"
            )
            self.experiment_status_label.setText("FAILED")
            self.experiment_status_label.setObjectName(
                "connectionOffline"
            )

        self.active_experiment = None
        self.update_home_buttons()
        self.update_gripper_buttons()

        # A just-finished run may have created a new completed trial.
        self.refresh_analysis_trials(
            preserve_selection=False
        )
        self.update_experiment_controls()

    def _analysis_trial_label(
        self,
        trial_dir,
        metadata,
    ):
        trial_id = metadata.get(
            "trial_id",
            trial_dir.name,
        )

        timestamp_text = trial_id
        parts = trial_id.split("_")

        if len(parts) >= 3:
            try:
                stamp = datetime.strptime(
                    f"{parts[1]}_{parts[2]}",
                    "%Y%m%d_%H%M%S",
                )
                timestamp_text = stamp.strftime(
                    "%Y-%m-%d %H:%M:%S"
                )
            except ValueError:
                pass

        task = str(
            metadata.get("task", "unknown")
        ).replace("_", " ").title()

        hand = str(
            metadata.get(
                "substituted_human_hand",
                "unknown",
            )
        ).replace("_", " ").title()

        control = str(
            metadata.get("control_mode", "unknown")
        ).replace("_", " ").title()

        motion = str(
            metadata.get("motion_method", "unknown")
        ).replace("_", " ").title()

        return (
            f"{timestamp_text} | "
            f"{task} | {hand} | {control} | {motion}"
        )

    def refresh_analysis_trials(
        self,
        preserve_selection=True,
    ):
        if not hasattr(self, "analysis_trial_combo"):
            return

        previous_path = None

        if preserve_selection:
            previous_path = (
                self.analysis_trial_combo.currentData()
            )

        project_root = Path(
            os.path.expanduser(
                "~/phd_polishing_experiments"
            )
        )
        experiments_root = (
            project_root
            / "results"
            / "experiments"
        )

        completed_trials = []

        if experiments_root.is_dir():
            for trial_dir in experiments_root.iterdir():
                if not trial_dir.is_dir():
                    continue

                metadata_path = (
                    trial_dir
                    / "metadata.json"
                )

                if not metadata_path.is_file():
                    continue

                try:
                    with metadata_path.open(
                        "r",
                        encoding="utf-8",
                    ) as f:
                        metadata = json.load(f)
                except Exception:
                    continue

                timebase = metadata.get(
                    "timebase",
                    {},
                )

                completed = (
                    timebase.get(
                        "trial_end_ros_time_ns"
                    )
                    is not None
                    or metadata.get(
                        "experiment_end_time_utc"
                    )
                    is not None
                )

                if not completed:
                    continue

                completed_trials.append(
                    (
                        trial_dir,
                        metadata,
                    )
                )

        completed_trials.sort(
            key=lambda item: item[0].name,
            reverse=True,
        )

        self.analysis_trial_combo.blockSignals(True)
        self.analysis_trial_combo.clear()

        selected_index = 0

        if completed_trials:
            for index, (
                trial_dir,
                metadata,
            ) in enumerate(completed_trials):

                label = self._analysis_trial_label(
                    trial_dir,
                    metadata,
                )

                self.analysis_trial_combo.addItem(
                    label,
                    str(trial_dir),
                )

                self.analysis_trial_combo.setItemData(
                    index,
                    (
                        f"{trial_dir.name}\n"
                        f"{trial_dir}"
                    ),
                    Qt.ToolTipRole,
                )

                if (
                    previous_path is not None
                    and str(previous_path)
                    == str(trial_dir)
                ):
                    selected_index = index

            self.analysis_trial_combo.setCurrentIndex(
                selected_index
            )

        else:
            self.analysis_trial_combo.addItem(
                "No completed trials found",
                None,
            )
            self.analysis_trial_combo.setCurrentIndex(0)

        self.analysis_trial_combo.blockSignals(False)

        if hasattr(self, "run_experiment_button"):
            self.update_experiment_controls()

    def analyze_selected_trial(self):
        if self.setup_combo.currentText() != "Dual UR7e":
            return

        if (
            self.analysis_process.state()
            != QProcess.NotRunning
            or self.experiment_process.state()
            != QProcess.NotRunning
        ):
            return

        project_root = Path(
            os.path.expanduser("~/phd_polishing_experiments")
        )
        analyzer = (
            project_root
            / "experiment_analysis"
            / "analyze_trial.py"
        )

        if not analyzer.is_file():
            QMessageBox.critical(
                self,
                "Experiment analyzer missing",
                f"Analyzer not found:\n{analyzer}",
            )
            return

        selected_trial = (
            self.analysis_trial_combo.currentData()
        )

        if selected_trial is None:
            QMessageBox.information(
                self,
                "No completed trial",
                "There is no completed experiment trial to analyze.",
            )
            return

        trial_dir = Path(
            str(selected_trial)
        ).expanduser().resolve()

        if not (
            trial_dir.is_dir()
            and (trial_dir / "metadata.json").is_file()
            and (trial_dir / "rosbag").is_dir()
        ):
            QMessageBox.critical(
                self,
                "Invalid experiment trial",
                (
                    "The selected trial is incomplete or missing:\n"
                    f"{trial_dir}"
                ),
            )
            self.refresh_analysis_trials(
                preserve_selection=False
            )
            return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && cd {shlex.quote(str(project_root))}"
            " && exec /usr/bin/python3 -u -m "
            "experiment_analysis.analyze_trial "
            f"{shlex.quote(str(trial_dir))}"
        )

        self.analysis_output_buffer = ""
        self.analysis_report_path = None

        self.log_output.appendPlainText(
            "\n============================================================\n"
            "ANALYZE SELECTED TRIAL\n"
            "============================================================\n"
            f"Trial:    {trial_dir.name}\n"
            f"Path:     {trial_dir}\n"
            f"Analyzer: {analyzer}\n"
        )

        self.analysis_status_label.setText("RUNNING")
        self.analysis_status_label.setObjectName(
            "connectionTesting"
        )
        self.analysis_status_label.style().unpolish(
            self.analysis_status_label
        )
        self.analysis_status_label.style().polish(
            self.analysis_status_label
        )

        self.run_experiment_button.setEnabled(False)
        self.analyze_trial_button.setEnabled(False)
        self.analysis_trial_combo.setEnabled(False)
        self.refresh_analysis_button.setEnabled(False)

        self.analysis_process.start(
            "/bin/bash",
            ["-lc", full_command],
        )

    def read_analysis_output(self):
        text = bytes(
            self.analysis_process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        if not text:
            return

        self.analysis_output_buffer += text
        self.log_output.appendPlainText(
            text.rstrip()
        )

        for line in text.splitlines():
            if line.startswith("ANALYSIS_REPORT="):
                report_text = line.split("=", 1)[1].strip()
                if report_text:
                    self.analysis_report_path = Path(
                        report_text
                    ).expanduser()

        scrollbar = self.log_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def analysis_finished(
        self,
        exit_code,
        exit_status,
    ):
        # Catch a marker split across two QProcess reads.
        if self.analysis_report_path is None:
            for line in self.analysis_output_buffer.splitlines():
                if line.startswith("ANALYSIS_REPORT="):
                    report_text = line.split("=", 1)[1].strip()
                    if report_text:
                        self.analysis_report_path = Path(
                            report_text
                        ).expanduser()
                    break

        if exit_code == 0:
            self.log_output.appendPlainText(
                "\n[ANALYSIS COMPLETE]"
            )
            self.analysis_status_label.setText("COMPLETE")
            self.analysis_status_label.setObjectName(
                "connectionReachable"
            )

            report = self.analysis_report_path
            if report is not None and report.is_file():
                self.log_output.appendPlainText(
                    f"[OPENING REPORT] {report}"
                )
                QProcess.startDetached(
                    "xdg-open",
                    [str(report)],
                )
            else:
                self.log_output.appendPlainText(
                    "[ANALYSIS COMPLETE, but report path was not found]"
                )
        else:
            self.log_output.appendPlainText(
                f"\n[ANALYSIS FAILED - exit code {exit_code}]"
            )
            self.analysis_status_label.setText("FAILED")
            self.analysis_status_label.setObjectName(
                "connectionOffline"
            )

        self.analysis_status_label.style().unpolish(
            self.analysis_status_label
        )
        self.analysis_status_label.style().polish(
            self.analysis_status_label
        )

        self.update_experiment_controls()

    def stop_analysis(self):
        if (
            self.analysis_process.state()
            == QProcess.NotRunning
        ):
            return

        self.analysis_process.terminate()

        if not self.analysis_process.waitForFinished(1500):
            self.analysis_process.kill()
            self.analysis_process.waitForFinished(1000)

    def stop_experiment(self):
        if (
            self.experiment_process.state()
            == QProcess.NotRunning
        ):
            return

        pid = int(self.experiment_process.processId())
        if pid > 0:
            try:
                os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            self.experiment_process.terminate()

    # =========================================================
    # 2FG7 gripper control
    # =========================================================

    def update_gripper_buttons(self):

        system_running = (
            self.status_label.text() == "RUNNING"
        )

        dual = (
            self.setup_combo.currentText() == "Dual UR7e"
        )

        command_idle = (
            self.gripper_process.state()
            == QProcess.NotRunning
        )

        setup_motion_idle = (
            self.home_process.state()
            == QProcess.NotRunning
        )

        experiment_idle = (
            self.experiment_process.state()
            == QProcess.NotRunning
        )

        base_enabled = (
            system_running
            and dual
            and command_idle
            and setup_motion_idle
            and experiment_idle
        )

        dual_real = (
            dual
            and self.mode_combo.currentText() == "Real Robot(s)"
        )

        self.robot1_gripper_move_button.setEnabled(
            base_enabled
            and (
                not dual_real
                or self.robot_ready["robot1"]
            )
        )
        self.robot2_gripper_move_button.setEnabled(
            base_enabled
            and (
                not dual_real
                or self.robot_ready["robot2"]
            )
        )

        slider_enabled = (
            dual
            and command_idle
            and setup_motion_idle
            and experiment_idle
        )
        self.robot1_gripper_slider.setEnabled(slider_enabled)
        self.robot2_gripper_slider.setEnabled(slider_enabled)

    def command_gripper_position(self, robot, value):

        if self.status_label.text() != "RUNNING":
            return

        if self.setup_combo.currentText() != "Dual UR7e":
            return

        if (
            self.gripper_process.state()
            != QProcess.NotRunning
            or self.home_process.state()
            != QProcess.NotRunning
            or self.experiment_process.state()
            != QProcess.NotRunning
        ):
            return

        if robot not in ("robot1", "robot2"):
            return

        if (
            self.mode_combo.currentText() == "Real Robot(s)"
            and not self.robot_ready[robot]
        ):
            self.show_robot_not_ready_warning(robot)
            return

        value = max(0.0, min(1.0, float(value)))

        simulation = (
            self.mode_combo.currentText() == "Simulation"
        )

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        visual_topic = f"/{robot}/gripper_visual/position"
        visual_command = (
            f"timeout 3s ros2 topic pub --once {visual_topic} "
            "std_msgs/msg/Float64 "
            f"'{{data: {value:.3f}}}'"
        )

        if simulation:
            command = visual_command
        else:
            real_service = (
                f"/{robot}/io_and_status_controller/"
                "set_analog_output"
            )
            real_command = (
                f"timeout 5s ros2 service call {real_service} "
                "ur_msgs/srv/SetAnalogOutput "
                f"'{{data: {{pin: 0, domain: 1, state: {value:.3f}}}}}'"
            )

            command = f"{real_command} && {visual_command}"

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && {command}"
        )

        self.active_gripper_command = (robot, value)

        self.log_output.appendPlainText(
            f"\n$ {command}\n"
        )

        self.update_gripper_buttons()
        self.robot1_gripper_move_button.setEnabled(False)
        self.robot2_gripper_move_button.setEnabled(False)
        self.robot1_gripper_slider.setEnabled(False)
        self.robot2_gripper_slider.setEnabled(False)

        self.gripper_process.start(
            "/bin/bash",
            [
                "-lc",
                full_command,
            ],
        )
        self.update_home_buttons()

    def read_gripper_output(self):

        text = bytes(
            self.gripper_process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        if text:
            self.log_output.appendPlainText(
                text.rstrip()
            )

            scrollbar = (
                self.log_output.verticalScrollBar()
            )
            scrollbar.setValue(
                scrollbar.maximum()
            )

    def gripper_command_finished(
        self,
        exit_code,
        exit_status
    ):

        command_info = self.active_gripper_command

        if command_info is not None:
            robot, value = command_info

            if exit_code == 0:
                self.log_output.appendPlainText(
                    f"\n[GRIPPER {robot}: position {value:.2f} completed]"
                )
            else:
                self.log_output.appendPlainText(
                    f"\n[GRIPPER {robot}: position {value:.2f} failed "
                    f"- exit code {exit_code}]"
                )

        self.active_gripper_command = None
        self.update_gripper_buttons()
        self.update_home_buttons()
        self.update_experiment_controls()

    # =========================================================
    # Configuration validation
    # =========================================================

    def validate_configuration(self):

        if (
            self.mode_combo.currentText()
            == "Simulation"
        ):
            return True

        if (
            self.setup_combo.currentText()
            == "Single UR5"
        ):

            ip = self.ur5_ip.text().strip()

            if not self.valid_ip(ip):

                QMessageBox.warning(
                    self,
                    "Invalid IP",
                    f"Invalid UR5 IP address:\n{ip}",
                )

                return False

        else:

            ip1 = (
                self.robot1_ip.text().strip()
            )

            ip2 = (
                self.robot2_ip.text().strip()
            )

            if not self.valid_ip(ip1):

                QMessageBox.warning(
                    self,
                    "Invalid IP",
                    f"Invalid Robot 1 IP address:\n{ip1}",
                )

                return False

            if not self.valid_ip(ip2):

                QMessageBox.warning(
                    self,
                    "Invalid IP",
                    f"Invalid Robot 2 IP address:\n{ip2}",
                )

                return False

        return True

    # =========================================================
    # Build ROS launch command
    # =========================================================

    def build_ros_command(self):

        simulation = (
            self.mode_combo.currentText()
            == "Simulation"
        )

        fake = (
            "true"
            if simulation
            else "false"
        )

        if (
            self.setup_combo.currentText()
            == "Single UR5"
        ):

            robot_type = (
                self.ur5_type.currentText()
            )

            robot_ip = (
                self.ur5_ip.text().strip()
            )

            command = [
                "ros2",
                "launch",
                "ur7e_tools",
                "single_ur5.launch.py",
                f"ur_type:={robot_type}",
                f"robot_ip:={robot_ip}",
                f"use_fake_hardware:={fake}",
                "launch_rviz:=true",
            ]

        else:

            robot1_ip = (
                self.robot1_ip.text().strip()
            )

            robot2_ip = (
                self.robot2_ip.text().strip()
            )

            command = [
                "ros2",
                "launch",
                "ur7e_tools",
                "dual_ur7e.launch.py",
                f"robot1_ip:={robot1_ip}",
                f"robot2_ip:={robot2_ip}",
                f"use_fake_hardware:={fake}",
                "launch_rviz:=true",
            ]

        return command

    # =========================================================
    # Start
    # =========================================================

    def start_system(self):

        if (
            self.ros_process.state()
            != QProcess.NotRunning
        ):
            return

        if not self.validate_configuration():
            return

        if (
            self.setup_combo.currentText() == "Dual UR7e"
            and self.mode_combo.currentText() == "Real Robot(s)"
            and not self.preflight_before_start()
        ):
            return

        if (
            self.setup_combo.currentText() == "Dual UR7e"
            and self.mode_combo.currentText() == "Real Robot(s)"
        ):
            robot1_ready = (
                self.robot1_connection_status.text()
                == "REACHABLE"
            )
            robot2_ready = (
                self.robot2_connection_status.text()
                == "REACHABLE"
            )

            if not (robot1_ready and robot2_ready):
                self.start_guard_label.setText(
                    "Press TEST for both robot connections before "
                    "starting the system."
                )
                self.start_guard_label.setVisible(True)
                return

        self.start_guard_label.clear()
        self.start_guard_label.setVisible(False)

        # Begin a fresh readiness session before launching ROS. Any
        # robot_program_running/controller/TF state from before this point is
        # ignored. The ROS spin thread also clears its health caches/TF buffer.
        self.system_session_started_at = time.monotonic()
        self._ros_output_parse_buffer = ""
        self.wrench_listener.request_health_reset()
        self.reset_robot_readiness("NOT STARTED")
        self.set_health_status(
            "CHECKING...",
            "checking",
            "Waiting for current-session ROS/TF/controller health.",
        )

        command = (
            self.build_ros_command()
        )

        printable_command = (
            shlex.join(command)
        )

        workspace_setup = (
            os.path.expanduser(
                "~/ros2_ws/install/setup.bash"
            )
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && exec setsid {printable_command}"
        )

        self.log_output.clear()

        self.log_output.appendPlainText(
            "$ "
            + printable_command
            + "\n"
        )

        self.set_status(
            "STARTING"
        )

        self.ros_process.start(
            "/bin/bash",
            [
                "-lc",
                full_command,
            ],
        )

    # =========================================================
    # Stop
    # =========================================================

    def stop_system(self):

        if (
            self.experiment_process.state()
            != QProcess.NotRunning
        ):
            self.stop_experiment()

        if (
            self.ros_process.state()
            == QProcess.NotRunning
        ):
            return

        if (
            self.home_process.state()
            != QProcess.NotRunning
        ):
            self.home_process.terminate()

        self.set_status(
            "STOPPING"
        )

        self.wrench_listener.set_health_monitor_enabled(False)
        self.set_health_status(
            "STOPPING",
            "checking",
            "Health monitoring paused during workcell shutdown.",
        )
        self.reset_robot_readiness("NOT STARTED")

        pid = int(
            self.ros_process.processId()
        )

        if pid > 0:

            try:
                os.killpg(
                    pid,
                    signal.SIGINT
                )

            except ProcessLookupError:
                pass

        QTimer.singleShot(
            4000,
            self.force_stop_if_needed,
        )

    def force_stop_if_needed(self):

        if (
            self.ros_process.state()
            == QProcess.NotRunning
        ):
            return

        pid = int(
            self.ros_process.processId()
        )

        if pid > 0:

            try:
                os.killpg(
                    pid,
                    signal.SIGTERM
                )

            except ProcessLookupError:
                pass

    # =========================================================
    # Process events
    # =========================================================

    def on_process_started(self):

        self.set_status(
            "RUNNING"
        )

        self.start_button.setEnabled(
            False
        )

        self.stop_button.setEnabled(
            True
        )

        self.setup_combo.setEnabled(
            False
        )

        self.mode_combo.setEnabled(
            False
        )

        self.ur5_type.setEnabled(
            False
        )

        self.ur5_ip.setEnabled(
            False
        )

        self.robot1_ip.setEnabled(
            False
        )

        self.robot2_ip.setEnabled(
            False
        )

        if self.setup_combo.currentText() == "Dual UR7e":
            if self.mode_combo.currentText() == "Simulation":
                self.wrench_listener.set_health_monitor_enabled(False)
                for robot in ("robot1", "robot2"):
                    self.robot_ready[robot] = True
                    self.robot_reverse_ready_seen[robot] = True
                    self.robot_core_health[robot] = "ok"
                    self.set_robot_ready_status(
                        robot,
                        "READY",
                        "Simulation mode.",
                    )
                self.set_health_status(
                    "SIMULATION",
                    "ok",
                    "Simulation mode: real-robot health gates are bypassed.",
                )
                self.update_robot_ready_summary()
            else:
                self.reset_robot_readiness("NOT STARTED")
                self.wrench_listener.set_health_monitor_enabled(True)
                self.set_health_status(
                    "CHECKING...",
                    "checking",
                    "Waiting for current-session ROS/TF/controller health.",
                )

        self.update_home_buttons()
        self.update_gripper_buttons()
        self.refresh_wrench_display()

    def on_process_finished(
        self,
        exit_code,
        exit_status
    ):

        self.set_status(
            "STOPPED"
        )

        self.start_button.setEnabled(
            True
        )

        self.stop_button.setEnabled(
            False
        )

        self.setup_combo.setEnabled(
            True
        )

        self.mode_combo.setEnabled(
            True
        )

        self.ur5_type.setEnabled(
            True
        )

        self.ur5_ip.setEnabled(
            True
        )

        self.robot1_ip.setEnabled(
            True
        )

        self.robot2_ip.setEnabled(
            True
        )

        self.wrench_listener.set_health_monitor_enabled(False)
        self.system_session_started_at = None
        self._ros_output_parse_buffer = ""
        self.reset_robot_readiness("NOT STARTED")
        self.set_health_status(
            "STOPPED",
            "stopped",
            "Workcell is stopped.",
        )

        self.ur5_home_button.setEnabled(False)
        self.robot1_home_button.setEnabled(False)
        self.robot2_home_button.setEnabled(False)

        self.robot1_gripper_move_button.setEnabled(False)
        self.robot2_gripper_move_button.setEnabled(False)

        self.refresh_wrench_display()

        self.log_output.appendPlainText(
            f"\n[System stopped - exit code {exit_code}]"
        )

    # =========================================================
    # ROS output
    # =========================================================

    def read_ros_output(self):

        text = bytes(
            self.ros_process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        if text:

            self.log_output.appendPlainText(
                text.rstrip()
            )

            # QProcess can split a ROS log message anywhere, including before
            # the final newline. Keep a rolling buffer and parse both complete
            # lines and the current unterminated tail. This is important after
            # a STOP -> PLAY reconnect, where the reverse-interface READY line
            # can otherwise remain buffered forever if no later output arrives.
            self._ros_output_parse_buffer += text

            ready_phrase = (
                "Robot connected to reverse interface. "
                "Ready to receive control commands."
            )
            dropped_phrase = "Connection to reverse interface dropped."
            requested_phrase = "Robot requested program"

            def parse_reverse_interface_message(message):
                robot = None

                if "UR_Client_Library:robot1_" in message:
                    robot = "robot1"
                elif "UR_Client_Library:robot2_" in message:
                    robot = "robot2"

                if robot is None:
                    return False

                # Parse state-reset events before READY. This makes each PLAY
                # a fresh handshake and avoids carrying READY across cycles.
                if dropped_phrase in message:
                    self.mark_reverse_interface_not_ready(robot)
                    return True

                if requested_phrase in message:
                    self.mark_reverse_interface_not_ready(robot)
                    return True

                if ready_phrase in message:
                    self.mark_reverse_interface_ready(robot)
                    return True

                return False

            while "\n" in self._ros_output_parse_buffer:
                line, self._ros_output_parse_buffer = (
                    self._ros_output_parse_buffer.split("\n", 1)
                )
                parse_reverse_interface_message(line)

            # Do not wait for a newline if a complete reverse-interface event
            # is already present in the tail. This also covers reconnects where
            # the final READY line is the last output produced for a while.
            if parse_reverse_interface_message(
                self._ros_output_parse_buffer
            ):
                self._ros_output_parse_buffer = ""

            # Bound the tail in case an unexpected process writes a very long
            # line without newlines. The READY marker is much shorter than this.
            if len(self._ros_output_parse_buffer) > 8192:
                self._ros_output_parse_buffer = (
                    self._ros_output_parse_buffer[-4096:]
                )

            scrollbar = (
                self.log_output.verticalScrollBar()
            )

            scrollbar.setValue(
                scrollbar.maximum()
            )

    # =========================================================
    # System status
    # =========================================================

    def set_status(
        self,
        status
    ):

        self.status_label.setText(
            status
        )

        if status == "RUNNING":

            self.status_label.setObjectName(
                "statusRunning"
            )

        elif status in (
            "STARTING",
            "STOPPING"
        ):

            self.status_label.setObjectName(
                "statusTransition"
            )

        else:

            self.status_label.setObjectName(
                "statusStopped"
            )

        self.status_label.style().unpolish(
            self.status_label
        )

        self.status_label.style().polish(
            self.status_label
        )

        if hasattr(self, "run_experiment_button"):
            self.update_experiment_controls()

    # =========================================================
    # Closing
    # =========================================================

    def closeEvent(
        self,
        event
    ):

        if hasattr(self, "lower_workspace_splitter"):
            self.settings.setValue(
                "lower_workspace_splitter",
                self.lower_workspace_splitter.saveState(),
            )

        if (
            self.ros_process.state()
            != QProcess.NotRunning
        ):

            self.stop_system()

            self.ros_process.waitForFinished(
                3000
            )

        if (
            self.experiment_process.state()
            != QProcess.NotRunning
        ):
            self.stop_experiment()
            self.experiment_process.waitForFinished(2000)

        if (
            self.analysis_process.state()
            != QProcess.NotRunning
        ):
            self.stop_analysis()

        if (
            self.home_process.state()
            != QProcess.NotRunning
        ):
            self.home_process.terminate()
            self.home_process.waitForFinished(1000)

        if hasattr(
            self,
            "internal_ft_zero_processes",
        ):
            for process in (
                self.internal_ft_zero_processes.values()
            ):
                if (
                    process.state()
                    != QProcess.NotRunning
                ):
                    process.terminate()
                    process.waitForFinished(
                        1000
                    )

        if (
            self.ft_process.state()
            != QProcess.NotRunning
        ):
            self.stop_external_ft()
            self.ft_process.waitForFinished(
                2000
            )

        if (
            self.ft_zero_process.state()
            != QProcess.NotRunning
        ):
            self.ft_zero_process.terminate()
            self.ft_zero_process.waitForFinished(
                1000
            )

        if hasattr(self, "wrench_listener"):
            self.wrench_listener.set_health_monitor_enabled(False)

        if hasattr(
            self,
            "wrench_refresh_timer",
        ):
            self.wrench_refresh_timer.stop()

        if hasattr(
            self,
            "robot_state_timer",
        ):
            self.robot_state_timer.stop()

        if hasattr(
            self,
            "nansense_widget",
        ):
            self.nansense_widget.shutdown()

        if hasattr(self, "wrench_listener"):
            self.stop_all_wrench_recordings(silent=True)

        if self._owns_rclpy_context and rclpy.ok():
            rclpy.shutdown()

        if hasattr(
            self,
            "wrench_spin_thread",
        ):
            self.wrench_spin_thread.join(
                timeout=1.0
            )

        if hasattr(
            self,
            "wrench_listener",
        ):
            try:
                self.wrench_listener.destroy_node()
            except Exception:
                pass

        event.accept()

    # =========================================================
    # Style
    # =========================================================

    def apply_style(self):

        self.setStyleSheet(
            """
            QMainWindow,
            QWidget#centralWidget {
                background: #202124;
            }

            QWidget {
                color: #e8eaed;
                font-size: 13px;
            }

            QLabel#title {
                font-size: 22px;
                font-weight: 600;
            }

            QLabel#subtitle {
                color: #9aa0a6;
                margin-bottom: 2px;
            }

            QLabel#robotSectionTitle {
                font-size: 14px;
                font-weight: 700;
            }

            QLabel#wrenchColumnTitle {
                font-size: 12px;
                font-weight: 700;
                color: #bdc1c6;
                margin-bottom: 1px;
            }

            QFrame#robotCard {
                background: #292a2d;
                border: 1px solid #3c4043;
                border-radius: 6px;
            }


            QFrame#recordingBar {
                background: #292a2d;
                border: 1px solid #3c4043;
                border-radius: 6px;
            }

            QFrame#collapsibleBody {
                background: #202124;
                border: 1px solid #3c4043;
                border-top: 0px;
                border-bottom-left-radius: 7px;
                border-bottom-right-radius: 7px;
            }

            QFrame#sensorPanel {
                background: #202124;
                border: 0px;
            }

            QPushButton#collapseButton {
                min-height: 30px;
                text-align: left;
                padding-left: 10px;
                background: #292a2d;
                border: 1px solid #3c4043;
                border-radius: 7px;
                font-weight: 700;
            }

            QPushButton#collapseButton:checked {
                border-bottom-left-radius: 0px;
                border-bottom-right-radius: 0px;
            }

            QPushButton#recordStartButton:enabled {
                background: #245c34;
            }

            QPushButton#recordStopButton:enabled {
                background: #7a2e2a;
            }

            QLabel#recordingActive {
                color: #ff8a80;
                font-weight: 800;
                padding: 1px 5px;
            }

            QGroupBox {
                border: 1px solid #3c4043;
                border-radius: 7px;
                margin-top: 9px;
                padding: 7px;
                font-weight: 600;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
            }

            QLineEdit,
            QPlainTextEdit {
                background: #303134;
                border: 1px solid #5f6368;
                border-radius: 5px;
                padding: 5px;
            }

            QComboBox,
            QDoubleSpinBox {
                background: #303134;
                color: #ffffff;
                border: 1px solid #7a7f85;
                border-radius: 5px;
                padding: 5px 28px 5px 8px;
                min-height: 22px;
                font-weight: 600;
            }

            QComboBox:hover,
            QDoubleSpinBox:hover {
                border: 1px solid #aeb4ba;
                background: #35373a;
            }

            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 30px;
                border-left: 1px solid #5f6368;
            }

            QComboBox QAbstractItemView {
                background: #303134;
                color: #ffffff;
                border: 1px solid #7a7f85;
                selection-background-color: #5f6368;
                selection-color: #ffffff;
                outline: 0px;
                padding: 4px;
            }

            QMenu {
                background: #252628;
                color: #ffffff;
                border: 1px solid #6f7479;
                padding: 4px;
            }

            QMenu::item {
                background: transparent;
                color: #ffffff;
                padding: 7px 28px 7px 12px;
                min-width: 220px;
            }

            QMenu::item:selected {
                background: #4a4d51;
                color: #ffffff;
            }

            QMenu::item:disabled {
                color: #9aa0a6;
            }

            QPushButton {
                min-height: 28px;
                border-radius: 5px;
                padding: 3px 10px;
                font-weight: 600;
                background: #3c4043;
            }

            QPushButton:hover {
                background: #4a4d51;
            }

            QPushButton#startButton {
                background: #2e7d32;
                color: white;
            }

            QPushButton#stopButton {
                background: #b3261e;
                color: white;
            }

            QPushButton:disabled {
                background: #303134;
                color: #777777;
            }

            QLabel#statusRunning {
                background: #254c32;
                border-radius: 5px;
                padding: 2px 6px;
                color: #81c995;
                font-weight: 700;
            }

            QLabel#statusStopped {
                background: #542b29;
                border-radius: 5px;
                padding: 2px 6px;
                color: #f28b82;
                font-weight: 700;
            }

            QLabel#statusTransition {
                background: #55491f;
                border-radius: 5px;
                padding: 2px 6px;
                color: #fdd663;
                font-weight: 700;
            }

            QLabel#connectionReachable {
                background: #254c32;
                border-radius: 5px;
                padding: 2px 6px;
                color: #81c995;
                font-weight: 700;
            }

            QLabel#connectionOffline {
                background: #542b29;
                border-radius: 5px;
                padding: 2px 6px;
                color: #f28b82;
                font-weight: 700;
            }

            QLabel#connectionTesting {
                background: #55491f;
                border-radius: 5px;
                padding: 2px 6px;
                color: #fdd663;
                font-weight: 700;
            }

            QLabel#connectionUnknown {
                background: #303134;
                border-radius: 5px;
                padding: 2px 6px;
                color: #9aa0a6;
                font-weight: 700;
            }

            QLabel#systemWarning {
                color: #fdd663;
                font-weight: 700;
                padding: 2px 4px;
            }

            QLabel#robotStateReady,
            QLabel#robotSummaryReady {
                background: #254c32;
                border-radius: 5px;
                padding: 2px 6px;
                color: #81c995;
                font-weight: 700;
            }

            QLabel#robotStateWaiting,
            QLabel#robotStateConnecting,
            QLabel#robotSummaryPartial {
                background: #55491f;
                border-radius: 5px;
                padding: 2px 6px;
                color: #fdd663;
                font-weight: 700;
            }

            QLabel#robotStateDisconnected {
                background: #542b29;
                border-radius: 5px;
                padding: 2px 6px;
                color: #f28b82;
                font-weight: 700;
            }

            QLabel#robotStateStopped,
            QLabel#robotSummaryUnknown {
                background: #303134;
                border-radius: 5px;
                padding: 2px 6px;
                color: #9aa0a6;
                font-weight: 700;
            }

            QMessageBox {
                background-color: #202124;
            }

            QMessageBox QLabel {
                color: #e8eaed;
                font-size: 13px;
            }

            QMessageBox QPushButton {
                min-width: 70px;
                min-height: 28px;
                background-color: #3c4043;
                color: #ffffff;
                border: 1px solid #5f6368;
                border-radius: 5px;
                padding: 3px 10px;
            }

            QMessageBox QPushButton:hover {
                background-color: #4a4d51;
            }

            QFrame#separator {
                color: #3c4043;
                background: #3c4043;
                max-height: 1px;
                margin-top: 8px;
                margin-bottom: 8px;
            }

            QSplitter::handle {
                background: #3c4043;
                border-radius: 2px;
            }

            QSplitter::handle:hover {
                background: #8ab4f8;
            }
            """
        )


def main(args=None):

    startup_preflight_report = (
        perform_startup_fastdds_preflight()
    )

    app = QApplication(
        sys.argv
    )

    app.setDesktopFileName("Robot Control")

    window = WorkcellUI(
        startup_preflight_report=startup_preflight_report
    )
    window.show()

    sys.exit(
        app.exec()
    )


if __name__ == "__main__":
    main()
