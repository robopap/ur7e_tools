#!/usr/bin/env python3

import csv
import ipaddress
import glob
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
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from ur_dashboard_msgs.msg import RobotMode
from visualization_msgs.msg import Marker, MarkerArray

from PySide6.QtCore import QProcess, QSettings, QTimer, Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import (
    QApplication,
    QAbstractScrollArea,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSlider,
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

    def update_nansense_frame(self, frame):
        with self._nansense_lock:
            self._latest_nansense_frame = frame

    def update_nansense_calibration(self, calibration):
        with self._nansense_lock:
            self._nansense_calibration = {
                key: float(calibration[key])
                for key in NANSENSE_DEFAULT_CALIBRATION
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
        x_m = position_world_cm[0] * 0.01
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
            MarkerArray(markers=[joint_marker, bone_marker])
        )

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
        # HOME motion process
        # -----------------------------------------------------

        self.home_process = QProcess(self)
        self.home_process.setProcessChannelMode(QProcess.MergedChannels)

        self.home_process.readyReadStandardOutput.connect(
            self.read_home_output
        )
        self.home_process.finished.connect(
            self.home_motion_finished
        )

        self.active_home_target = None

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
            "MOVE TO HOME"
        )
        self.robot1_home_button.setEnabled(False)
        self.robot1_home_button.clicked.connect(
            lambda: self.move_to_home("robot1")
        )
        self.robot1_home_button.setToolTip(
            "Move Robot 1 to its saved HOME joint configuration."
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
            "MOVE"
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
            "MOVE TO HOME"
        )
        self.robot2_home_button.setEnabled(False)
        self.robot2_home_button.clicked.connect(
            lambda: self.move_to_home("robot2")
        )
        self.robot2_home_button.setToolTip(
            "Move Robot 2 to its saved HOME joint configuration."
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
            "MOVE"
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

        lower_workspace = QWidget()
        lower_workspace_layout = QHBoxLayout(lower_workspace)
        lower_workspace_layout.setContentsMargins(0, 0, 0, 0)
        lower_workspace_layout.setSpacing(10)

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
        )
        self.nansense_widget.setMinimumWidth(520)

        lower_workspace_layout.addWidget(
            sensor_scroll,
            1,
        )
        lower_workspace_layout.addWidget(
            self.nansense_widget,
            1,
        )

        main_layout.addWidget(
            lower_workspace,
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
    # HOME motion
    # =========================================================

    def update_home_buttons(self):

        system_running = (
            self.status_label.text() == "RUNNING"
        )

        home_idle = (
            self.home_process.state()
            == QProcess.NotRunning
        )

        single = (
            self.setup_combo.currentText()
            == "Single UR5"
        )

        self.ur5_home_button.setEnabled(
            system_running
            and home_idle
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
            and home_idle
            and not single
            and robot1_ready
        )

        self.robot2_home_button.setEnabled(
            system_running
            and home_idle
            and not single
            and robot2_ready
        )

    def move_to_home(self, target):

        if self.status_label.text() != "RUNNING":
            return

        if (
            self.home_process.state()
            != QProcess.NotRunning
        ):
            return

        dual_real_target = (
            target in ("robot1", "robot2")
            and self.setup_combo.currentText() == "Dual UR7e"
            and self.mode_combo.currentText() == "Real Robot(s)"
        )

        if (
            dual_real_target
            and not self.robot_ready[target]
        ):
            self.show_robot_not_ready_warning(target)
            return

        # Require explicit confirmation on real hardware.
        if (
            self.mode_combo.currentText()
            == "Real Robot(s)"
        ):

            display_names = {
                "ur5": "UR5",
                "robot1": "Robot 1",
                "robot2": "Robot 2",
            }

            answer = QMessageBox.question(
                self,
                "Move to HOME",
                f"Move {display_names[target]} to its saved HOME position?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )

            if answer != QMessageBox.Yes:
                return

            # Re-check after the confirmation dialog. PLAY may have been
            # stopped while the dialog was open.
            if (
                dual_real_target
                and not self.robot_ready[target]
            ):
                self.show_robot_not_ready_warning(target)
                return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        command = (
            f"ros2 run ur7e_tools home_pose "
            f"--target {target} "
            f"--move "
            f"--duration 5.0"
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && exec {command}"
        )

        self.active_home_target = target

        self.log_output.appendPlainText(
            f"\n$ {command}\n"
        )

        # Prevent a second HOME command until this one finishes.
        self.ur5_home_button.setEnabled(False)
        self.robot1_home_button.setEnabled(False)
        self.robot2_home_button.setEnabled(False)

        self.home_process.start(
            "/bin/bash",
            [
                "-lc",
                full_command,
            ],
        )

    def read_home_output(self):

        text = bytes(
            self.home_process.readAllStandardOutput()
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

    def home_motion_finished(
        self,
        exit_code,
        exit_status
    ):

        target = self.active_home_target

        if exit_code == 0:

            self.log_output.appendPlainText(
                f"\n[HOME completed: {target}]"
            )

        else:

            self.log_output.appendPlainText(
                f"\n[HOME failed: {target} "
                f"- exit code {exit_code}]"
            )

        self.active_home_target = None
        self.update_home_buttons()

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

        base_enabled = (
            system_running
            and dual
            and command_idle
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

        slider_enabled = dual and command_idle
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

    # =========================================================
    # Closing
    # =========================================================

    def closeEvent(
        self,
        event
    ):

        if (
            self.ros_process.state()
            != QProcess.NotRunning
        ):

            self.stop_system()

            self.ros_process.waitForFinished(
                3000
            )

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

            QComboBox {
                background: #303134;
                color: #ffffff;
                border: 1px solid #7a7f85;
                border-radius: 5px;
                padding: 5px 28px 5px 8px;
                min-height: 22px;
                font-weight: 600;
            }

            QComboBox:hover {
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
            """
        )


def main(args=None):

    startup_preflight_report = (
        perform_startup_fastdds_preflight()
    )

    app = QApplication(
        sys.argv
    )

    window = WorkcellUI(
        startup_preflight_report=startup_preflight_report
    )
    window.show()

    sys.exit(
        app.exec()
    )


if __name__ == "__main__":
    main()
