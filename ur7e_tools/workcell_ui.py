#!/usr/bin/env python3

import csv
import ipaddress
import os
import shlex
import signal
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped

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

from ur7e_tools.nansense_live_widget import NansenseLiveWidget


FORCE_BAR_LIMIT = 50.0
TORQUE_BAR_LIMIT = 5.0
WRENCH_UI_REFRESH_MS = 50
WRENCH_STALE_SEC = 0.5


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

    # All three currently verified wrench streams run at approximately 100 Hz.
    # Lower CSV rates are obtained by deterministic sample decimation while
    # leaving the ROS acquisition/control streams untouched at full rate.
    NOMINAL_SOURCE_RATE_HZ = 100

    def __init__(self):
        super().__init__("workcell_ui_wrench_listener")

        self._lock = threading.Lock()
        self._latest = {}
        self._recordings = {}
        self._subscriptions = []

        for key, topic in self.TOPICS.items():
            subscription = self.create_subscription(
                WrenchStamped,
                topic,
                lambda msg, sensor_key=key:
                    self._wrench_callback(sensor_key, msg),
                qos_profile_sensor_data,
            )
            self._subscriptions.append(subscription)

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

    def __init__(self):
        super().__init__()

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
        setup_layout = QHBoxLayout(setup_group)
        setup_layout.setSpacing(8)

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
            "10.0.0.2"
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
            "20.0.0.2"
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

        self.nansense_widget = NansenseLiveWidget()
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

        self.start_ping(
            self.robot1_ip.text().strip(),
            self.robot1_ping_process,
            self.robot1_connection_status,
        )

    def robot1_ping_finished(
        self,
        exit_code,
        exit_status
    ):

        if exit_code == 0:

            self.set_connection_status(
                self.robot1_connection_status,
                "REACHABLE"
            )

        else:

            self.set_connection_status(
                self.robot1_connection_status,
                "OFFLINE"
            )

    # =========================================================
    # Robot 2 ping
    # =========================================================

    def test_robot2_connection(self):

        self.start_ping(
            self.robot2_ip.text().strip(),
            self.robot2_ping_process,
            self.robot2_connection_status,
        )

    def robot2_ping_finished(
        self,
        exit_code,
        exit_status
    ):

        if exit_code == 0:

            self.set_connection_status(
                self.robot2_connection_status,
                "REACHABLE"
            )

        else:

            self.set_connection_status(
                self.robot2_connection_status,
                "OFFLINE"
            )

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

        self.robot1_home_button.setEnabled(
            system_running
            and home_idle
            and not single
        )

        self.robot2_home_button.setEnabled(
            system_running
            and home_idle
            and not single
        )

    def move_to_home(self, target):

        if self.status_label.text() != "RUNNING":
            return

        if (
            self.home_process.state()
            != QProcess.NotRunning
        ):
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

        enabled = (
            system_running
            and dual
            and command_idle
        )

        self.robot1_gripper_move_button.setEnabled(enabled)
        self.robot2_gripper_move_button.setEnabled(enabled)

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

        if hasattr(
            self,
            "wrench_refresh_timer",
        ):
            self.wrench_refresh_timer.stop()

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

    app = QApplication(
        sys.argv
    )

    window = WorkcellUI()
    window.show()

    sys.exit(
        app.exec()
    )


if __name__ == "__main__":
    main()