#!/usr/bin/env python3

import ipaddress
import os
import shlex
import signal
import sys

from PySide6.QtCore import QProcess, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class WorkcellUI(QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Robot Workcell Control")
        self.resize(780, 720)

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
        # Simulation-only 2FG7 visualization commands
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

        self.build_ui()
        self.apply_style()
        self.update_setup_view()

    # =========================================================
    # UI
    # =========================================================

    def build_ui(self):

        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(14)

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
        # Setup selection
        # -----------------------------------------------------

        setup_group = QGroupBox("System configuration")
        setup_form = QFormLayout(setup_group)

        self.setup_combo = QComboBox()
        self.setup_combo.addItems([
            "Single UR5",
            "Dual UR7e",
        ])

        self.setup_combo.currentIndexChanged.connect(
            self.update_setup_view
        )

        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "Simulation",
            "Real Robot(s)",
        ])

        setup_form.addRow(
            "Robot setup:",
            self.setup_combo
        )

        setup_form.addRow(
            "Operating mode:",
            self.mode_combo
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

        dual_layout = QVBoxLayout(
            self.dual_group
        )

        # -----------------------------------------------------
        # Robot 1
        # -----------------------------------------------------

        robot1_title = QLabel(
            "Robot 1"
        )
        robot1_title.setObjectName(
            "robotSectionTitle"
        )

        dual_layout.addWidget(
            robot1_title
        )

        robot1_connection_layout = QHBoxLayout()

        self.robot1_ip = QLineEdit(
            "10.0.0.1"
        )

        self.robot1_test_button = QPushButton(
            "TEST"
        )

        self.robot1_test_button.clicked.connect(
            self.test_robot1_connection
        )

        robot1_connection_layout.addWidget(
            QLabel("IP:")
        )

        robot1_connection_layout.addWidget(
            self.robot1_ip,
            1
        )

        robot1_connection_layout.addWidget(
            self.robot1_test_button
        )

        dual_layout.addLayout(
            robot1_connection_layout
        )

        robot1_status_layout = QHBoxLayout()

        robot1_status_layout.addWidget(
            QLabel("Connection:")
        )

        self.robot1_connection_status = QLabel(
            "NOT TESTED"
        )
        self.robot1_connection_status.setObjectName(
            "connectionUnknown"
        )

        robot1_status_layout.addWidget(
            self.robot1_connection_status
        )

        robot1_status_layout.addStretch()

        dual_layout.addLayout(
            robot1_status_layout
        )

        self.robot1_home_button = QPushButton(
            "MOVE ROBOT 1 TO HOME"
        )

        self.robot1_home_button.setEnabled(
            False
        )

        self.robot1_home_button.clicked.connect(
            lambda: self.move_to_home("robot1")
        )

        self.robot1_home_button.setToolTip(
            "Move Robot 1 to its saved HOME joint configuration."
        )

        dual_layout.addWidget(
            self.robot1_home_button
        )

        robot1_gripper_layout = QHBoxLayout()

        self.robot1_gripper_open_button = QPushButton(
            "OPEN GRIPPER"
        )
        self.robot1_gripper_close_button = QPushButton(
            "CLOSE GRIPPER"
        )

        self.robot1_gripper_open_button.setEnabled(False)
        self.robot1_gripper_close_button.setEnabled(False)

        self.robot1_gripper_open_button.clicked.connect(
            lambda: self.command_gripper("robot1", "open")
        )
        self.robot1_gripper_close_button.clicked.connect(
            lambda: self.command_gripper("robot1", "close")
        )

        self.robot1_gripper_open_button.setToolTip(
            "Simulation only: open the Robot 1 2FG7 in RViz."
        )
        self.robot1_gripper_close_button.setToolTip(
            "Simulation only: close the Robot 1 2FG7 in RViz."
        )

        robot1_gripper_layout.addWidget(
            self.robot1_gripper_open_button
        )
        robot1_gripper_layout.addWidget(
            self.robot1_gripper_close_button
        )

        dual_layout.addLayout(
            robot1_gripper_layout
        )

        # Separator
        separator = QFrame()
        separator.setFrameShape(
            QFrame.HLine
        )
        separator.setObjectName(
            "separator"
        )

        dual_layout.addWidget(
            separator
        )

        # -----------------------------------------------------
        # Robot 2
        # -----------------------------------------------------

        robot2_title = QLabel(
            "Robot 2"
        )
        robot2_title.setObjectName(
            "robotSectionTitle"
        )

        dual_layout.addWidget(
            robot2_title
        )

        robot2_connection_layout = QHBoxLayout()

        self.robot2_ip = QLineEdit(
            "20.0.0.1"
        )

        self.robot2_test_button = QPushButton(
            "TEST"
        )

        self.robot2_test_button.clicked.connect(
            self.test_robot2_connection
        )

        robot2_connection_layout.addWidget(
            QLabel("IP:")
        )

        robot2_connection_layout.addWidget(
            self.robot2_ip,
            1
        )

        robot2_connection_layout.addWidget(
            self.robot2_test_button
        )

        dual_layout.addLayout(
            robot2_connection_layout
        )

        robot2_status_layout = QHBoxLayout()

        robot2_status_layout.addWidget(
            QLabel("Connection:")
        )

        self.robot2_connection_status = QLabel(
            "NOT TESTED"
        )
        self.robot2_connection_status.setObjectName(
            "connectionUnknown"
        )

        robot2_status_layout.addWidget(
            self.robot2_connection_status
        )

        robot2_status_layout.addStretch()

        dual_layout.addLayout(
            robot2_status_layout
        )

        self.robot2_home_button = QPushButton(
            "MOVE ROBOT 2 TO HOME"
        )

        self.robot2_home_button.setEnabled(
            False
        )

        self.robot2_home_button.clicked.connect(
            lambda: self.move_to_home("robot2")
        )

        self.robot2_home_button.setToolTip(
            "Move Robot 2 to its saved HOME joint configuration."
        )

        dual_layout.addWidget(
            self.robot2_home_button
        )

        robot2_gripper_layout = QHBoxLayout()

        self.robot2_gripper_open_button = QPushButton(
            "OPEN GRIPPER"
        )
        self.robot2_gripper_close_button = QPushButton(
            "CLOSE GRIPPER"
        )

        self.robot2_gripper_open_button.setEnabled(False)
        self.robot2_gripper_close_button.setEnabled(False)

        self.robot2_gripper_open_button.clicked.connect(
            lambda: self.command_gripper("robot2", "open")
        )
        self.robot2_gripper_close_button.clicked.connect(
            lambda: self.command_gripper("robot2", "close")
        )

        self.robot2_gripper_open_button.setToolTip(
            "Simulation only: open the Robot 2 2FG7 in RViz."
        )
        self.robot2_gripper_close_button.setToolTip(
            "Simulation only: close the Robot 2 2FG7 in RViz."
        )

        robot2_gripper_layout.addWidget(
            self.robot2_gripper_open_button
        )
        robot2_gripper_layout.addWidget(
            self.robot2_gripper_close_button
        )

        dual_layout.addLayout(
            robot2_gripper_layout
        )

        main_layout.addWidget(
            self.dual_group
        )

        # =====================================================
        # System controls
        # =====================================================

        controls = QFrame()
        controls_layout = QHBoxLayout(
            controls
        )

        controls_layout.setContentsMargins(
            0,
            0,
            0,
            0
        )

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
        self.stop_button.setEnabled(
            False
        )

        self.stop_button.clicked.connect(
            self.stop_system
        )

        controls_layout.addWidget(
            self.start_button
        )

        controls_layout.addWidget(
            self.stop_button
        )

        main_layout.addWidget(
            controls
        )

        # -----------------------------------------------------
        # System status
        # -----------------------------------------------------

        status_layout = QHBoxLayout()

        status_layout.addWidget(
            QLabel("System status:")
        )

        self.status_label = QLabel(
            "STOPPED"
        )

        self.status_label.setObjectName(
            "statusStopped"
        )

        status_layout.addWidget(
            self.status_label
        )

        status_layout.addStretch()

        main_layout.addLayout(
            status_layout
        )

        # -----------------------------------------------------
        # ROS output
        # -----------------------------------------------------

        log_group = QGroupBox(
            "ROS 2 output"
        )

        log_layout = QVBoxLayout(
            log_group
        )

        self.log_output = QPlainTextEdit()

        self.log_output.setReadOnly(
            True
        )

        self.log_output.setPlaceholderText(
            "ROS launch output will appear here..."
        )

        log_layout.addWidget(
            self.log_output
        )

        main_layout.addWidget(
            log_group,
            1
        )

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

        if hasattr(self, "home_process"):
            self.update_home_buttons()

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
    # Simulation-only 2FG7 visualization control
    # =========================================================

    def update_gripper_buttons(self):

        system_running = (
            self.status_label.text() == "RUNNING"
        )

        simulation = (
            self.mode_combo.currentText() == "Simulation"
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
            and simulation
            and dual
            and command_idle
        )

        self.robot1_gripper_open_button.setEnabled(enabled)
        self.robot1_gripper_close_button.setEnabled(enabled)
        self.robot2_gripper_open_button.setEnabled(enabled)
        self.robot2_gripper_close_button.setEnabled(enabled)

    def command_gripper(self, robot, action):

        if self.status_label.text() != "RUNNING":
            return

        if self.mode_combo.currentText() != "Simulation":
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

        if action not in ("open", "close"):
            return

        workspace_setup = os.path.expanduser(
            "~/ros2_ws/install/setup.bash"
        )

        service = f"/{robot}/gripper_visual/{action}"
        command = (
            f"ros2 service call {service} "
            'std_srvs/srv/Trigger "{}"'
        )

        full_command = (
            "source /opt/ros/humble/setup.bash"
            f" && source {shlex.quote(workspace_setup)}"
            f" && exec {command}"
        )

        self.active_gripper_command = (robot, action)

        self.log_output.appendPlainText(
            f"\n$ {command}\n"
        )

        self.update_gripper_buttons()
        self.robot1_gripper_open_button.setEnabled(False)
        self.robot1_gripper_close_button.setEnabled(False)
        self.robot2_gripper_open_button.setEnabled(False)
        self.robot2_gripper_close_button.setEnabled(False)

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
            robot, action = command_info

            if exit_code == 0:
                self.log_output.appendPlainText(
                    f"\n[GRIPPER {robot}: {action.upper()} completed]"
                )
            else:
                self.log_output.appendPlainText(
                    f"\n[GRIPPER {robot}: {action.upper()} failed "
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

        self.robot1_gripper_open_button.setEnabled(False)
        self.robot1_gripper_close_button.setEnabled(False)
        self.robot2_gripper_open_button.setEnabled(False)
        self.robot2_gripper_close_button.setEnabled(False)

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

        event.accept()

    # =========================================================
    # Style
    # =========================================================

    def apply_style(self):

        self.setStyleSheet(
            """
            QMainWindow {
                background: #202124;
            }

            QWidget {
                color: #e8eaed;
                font-size: 14px;
            }

            QLabel#title {
                font-size: 26px;
                font-weight: 600;
            }

            QLabel#subtitle {
                color: #9aa0a6;
                margin-bottom: 8px;
            }

            QLabel#robotSectionTitle {
                font-size: 15px;
                font-weight: 700;
            }

            QGroupBox {
                border: 1px solid #3c4043;
                border-radius: 8px;
                margin-top: 12px;
                padding: 12px;
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
                padding: 7px;
            }

            QComboBox {
                background: #303134;
                color: #ffffff;
                border: 1px solid #7a7f85;
                border-radius: 5px;
                padding: 7px 32px 7px 10px;
                min-height: 24px;
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
                min-height: 36px;
                border-radius: 6px;
                padding: 5px 16px;
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
                color: #81c995;
                font-weight: 700;
            }

            QLabel#statusStopped {
                color: #f28b82;
                font-weight: 700;
            }

            QLabel#statusTransition {
                color: #fdd663;
                font-weight: 700;
            }

            QLabel#connectionReachable {
                color: #81c995;
                font-weight: 700;
            }

            QLabel#connectionOffline {
                color: #f28b82;
                font-weight: 700;
            }

            QLabel#connectionTesting {
                color: #fdd663;
                font-weight: 700;
            }

            QLabel#connectionUnknown {
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