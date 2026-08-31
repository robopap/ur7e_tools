#!/usr/bin/env python3
import socket
import threading
import time
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox
)

from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

UDP_IP = "0.0.0.0"
UDP_PORT = 33333

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

class NansenseLiveWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.receiver = LatestFrameReceiver()

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

        top.addStretch(1)

        self.status_label = QLabel("Disconnected")
        top.addWidget(self.status_label)

        self.main_layout.addLayout(top)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.main_layout.addWidget(self.canvas, stretch=1)

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

        self.timer = QTimer(self)
        self.timer.setInterval(33)  # ~30 FPS visualization
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

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
        self.connect_button.setText("CONNECT NANSENSE")
        self.status_label.setText("Disconnected")
        self.clear_plot()
        self.canvas.draw_idle()

    def clear_plot(self):
        for line in self.line_artists:
            line.set_data([], [])
            line.set_3d_properties([])

    def update_plot(self):
        if not self.receiver.running:
            return

        frame = self.receiver.get_latest()
        if frame is None:
            self.status_label.setText(f"Waiting for UDP :{UDP_PORT}...")
            return

        chains = VIEW_CHAINS[self.view_combo.currentText()]
        self.clear_plot()

        for line, chain in zip(self.line_artists, chains):
            available_chain = [joint for joint in chain if joint in frame["joints"]]
            if len(available_chain) < 2:
                continue

            xyz = [display_position(frame, joint) for joint in available_chain]
            xs = [p[0] for p in xyz]
            ys = [p[1] for p in xyz]
            zs = [p[2] for p in xyz]

            line.set_data(xs, ys)
            line.set_3d_properties(zs)

        age_ms = (time.time() - frame["receive_time"]) * 1000.0
        self.status_label.setText(
            f"Connected | UDP {self.receiver.rate:.1f} Hz | frame {age_ms:.1f} ms"
        )
        self.canvas.draw_idle()

    def shutdown(self):
        self.timer.stop()
        self.receiver.stop()

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
