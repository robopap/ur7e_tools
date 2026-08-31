#!/usr/bin/env python3

import socket
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import RadioButtons


UDP_IP = "0.0.0.0"
UDP_PORT = 33333

# ---------------------------------------------------------------------
# Canonical NANSENSE body joints kept by this viewer.
#
# Intentionally excluded:
#   - all finger joints
#   - Jaw / JawTip
#   - LeftEye / RightEye / eye tips
#
# We keep toes because they are useful for a complete lower-body pose.
# ---------------------------------------------------------------------

BODY_JOINTS = {
    # Trunk / head
    "Hips",
    "Spine",
    "Spine1",
    "Spine2",
    "Spine3",
    "Neck",
    "Head",
    "HeadTip",

    # Left arm
    "LeftShoulder",
    "LeftShoulder2",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",

    # Right arm
    "RightShoulder",
    "RightShoulder2",
    "RightArm",
    "RightForeArm",
    "RightHand",

    # Left leg / foot
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToeBase",
    "LeftFootToe",
    "LeftFootToeTip",

    # Right leg / foot
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToeBase",
    "RightFootToe",
    "RightFootToeTip",
}


TRUNK_CHAIN = [
    "Hips",
    "Spine",
    "Spine1",
    "Spine2",
    "Spine3",
    "Neck",
    "Head",
    "HeadTip",
]

LEFT_ARM_CHAIN = [
    "Spine3",
    "LeftShoulder",
    "LeftShoulder2",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
]

RIGHT_ARM_CHAIN = [
    "Spine3",
    "RightShoulder",
    "RightShoulder2",
    "RightArm",
    "RightForeArm",
    "RightHand",
]

LEFT_LEG_CHAIN = [
    "Hips",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToeBase",
    "LeftFootToe",
    "LeftFootToeTip",
]

RIGHT_LEG_CHAIN = [
    "Hips",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToeBase",
    "RightFootToe",
    "RightFootToeTip",
]


VIEW_CHAINS = {
    "Full Body": [
        TRUNK_CHAIN,
        LEFT_ARM_CHAIN,
        RIGHT_ARM_CHAIN,
        LEFT_LEG_CHAIN,
        RIGHT_LEG_CHAIN,
    ],
    "Upper Body": [
        TRUNK_CHAIN,
        LEFT_ARM_CHAIN,
        RIGHT_ARM_CHAIN,
    ],
    "Both Arms": [
        LEFT_ARM_CHAIN,
        RIGHT_ARM_CHAIN,
    ],
    "Left Arm": [
        LEFT_ARM_CHAIN,
    ],
    "Right Arm": [
        RIGHT_ARM_CHAIN,
    ],
}


def normalize_joint_name(name):
    name = name.strip()

    if name.startswith("mixamorig:"):
        name = name[len("mixamorig:"):]

    return name


def parse_triplet(parts, start):
    return (
        float(parts[start]),
        float(parts[start + 1]),
        float(parts[start + 2]),
    )


def parse_packet(data):
    """
    NANSENSE STUDIO MATLAB UDP format:

    JointName, ParentName,
    PX, PY, PZ,          world position [cm]
    RLX, RLY, RLZ,       local rotation [deg]
    RWX, RWY, RWZ        world rotation [deg]
    """
    text = data.decode("utf-8", errors="ignore")

    frame = {
        "receive_time": time.time(),
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
    """
    UDP reception runs independently from plotting.

    NANSENSE may broadcast at ~240 Hz while the viewer refreshes at
    ~30 FPS. The latest complete body frame is kept for display.
    """

    def __init__(self, ip="0.0.0.0", port=33333):
        self.ip = ip
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.2)

        self.lock = threading.Lock()
        self.latest_frame = None

        self.running = False
        self.thread = None

        self.rate = 0.0
        self.rate_count = 0
        self.rate_t0 = time.monotonic()

    def start(self):
        self.running = True
        self.thread = threading.Thread(
            target=self._loop,
            daemon=True,
        )
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

            # Keep frames that contain the core root at minimum.
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

        try:
            self.sock.close()
        except OSError:
            pass

        if self.thread is not None:
            self.thread.join(timeout=1.0)


def display_position(frame, joint_name):
    """
    Center visualization on Hips only.

    Raw NANSENSE data in `frame` remain unchanged.

    Viewer axes:
        X = relative NANSENSE PX
        Y = relative NANSENSE PZ
        Z = -relative NANSENSE PY
    """
    joints = frame["joints"]

    p = joints[joint_name]["position_world_cm"]
    hips = joints["Hips"]["position_world_cm"]

    x = p[0] - hips[0]
    y = p[2] - hips[2]
    z = -(p[1] - hips[1])

    return x, y, z


def main():
    receiver = LatestFrameReceiver(
        UDP_IP,
        UDP_PORT,
    )
    receiver.start()

    print(f"Listening on UDP {UDP_IP}:{UDP_PORT}")
    print("Waiting for NANSENSE data...")

    deadline = time.time() + 5.0

    while receiver.get_latest() is None:
        if time.time() > deadline:
            receiver.stop()
            raise RuntimeError(
                "No NANSENSE frame received within 5 seconds."
            )

        time.sleep(0.01)

    print("NANSENSE frame received. Opening viewer.")

    fig = plt.figure(
        "NANSENSE Live Skeleton",
        figsize=(10, 7),
    )

    # Leave room on the left for the view selector.
    ax = fig.add_axes(
        [0.25, 0.08, 0.72, 0.84],
        projection="3d",
    )

    # Fixed limits around Hips.
    ax.set_xlim(-110, 110)
    ax.set_ylim(-110, 110)
    ax.set_zlim(-110, 110)

    ax.set_xlabel("X [cm]")
    ax.set_ylabel("Y [cm]")
    ax.set_zlabel("Z [cm]")
    ax.set_title("NANSENSE Live Skeleton")

    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass

    ax.view_init(
        elev=12,
        azim=-70,
    )

    # Create enough reusable line artists for the largest view.
    max_chains = max(
        len(chains)
        for chains in VIEW_CHAINS.values()
    )

    line_artists = []

    for _ in range(max_chains):
        line, = ax.plot(
            [],
            [],
            [],
            marker="o",
            linewidth=2,
            markersize=4,
        )
        line_artists.append(line)

    # -----------------------------------------------------------------
    # View selector
    # -----------------------------------------------------------------

    selector_ax = fig.add_axes(
        [0.03, 0.58, 0.17, 0.25]
    )

    selector_ax.set_title(
        "View",
        fontsize=10,
    )

    view_names = list(VIEW_CHAINS.keys())

    radio = RadioButtons(
        selector_ax,
        view_names,
        active=0,
    )

    selected_view = {
        "name": "Full Body"
    }

    def on_view_change(label):
        selected_view["name"] = label

    radio.on_clicked(on_view_change)

    info_text = fig.text(
        0.03,
        0.47,
        "Excluded:\n"
        "• Fingers\n"
        "• Eyes\n"
        "• Jaw",
        fontsize=9,
        va="top",
    )

    status_text = fig.text(
        0.03,
        0.12,
        "",
        fontsize=9,
        va="bottom",
    )

    def clear_line(line):
        line.set_data([], [])
        line.set_3d_properties([])

    def update(_):
        frame = receiver.get_latest()

        if frame is None:
            return line_artists

        chains = VIEW_CHAINS[
            selected_view["name"]
        ]

        # Hide unused artists.
        for line in line_artists:
            clear_line(line)

        for line, chain in zip(
            line_artists,
            chains,
        ):
            available_chain = [
                joint
                for joint in chain
                if joint in frame["joints"]
            ]

            if len(available_chain) < 2:
                continue

            xyz = [
                display_position(frame, joint)
                for joint in available_chain
            ]

            xs = [p[0] for p in xyz]
            ys = [p[1] for p in xyz]
            zs = [p[2] for p in xyz]

            line.set_data(xs, ys)
            line.set_3d_properties(zs)

        age_ms = (
            time.time()
            - frame["receive_time"]
        ) * 1000.0

        status_text.set_text(
            f"UDP: {receiver.rate:5.1f} Hz\n"
            f"Viewer: ~30 FPS\n"
            f"Frame age: {age_ms:5.1f} ms\n"
            f"View: {selected_view['name']}"
        )

        return line_artists

    animation = FuncAnimation(
        fig,
        update,
        interval=33,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
