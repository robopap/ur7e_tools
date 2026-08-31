#!/usr/bin/env python3

import socket
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 33333

JOINTS = {
    "Hips",
    "Spine",
    "Spine1",
    "Spine2",
    "Spine3",
    "Neck",
    "Head",
    "HeadTip",
    "LeftShoulder",
    "LeftShoulder2",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "RightShoulder",
    "RightShoulder2",
    "RightArm",
    "RightForeArm",
    "RightHand",
}


def parse_float_triplet(parts, start):
    return (
        float(parts[start]),
        float(parts[start + 1]),
        float(parts[start + 2]),
    )


def normalize_joint_name(name):
    name = name.strip()

    if name.startswith("mixamorig:"):
        name = name[len("mixamorig:"):]

    return name


def parse_packet(data):
    # NANSENSE STUDIO MATLAB row format:
    # JointName, ParentName,
    # PX, PY, PZ          world position [cm]
    # RLX, RLY, RLZ       local rotation [deg]
    # RWX, RWY, RWZ       world rotation [deg]

    text = data.decode("utf-8", errors="ignore")

    frame = {
        "displacement_cm": None,
        "timecode": None,
        "timestamp_ms": None,
        "frame_number": None,
        "joints": {},
    }

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        parts = [p.strip() for p in line.split(",")]
        key = normalize_joint_name(parts[0])

        # Packet header
        if key.lower() == "displacement" and len(parts) >= 4:
            try:
                frame["displacement_cm"] = parse_float_triplet(parts, 1)
            except ValueError:
                pass
            continue

        # Optional timecode-related labelled rows.
        if key.lower() == "timecode" and len(parts) >= 2:
            frame["timecode"] = parts[1]
            continue

        if key.lower() == "timestamp" and len(parts) >= 2:
            try:
                frame["timestamp_ms"] = int(float(parts[1]))
            except ValueError:
                pass
            continue

        if key.lower() in ("framenumber", "frame_number") and len(parts) >= 2:
            try:
                frame["frame_number"] = int(float(parts[1]))
            except ValueError:
                pass
            continue

        # Joint row = name,parent + 9 numerical values.
        if key not in JOINTS or len(parts) < 11:
            continue

        try:
            world_position_cm = parse_float_triplet(parts, 2)
            local_rotation_deg = parse_float_triplet(parts, 5)
            world_rotation_deg = parse_float_triplet(parts, 8)
        except ValueError:
            continue

        frame["joints"][key] = {
            "parent": normalize_joint_name(parts[1]),
            "position_world_cm": world_position_cm,
            "rotation_local_deg": local_rotation_deg,
            "rotation_world_deg": world_rotation_deg,
        }

    return frame


def print_joint(frame, joint_name):
    joint = frame["joints"].get(joint_name)

    if joint is None:
        print(f"{joint_name}: MISSING")
        return

    p = joint["position_world_cm"]
    rl = joint["rotation_local_deg"]
    rw = joint["rotation_world_deg"]

    print(
        f"{joint_name:14s}  "
        f"Pworld[cm]=({p[0]:8.3f}, {p[1]:8.3f}, {p[2]:8.3f})  "
        f"Rlocal[deg]=({rl[0]:8.3f}, {rl[1]:8.3f}, {rl[2]:8.3f})  "
        f"Rworld[deg]=({rw[0]:8.3f}, {rw[1]:8.3f}, {rw[2]:8.3f})"
    )


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening on UDP {UDP_IP}:{UDP_PORT} ...")
    print("Press Ctrl+C to stop.\n")

    packet_count = 0
    rate_t0 = time.monotonic()
    last_summary = 0.0

    try:
        while True:
            data, addr = sock.recvfrom(65535)

            receive_time = time.time()
            frame = parse_packet(data)

            packet_count += 1
            now = time.monotonic()

            # Print a readable verification snapshot once per second.
            if now - last_summary >= 1.0:
                dt = now - rate_t0
                rate = packet_count / dt if dt > 0 else 0.0

                print("\n" + "=" * 120)
                print(f"Source: {addr[0]}:{addr[1]}")
                print(f"Ubuntu receive timestamp: {receive_time:.6f}")
                print(
                    f"Parsed joints: "
                    f"{len(frame['joints'])}/{len(JOINTS)}"
                )
                print(f"Receive rate: {rate:.2f} packets/s")
                print(
                    f"Skeleton displacement [cm]: "
                    f"{frame['displacement_cm']}"
                )

                if frame["timecode"] is not None:
                    print(f"TimeCode: {frame['timecode']}")
                if frame["timestamp_ms"] is not None:
                    print(f"TimeStamp [ms]: {frame['timestamp_ms']}")
                if frame["frame_number"] is not None:
                    print(f"FrameNumber: {frame['frame_number']}")

                print("-" * 120)

                print_joint(frame, "Hips")
                print_joint(frame, "Head")
                print_joint(frame, "LeftHand")
                print_joint(frame, "RightHand")

                packet_count = 0
                rate_t0 = now
                last_summary = now

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        sock.close()


if __name__ == "__main__":
    main()
