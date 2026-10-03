#!/usr/bin/env python3
import argparse, csv, json, os, shutil, signal, subprocess, sys, time
from pathlib import Path
import numpy as np

def stop_pg(proc, timeout=8.0):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait_s in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait_s)
            return
        except subprocess.TimeoutExpired:
            pass

def start_bag(out_dir, topics):
    ros2 = shutil.which("ros2")
    if not ros2:
        raise RuntimeError("ros2 executable not found")
    cmd = [ros2, "bag", "record", "-o", str(out_dir),
           "--compression-mode", "message", "--compression-format", "zstd", *topics]
    print("[BAG] " + " ".join(cmd), flush=True)
    return subprocess.Popen(cmd, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def bag_reader(path):
    import rosbag2_py

    # The staging bags are recorded with message-level Zstd compression.
    # On ROS 2 Humble they must be read through the compression-aware reader;
    # SequentialReader alone can expose the bag metadata but not yield usable
    # decompressed Image messages for our exporter.
    compression_reader = getattr(
        rosbag2_py,
        "SequentialCompressionReader",
        None,
    )
    if compression_reader is not None:
        r = compression_reader()
        reader_name = "SequentialCompressionReader"
    else:
        # Fallback for rosbag2_py builds where compression is transparently
        # handled by SequentialReader.
        r = rosbag2_py.SequentialReader()
        reader_name = "SequentialReader"

    r.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    types = {x.name: x.type for x in r.get_all_topics_and_types()}
    print(
        f"[BAG READER] {reader_name} opened {path}",
        flush=True,
    )
    return r, types

def get_msg_cls(type_name):
    from rosidl_runtime_py.utilities import get_message
    return get_message(type_name)

def deserialize(data, cls):
    from rclpy.serialization import deserialize_message
    return deserialize_message(data, cls)

def ffmpeg_start(path, w, h, fps, crf):
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg executable not found")
    cmd = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s:v", f"{w}x{h}", "-r", f"{fps:g}", "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "veryfast",
        "-crf", str(crf), "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(path),
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

def ffmpeg_close(proc):
    if proc is None:
        return
    try:
        proc.stdin.close()
    except Exception:
        pass
    try:
        proc.wait(timeout=60)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)
    if proc.returncode != 0:
        err = b"" if proc.stderr is None else proc.stderr.read()
        raise RuntimeError("ffmpeg failed: " + err.decode(errors="replace"))

def pack_rgb(msg):
    enc = msg.encoding.lower()
    if enc not in ("rgb8", "bgr8"):
        raise RuntimeError(f"Unsupported RGB encoding: {msg.encoding}")
    w, h, step = int(msg.width), int(msg.height), int(msg.step)
    row = w * 3
    raw = memoryview(msg.data)
    if step == row:
        out = bytes(raw[:row*h])
    else:
        buf = bytearray(row*h)
        for y in range(h):
            buf[y*row:(y+1)*row] = raw[y*step:y*step+row]
        out = bytes(buf)
    if enc == "bgr8":
        a = np.frombuffer(out, np.uint8).reshape(h, w, 3)
        out = np.ascontiguousarray(a[:, :, ::-1]).tobytes()
    return out, w, h

def depth_rgb(msg):
    if msg.encoding.lower() not in ("16uc1", "mono16"):
        raise RuntimeError(f"Unsupported depth encoding: {msg.encoding}")
    w, h = int(msg.width), int(msg.height)
    row_vals = int(msg.step)//2
    a = np.frombuffer(msg.data, np.uint16)
    a = a[:h*row_vals].reshape(h, row_vals)[:, :w]
    valid = a > 0
    n = np.clip((a.astype(np.float32)-200.0)/(2500.0-200.0), 0.0, 1.0)
    x = 1.0 - n
    r = np.clip(1.5 - np.abs(4*x - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4*x - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4*x - 1), 0, 1)
    rgb = (np.stack((r,g,b), axis=-1)*255).astype(np.uint8)
    rgb[~valid] = 0
    return np.ascontiguousarray(rgb), w, h

def export_video(bag_dir, topic, out_mp4, out_csv, kind, fps, crf):
    r, types = bag_reader(bag_dir)
    if topic not in types:
        raise RuntimeError(f"Topic not found in bag: {topic}")
    cls = get_msg_cls(types[topic])
    proc = None
    count = 0
    first = last = None
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["frame_idx","stamp_sec","stamp_nanosec","ros_time_s"])
        try:
            while r.has_next():
                t, data, _ = r.read_next()
                if t != topic:
                    continue
                msg = deserialize(data, cls)
                if kind == "rgb":
                    frame, w, h = pack_rgb(msg)
                else:
                    arr, w, h = depth_rgb(msg)
                    frame = arr.tobytes()
                if proc is None:
                    proc = ffmpeg_start(out_mp4, w, h, fps, crf)
                proc.stdin.write(frame)
                sec = int(msg.header.stamp.sec)
                ns = int(msg.header.stamp.nanosec)
                ts = sec + ns*1e-9
                wr.writerow([count, sec, ns, f"{ts:.9f}"])
                first = ts if first is None else first
                last = ts
                count += 1
        finally:
            ffmpeg_close(proc)
    if count == 0:
        raise RuntimeError(
            f"No {kind.upper()} frames were exported from {bag_dir}. "
            f"The staging bag has been preserved for recovery."
        )

    if not Path(out_mp4).is_file() or Path(out_mp4).stat().st_size == 0:
        raise RuntimeError(
            f"{kind.upper()} export processed {count} frame(s), "
            f"but MP4 was not created: {out_mp4}"
        )

    dur = 0.0 if first is None else max(0.0, last-first)
    effective_fps = 0.0
    if count > 1 and dur > 0.0:
        effective_fps = (count - 1) / dur

    size_mib = Path(out_mp4).stat().st_size / (1024 * 1024)
    print(
        f"[{kind.upper()} MP4] {count} frames | ROS duration {dur:.3f}s "
        f"| effective fps {effective_fps:.2f} "
        f"| first {first if first is not None else 0.0:.6f} "
        f"| last {last if last is not None else 0.0:.6f} "
        f"| {size_mib:.1f} MiB",
        flush=True,
    )
    return {
        "frames": count,
        "ros_duration_s": dur,
        "effective_fps": effective_fps,
        "first_ros_time_s": first,
        "last_ros_time_s": last,
        "size_mib": size_mib,
        "path": str(out_mp4),
        "timestamps": str(out_csv),
    }

def summarize_raw_bag(bag_dir, image_topic, kind):
    r, types = bag_reader(bag_dir)
    if image_topic not in types:
        raise RuntimeError(f"Topic not found in bag: {image_topic}")

    cls = get_msg_cls(types[image_topic])
    count = 0
    first = None
    last = None

    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != image_topic:
            continue

        msg = deserialize(data, cls)
        sec = int(msg.header.stamp.sec)
        ns = int(msg.header.stamp.nanosec)
        ts = sec + ns * 1e-9

        if first is None:
            first = ts
        last = ts
        count += 1

    dur = 0.0 if first is None else max(0.0, last - first)
    effective_fps = 0.0
    if count > 1 and dur > 0.0:
        effective_fps = (count - 1) / dur

    print(
        f"[{kind.upper()} RAW] {count} frames | ROS duration {dur:.3f}s "
        f"| effective fps {effective_fps:.2f} "
        f"| first {first if first is not None else 0.0:.6f} "
        f"| last {last if last is not None else 0.0:.6f}",
        flush=True,
    )
    return {
        "frames": count,
        "ros_duration_s": dur,
        "effective_fps": effective_fps,
        "first_ros_time_s": first,
        "last_ros_time_s": last,
        "path": str(bag_dir),
    }


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--camera-key", required=True)
    p.add_argument("--color-topic", required=True)
    p.add_argument("--depth-topic", required=True)
    p.add_argument("--color-info-topic", required=True)
    p.add_argument("--depth-info-topic", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--rgb-mode", choices=("raw","compressed"), default="compressed")
    p.add_argument("--depth-mode", choices=("raw","compressed"), default="compressed")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--crf", type=int, default=20)
    return p.parse_args()

def main():
    a = parse_args()
    root = Path(a.output_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=False)
    stop = False
    def handler(_s, _f):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    capture_dir = root / ".capture_staging"
    capture_proc = None
    started = time.time()
    results = {}
    try:
        capture_topics = [
            a.color_topic,
            a.depth_topic,
            a.color_info_topic,
            a.depth_info_topic,
        ]
        capture_proc = start_bag(capture_dir, capture_topics)

        print(
            f"[READY] {a.camera_key} | single synchronized capture "
            f"| RGB={a.rgb_mode.upper()} | DEPTH={a.depth_mode.upper()}",
            flush=True,
        )

        while not stop:
            if capture_proc.poll() is not None:
                raise RuntimeError(
                    f"camera capture bag exited unexpectedly "
                    f"({capture_proc.returncode})"
                )
            time.sleep(0.1)

        print("[FINALIZING] stopping synchronized capture...", flush=True)
        stop_pg(capture_proc)
        capture_proc = None

        if a.rgb_mode == "compressed":
            print("[FINALIZING] RGB -> H.264 MP4...", flush=True)
            results["rgb"] = export_video(
                capture_dir,
                a.color_topic,
                root/"rgb.mp4",
                root/"rgb_timestamps.csv",
                "rgb",
                a.fps,
                a.crf,
            )
        else:
            results["rgb"] = summarize_raw_bag(
                capture_dir, a.color_topic, "rgb"
            )

        if a.depth_mode == "compressed":
            print("[FINALIZING] DEPTH -> colorized H.264 MP4...", flush=True)
            results["depth"] = export_video(
                capture_dir,
                a.depth_topic,
                root/"depth.mp4",
                root/"depth_timestamps.csv",
                "depth",
                a.fps,
                a.crf,
            )
        else:
            results["depth"] = summarize_raw_bag(
                capture_dir, a.depth_topic, "depth"
            )

        # If both streams are compressed, the synchronized staging bag is no
        # longer needed and is deleted. If either stream is RAW, keep the
        # synchronized bag as raw_bag so no sensor data is lost.
        if a.rgb_mode == "compressed" and a.depth_mode == "compressed":
            shutil.rmtree(capture_dir)
        else:
            raw_dir = root / "raw_bag"
            capture_dir.rename(raw_dir)
            for stream_name in ("rgb", "depth"):
                if stream_name in results and results[stream_name].get("path") == str(capture_dir):
                    results[stream_name]["path"] = str(raw_dir)

        for stream_name in ("rgb", "depth"):
            summary = results.get(stream_name)
            if not summary:
                continue
            mode_name = a.rgb_mode if stream_name == "rgb" else a.depth_mode
            print(
                f"[VERIFY] {a.camera_key} | {stream_name.upper()} "
                f"{mode_name.upper()} | {summary['frames']} frames "
                f"| {summary['ros_duration_s']:.3f}s "
                f"| {summary['effective_fps']:.2f} fps",
                flush=True,
            )

        # Extra synchronization verification for the two image streams.
        rgb_summary = results.get("rgb", {})
        depth_summary = results.get("depth", {})
        if rgb_summary and depth_summary:
            rgb_first = rgb_summary.get("first_ros_time_s")
            depth_first = depth_summary.get("first_ros_time_s")
            rgb_last = rgb_summary.get("last_ros_time_s")
            depth_last = depth_summary.get("last_ros_time_s")

            start_delta = (
                abs(rgb_first - depth_first)
                if rgb_first is not None and depth_first is not None
                else None
            )
            end_delta = (
                abs(rgb_last - depth_last)
                if rgb_last is not None and depth_last is not None
                else None
            )

            if start_delta is not None and end_delta is not None:
                print(
                    f"[SYNC] {a.camera_key} | "
                    f"RGB-depth first-frame delta {start_delta:.3f}s "
                    f"| last-frame delta {end_delta:.3f}s",
                    flush=True,
                )

        (root/"recording_modes.json").write_text(json.dumps({
            "camera": a.camera_key,
            "rgb_mode": a.rgb_mode,
            "depth_mode": a.depth_mode,
            "wall_duration_s": time.time()-started,
            "fps": a.fps,
            "crf": a.crf,
            "outputs": results,
        }, indent=2), encoding="utf-8")
        print(f"[SAVED] {root}", flush=True)
        return 0
    except Exception as e:
        print(f"[FATAL] {e}", file=sys.stderr, flush=True)
        return 1
    finally:
        stop_pg(capture_proc)

if __name__ == "__main__":
    raise SystemExit(main())
