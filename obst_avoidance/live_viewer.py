"""Monitor CheapStage on a live camera or replay a recording.

Live mode reads telemetry and never arms or sends flight commands.
Replay does not import or initialize ROS/MAVLink.
"""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .frame_source import LogFrameSource
from .perception import CheapStage
from .perception.features import feature_names

WINDOW = "CheapStage - camera and sector scores"
PANEL_HEIGHT = 130


def score_color(value):
    """BGR relative-score color; unknown sectors are grey."""
    if not np.isfinite(value):
        return (125, 125, 125)
    value = float(np.clip(value, 0.0, 1.0))
    return (70, int(70 + 170 * value), int(240 - 170 * value))


def _text(image, text, xy, color=(235, 235, 235), scale=0.45, center=False):
    x, y = xy
    if center:
        x -= cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] // 2
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_overlay(rgb, belief, features, tracks=None, label="LIVE MONITOR"):
    """Render actual inference outputs without mutating image or belief."""
    height, width = rgb.shape[:2]
    n = len(belief.scores)
    values = dict(zip(feature_names(n), features))
    canvas = np.full((height + PANEL_HEIGHT, width, 3), 25, dtype=np.uint8)
    canvas[:height] = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    bounds = np.ceil(np.linspace(0, width, n + 1)).astype(int)
    for x in bounds[1:-1]:
        cv2.line(canvas, (x, 0), (x, height + 90), (150, 150, 150), 1)
    if tracks is not None:
        for before, after in zip(*tracks):
            if np.all(np.isfinite(before)) and np.all(np.isfinite(after)):
                a, b = tuple(np.rint(before).astype(int)), tuple(np.rint(after).astype(int))
                cv2.line(canvas[:height], a, b, (80, 240, 80), 1, cv2.LINE_AA)
                cv2.circle(canvas[:height], b, 2, (60, 230, 255), -1)
    cv2.rectangle(canvas, (0, 0), (width, 47), (25, 25, 25), -1)
    _text(canvas, label, (8, 18))
    _text(canvas, f"Valid {int(belief.valid.sum())}/{n}   Confidence {belief.confidence:.2f}   Cheap {belief.latency_ms:.1f} ms",
          (8, 38), scale=0.42)
    for i in range(n):
        center = (bounds[i] + bounds[i + 1]) // 2
        score = float(belief.scores[i]) if belief.valid[i] else np.nan
        color = score_color(score)
        _text(canvas, f"S{i}", (center, height + 17), center=True, scale=0.38)
        _text(canvas, f"{score:.2f}" if np.isfinite(score) else "?",
              (center, height + 40), color, scale=0.55, center=True)
        tau = values[f"tau_sector_{i}"]
        _text(canvas, f"{tau:.1f}s" if np.isfinite(tau) else "UNKNOWN",
              (center, height + 61), color, scale=0.31, center=True)
        _text(canvas, f"pts {int(values[f'count_sector_{i}'])}",
              (center, height + 80), scale=0.32, center=True)
    _text(canvas, "Relative score: 0 lower / 1 higher | ? unknown | TTC in seconds", (8, height + 104), scale=0.39)
    _text(canvas, "Optical flow needs motion + texture. Q or Esc closes the viewer.", (8, height + 123), scale=0.39)
    return canvas


def _recorded_odom(directory):
    states = {}
    for line in (Path(directory) / "packets.jsonl").read_text().splitlines():
        if line.strip():
            row = json.loads(line)
            states[row["seq"]] = (row.get("position"), row.get("velocity"),
                                  row.get("attitude_quat"), row.get("odom_valid", False))
    return states


def _window_closed():
    """Qt destroys its receiver on the X button; treat that as normal close."""
    key = cv2.waitKey(1) & 0xff
    if key in (ord('q'), 27):
        return True
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, help="Replay a recorded camera stream")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--output", type=Path, help="Last frame (.png) or video (.avi/.mp4)")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--max-wall-time", type=float, default=0, help="0 means until closed/end of recording")
    parser.add_argument("--startup-timeout", type=float, default=30)
    parser.add_argument("--frame-timeout", type=float, default=10)
    parser.add_argument("--fps", type=float, default=25, help="Replay display/video frame rate")
    args = parser.parse_args(argv)
    if min(args.fps, args.frame_timeout, args.startup_timeout) <= 0:
        parser.error("fps and timeouts must be positive")
    if args.max_frames < 0 or args.max_wall_time < 0:
        parser.error("frame/time limits cannot be negative")
    if args.output and args.output.suffix.lower() not in (".png", ".avi", ".mp4"):
        parser.error("--output must end in .png, .avi, or .mp4")
    if args.output and args.output.exists():
        parser.error(f"output already exists: {args.output}")
    source = vehicle = writer = ros = last_image = None
    frames = 0
    started = time.monotonic()
    states = {}
    try:
        if args.recording:
            source = LogFrameSource(str(args.recording))
            states = _recorded_odom(args.recording)
        else:
            import rclpy
            from .frame_source import SimFrameSource
            from .platform import MavrosVehicle
            ros = rclpy
            ros.init()
            source = SimFrameSource(heartbeat_timeout_s=args.startup_timeout,
                                    startup_timeout_s=args.startup_timeout)
            # This object publishes nothing until send(), which monitoring
            # never calls. Reuse its time-aligned odometry buffer.
            vehicle = MavrosVehicle(node_name="cheap_viewer_telemetry")
        source.wait_for_intrinsics(timeout_s=args.startup_timeout)
        stage = CheapStage(source.intrinsics)
        if not args.headless:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, 960, int((stage.height + PANEL_HEIGHT) * 960 / stage.width))
        previous = None
        last_received = time.monotonic()
        mode = "REPLAY - recorded camera" if args.recording else "LIVE MONITOR - no flight commands"
        print(mode, flush=True)
        while ros is None or ros.ok():
            tick = time.monotonic()
            if args.max_wall_time and tick - started >= args.max_wall_time:
                break
            packet = source.read()
            if packet is None:
                if args.recording:
                    break
                if tick - last_received > args.frame_timeout:
                    raise TimeoutError("No new camera frames; check simulator and /camera/image bridge")
                if not args.headless and _window_closed():
                    break
                time.sleep(0.005)
                continue
            last_received = tick
            if previous is None:
                previous = packet
                continue
            odom = states.get(packet.seq, (None, None, None, False)) if args.recording else vehicle.state_at(packet.t_capture)
            belief, features = stage.infer(packet, previous, odom)
            last_image = draw_overlay(packet.image, belief, features, stage.last_tracks, mode)
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                if args.output.suffix.lower() != ".png":
                    if writer is None:
                        codec = "MJPG" if args.output.suffix.lower() == ".avi" else "mp4v"
                        writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*codec), args.fps,
                                                 (last_image.shape[1], last_image.shape[0]))
                        if not writer.isOpened():
                            raise RuntimeError(f"Could not open video output {args.output}")
                    writer.write(last_image)
            frames += 1
            if frames == 1 or frames % 100 == 0:
                print(f"frame={frames} valid={int(belief.valid.sum())}/{stage.n_sectors} cheap_ms={belief.latency_ms:.2f}", flush=True)
            if not args.headless:
                cv2.imshow(WINDOW, last_image)
                if _window_closed():
                    break
            previous = packet
            if args.max_frames and frames >= args.max_frames:
                break
            if args.recording and not args.headless:
                time.sleep(max(0.0, 1.0 / args.fps - (time.monotonic() - tick)))
        if frames == 0:
            raise RuntimeError("No frame pair processed; at least two camera frames are required")
        if args.output and args.output.suffix.lower() == ".png":
            if not cv2.imwrite(str(args.output), last_image):
                raise RuntimeError(f"Could not save {args.output}")
        print(f"Viewer finished: {frames} frames" + (f"; saved {args.output}" if args.output else ""), flush=True)
        return 0
    finally:
        if writer is not None:
            writer.release()
        if vehicle is not None:
            vehicle.close()
        if source is not None:
            source.close()
        if not args.headless:
            cv2.destroyAllWindows()
        if ros is not None and ros.ok():
            ros.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
