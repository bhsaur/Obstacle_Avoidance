#!/usr/bin/env python3
"""Fly a scripted pass while recording every FramePacket to disk.

Three modes:
  forward           GUIDED -> arm -> takeoff -> fly forward at a fixed
                    speed for N seconds (default; unchanged from before).
  yaw_only          GUIDED -> arm -> takeoff -> hold position, command a
                    pure yaw rate for N seconds. For validating
                    de-rotation: after subtracting the rotational flow
                    field, residual flow should approach zero (see
                    tools/flow_explore.py).
  roll_pitch_wiggle GUIDED -> arm -> takeoff -> hold altitude, no net
                    translation. First half of --duration: alternating
                    +y/-y lateral velocity (period --wiggle-period) to
                    induce roll rate. Second half: alternating +x/-x
                    fore/aft velocity to induce pitch rate. Exists
                    because yaw_only alone can't discriminate
                    wx_cam/wz_cam in flow_explore.py's 48-candidate axis
                    sweep -- both stay near-zero throughout a pure-yaw
                    maneuver. See Step U in README.

Each packets.jsonl row now also has: cmd_linear_x, cmd_linear_y,
cmd_angular_z (the commanded velocity at capture time), position,
velocity, attitude_quat (from /odometry -- needed later for ground-truth
collision labelling).
A metadata.json (label, mode, altitude, speed/yaw_rate/wiggle params,
duration, timestamp) is written alongside intrinsics.json.

This is a data-collection tool only -- no perception, control, or gating.
Flight control here is deliberately the simplest thing that produces a
scripted pass: no obstacle awareness at all.

Usage:
  ros2 run obst_avoidance record_pass --out-dir ~/ardu_ws/recordings/run1
  ros2 run obst_avoidance record_pass --out-dir ~/recordings/yaw1 \\
      --mode yaw_only --yaw-rate 0.5 --duration 10 --label yaw_only
"""
import argparse
import json
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import cv2
import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from obst_avoidance.frame_source import SimFrameSource


class FlightControl(Node):
    """Minimal GUIDED-mode arm/takeoff/fly control, separate from
    SimFrameSource so capture logic stays uninvolved with flight control.

    Uses its own private SingleThreadedExecutor rather than the
    module-level rclpy.spin_once()/spin_until_future_complete()
    convenience functions -- those spin a SHARED GLOBAL executor, and
    coexisting with SimFrameSource (which also spins itself) through that
    shared executor caused this node to stop receiving /state updates
    entirely (see SimFrameSource's docstring for the full story). A
    private executor avoids sharing anything with other nodes.
    """

    def __init__(self, odom_buffer_size: int = 200, odom_staleness_s: float = 0.05):
        super().__init__(
            "record_pass_control",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._set_mode_cli = self.create_client(SetMode, "/set_mode")
        self._arm_cli = self.create_client(CommandBool, "/cmd/arming")
        self._takeoff_cli = self.create_client(CommandTOL, "/cmd/takeoff")
        self._vel_pub = self.create_publisher(
            TwistStamped, "/setpoint_velocity/cmd_vel", 10
        )
        self._z = None
        self._mode = None
        self._cmd_linear_x = 0.0
        self._cmd_linear_y = 0.0
        self._cmd_angular_z = 0.0
        # Buffered (t_odom, position, velocity, attitude_quat) samples, for
        # interpolating to a frame's t_capture -- mirrors SimFrameSource's
        # gyro deque exactly (single spin thread there vs. single spin loop
        # here, but same "no cross-thread lock needed" reasoning applies:
        # _on_odom and every reader run on this node's own executor, driven
        # only from spin_once() calls made by this class's own methods).
        # See Step V in README: previously this class logged only the most
        # recently arrived odometry sample against packet.t_capture, with
        # no odometry timestamp recorded at all, so the true alignment was
        # unmeasurable.
        self._odom_staleness_s = odom_staleness_s
        self._odom_deque: deque = deque(maxlen=odom_buffer_size)
        self.last_odom_sync_delta_s = None  # diagnostic only, see state_at()
        self.create_subscription(Odometry, "/odometry", self._on_odom, 10)
        self.create_subscription(State, "/state", self._on_state, 10)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._z = p.z
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        t_odom = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._odom_deque.append(
            (t_odom, (p.x, p.y, p.z), (v.x, v.y, v.z), (q.x, q.y, q.z, q.w))
        )

    def _on_state(self, msg: State):
        self._mode = msg.mode

    @staticmethod
    def _lerp3(a, b, frac):
        return tuple(a[i] + frac * (b[i] - a[i]) for i in range(3))

    @staticmethod
    def _nlerp4(a, b, frac):
        """Linear interpolation + renormalize -- a standard cheap
        approximation to slerp, valid when the inter-sample rotation is
        small (holds here: /odometry's rate is high relative to the
        rotation accrued between consecutive samples, same assumption
        SimFrameSource's gyro interpolation makes about its own buffer)."""
        raw = tuple(a[i] + frac * (b[i] - a[i]) for i in range(4))
        norm = sum(c * c for c in raw) ** 0.5
        if norm == 0.0:
            return a
        return tuple(c / norm for c in raw)

    def state_at(self, t_capture: float):
        """Returns (position, velocity, attitude_quat, odom_valid),
        interpolated to t_capture from the buffered /odometry samples --
        same bracket/nearest-fallback/staleness-flag contract as
        SimFrameSource._interpolated_gyro (see that method's docstring
        for the full reasoning). odom_valid=False must not be read as
        'no motion' -- the returned values are still populated from the
        nearest sample, just outside odom_staleness_s. Sets
        last_odom_sync_delta_s as a diagnostic: bracket width when
        interpolated, or the actual delta to the nearest sample in the
        fallback case (mirrors SimFrameSource.last_sync_delta_s)."""
        if not self._odom_deque:
            self.last_odom_sync_delta_s = None
            return None, None, None, False
        samples = list(self._odom_deque)  # append-only -> time-ordered

        lo = None
        hi = None
        for t, p, v, q in samples:
            if t <= t_capture:
                lo = (t, p, v, q)
            if t >= t_capture and hi is None:
                hi = (t, p, v, q)

        if lo is not None and hi is not None and hi[0] > lo[0]:
            t0, p0, v0, q0 = lo
            t1, p1, v1, q1 = hi
            frac = (t_capture - t0) / (t1 - t0)
            self.last_odom_sync_delta_s = t1 - t0
            return (
                self._lerp3(p0, p1, frac),
                self._lerp3(v0, v1, frac),
                self._nlerp4(q0, q1, frac),
                True,
            )
        if lo is not None and hi is not None and hi[0] == lo[0]:
            self.last_odom_sync_delta_s = 0.0
            return lo[1], lo[2], lo[3], True

        nearest_t, nearest_p, nearest_v, nearest_q = min(
            samples, key=lambda item: abs(item[0] - t_capture)
        )
        delta = abs(t_capture - nearest_t)
        self.last_odom_sync_delta_s = delta
        return nearest_p, nearest_v, nearest_q, delta <= self._odom_staleness_s

    def spin_once(self, timeout_sec: float):
        self._executor.spin_once(timeout_sec=timeout_sec)

    def close(self):
        self._executor.remove_node(self)
        self._executor.shutdown()
        self.destroy_node()

    def _wait_for_service(self, client, name):
        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f"waiting for {name} ...")

    def guided_arm_takeoff(self, altitude: float, timeout_s: float = 30.0):
        self._wait_for_service(self._set_mode_cli, "/set_mode")
        fut = self._set_mode_cli.call_async(SetMode.Request(custom_mode="GUIDED"))
        self._executor.spin_until_future_complete(fut, timeout_sec=8.0)

        # mode_sent=True from the service call only means the request was
        # accepted for sending -- it does NOT mean the FCU has switched
        # yet. Arming while still in a mode that forbids it (e.g. leftover
        # LAND from a previous run) is rejected, so wait for /state.mode
        # to actually confirm GUIDED before arming.
        # /state publish rate is throttled by sim RTF (established in
        # Step A/D -- can be well under 1 Hz effective), so give this a
        # generous window rather than the service-call timeouts above.
        t_mode_end = time.time() + 20.0
        while time.time() < t_mode_end and self._mode != "GUIDED":
            self.spin_once(timeout_sec=0.2)
        if self._mode != "GUIDED":
            raise RuntimeError(f"mode did not become GUIDED (last={self._mode!r})")

        self._wait_for_service(self._arm_cli, "/cmd/arming")
        fut = self._arm_cli.call_async(CommandBool.Request(value=True))
        self._executor.spin_until_future_complete(fut, timeout_sec=8.0)
        if not (fut.result() and fut.result().success):
            raise RuntimeError(f"arm failed: {fut.result()}")

        self._wait_for_service(self._takeoff_cli, "/cmd/takeoff")
        fut = self._takeoff_cli.call_async(
            CommandTOL.Request(min_pitch=0.0, yaw=0.0, altitude=altitude)
        )
        self._executor.spin_until_future_complete(fut, timeout_sec=8.0)
        if not (fut.result() and fut.result().success):
            raise RuntimeError(f"takeoff failed: {fut.result()}")

        t_end = time.time() + timeout_s
        while time.time() < t_end:
            self.spin_once(timeout_sec=0.2)
            if self._z is not None and abs(self._z - altitude) < 0.5:
                return
        raise RuntimeError(
            f"did not reach altitude {altitude} within {timeout_s}s "
            f"(last z={self._z})"
        )

    def fly_forward(self, speed: float, duration_s: float, on_tick=None):
        t_end = time.time() + duration_s
        twist = TwistStamped()
        twist.twist.linear.x = speed
        self._cmd_linear_x = speed
        self._cmd_linear_y = 0.0
        self._cmd_angular_z = 0.0
        while time.time() < t_end:
            twist.header.stamp = self.get_clock().now().to_msg()
            self._vel_pub.publish(twist)
            self.spin_once(timeout_sec=0.02)
            if on_tick is not None:
                on_tick()
        twist.twist.linear.x = 0.0
        self._cmd_linear_x = 0.0
        for _ in range(10):
            twist.header.stamp = self.get_clock().now().to_msg()
            self._vel_pub.publish(twist)
            self.spin_once(timeout_sec=0.05)

    def hold_yaw_rate(self, yaw_rate: float, duration_s: float, on_tick=None):
        """Zero translation, constant commanded yaw rate -- for
        validating de-rotation (see tools/flow_explore.py)."""
        t_end = time.time() + duration_s
        twist = TwistStamped()
        twist.twist.angular.z = yaw_rate
        self._cmd_linear_x = 0.0
        self._cmd_linear_y = 0.0
        self._cmd_angular_z = yaw_rate
        while time.time() < t_end:
            twist.header.stamp = self.get_clock().now().to_msg()
            self._vel_pub.publish(twist)
            self.spin_once(timeout_sec=0.02)
            if on_tick is not None:
                on_tick()
        twist.twist.angular.z = 0.0
        self._cmd_angular_z = 0.0
        for _ in range(10):
            twist.header.stamp = self.get_clock().now().to_msg()
            self._vel_pub.publish(twist)
            self.spin_once(timeout_sec=0.05)

    def hold_roll_pitch_wiggle(
        self, speed: float, period_s: float, duration_s: float, on_tick=None
    ):
        """First half of duration_s: alternating +y/-y lateral velocity
        (half-period = period_s/2) to induce roll rate. Second half:
        alternating +x/-x fore/aft velocity to induce pitch rate. Equal
        time at each sign in both phases, so net displacement is
        approximately zero by symmetry -- this is a rate-inducing
        maneuver, not a translation. See Step U in README for why."""
        twist = TwistStamped()

        def _wiggle_phase(set_axis, phase_duration_s: float):
            t_end = time.time() + phase_duration_s
            sign = 1.0
            t_next_flip = time.time() + period_s / 2.0
            while time.time() < t_end:
                now = time.time()
                if now >= t_next_flip:
                    sign = -sign
                    t_next_flip = now + period_s / 2.0
                set_axis(sign * speed)
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.twist.linear.x = self._cmd_linear_x
                twist.twist.linear.y = self._cmd_linear_y
                self._vel_pub.publish(twist)
                self.spin_once(timeout_sec=0.02)
                if on_tick is not None:
                    on_tick()

        def _set_y(v):
            self._cmd_linear_y = v

        def _set_x(v):
            self._cmd_linear_x = v

        half = duration_s / 2.0
        _wiggle_phase(_set_y, half)
        self._cmd_linear_y = 0.0
        _wiggle_phase(_set_x, half)
        self._cmd_linear_x = 0.0

        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        for _ in range(10):
            twist.header.stamp = self.get_clock().now().to_msg()
            self._vel_pub.publish(twist)
            self.spin_once(timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", type=str, required=True)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--speed", type=float, default=1.0, help="forward m/s (mode=forward)")
    parser.add_argument("--yaw-rate", type=float, default=0.5, help="rad/s (mode=yaw_only)")
    parser.add_argument("--wiggle-speed", type=float, default=1.5,
                         help="m/s magnitude (mode=roll_pitch_wiggle)")
    parser.add_argument("--wiggle-period", type=float, default=1.0,
                         help="seconds, full +/- cycle (mode=roll_pitch_wiggle)")
    parser.add_argument("--duration", type=float, default=15.0, help="seconds")
    parser.add_argument("--mode", type=str, default="forward",
                         choices=["forward", "yaw_only", "roll_pitch_wiggle"])
    parser.add_argument("--label", type=str, default="", help="written into metadata.json")
    args = parser.parse_args()

    out_dir = Path(args.out_dir).expanduser()
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    control = FlightControl()
    source = SimFrameSource()

    intrinsics = source.wait_for_intrinsics()
    (out_dir / "intrinsics.json").write_text(json.dumps(intrinsics, indent=2))

    metadata = {
        "label": args.label,
        "mode": args.mode,
        "altitude": args.altitude,
        "speed": args.speed if args.mode == "forward" else None,
        "yaw_rate": args.yaw_rate if args.mode == "yaw_only" else None,
        "wiggle_speed": args.wiggle_speed if args.mode == "roll_pitch_wiggle" else None,
        "wiggle_period": args.wiggle_period if args.mode == "roll_pitch_wiggle" else None,
        "duration": args.duration,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
    }
    (out_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))

    packets_path = out_dir / "packets.jsonl"
    saved_count = 0
    invalid_gyro_count = 0
    invalid_odom_count = 0
    odom_sync_deltas = []
    t_start = time.time()

    with open(packets_path, "w") as packets_f:

        def capture_tick():
            nonlocal saved_count, invalid_gyro_count, invalid_odom_count
            packet = source.read()
            if packet is None:
                return
            if not packet.gyro_valid:
                invalid_gyro_count += 1
                print(f"WARNING: frame {packet.seq} has no fresh gyro sample "
                      f"(gyro_valid=False) -- recorded anyway, flagged in "
                      f"packets.jsonl")
            position, velocity, attitude_quat, odom_valid = control.state_at(
                packet.t_capture
            )
            if control.last_odom_sync_delta_s is not None:
                odom_sync_deltas.append(control.last_odom_sync_delta_s)
            if not odom_valid:
                invalid_odom_count += 1
                print(f"WARNING: frame {packet.seq} has no fresh odometry sample "
                      f"(odom_valid=False) -- recorded anyway, flagged in "
                      f"packets.jsonl")
            frame_path = frames_dir / f"frame_{packet.seq:06d}.png"
            # packet.image is rgb8 (see SimFrameSource docstring);
            # cv2.imwrite expects BGR, so convert explicitly here rather
            # than writing raw RGB bytes into a file that looks fine but
            # has red/blue swapped.
            bgr = cv2.cvtColor(packet.image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(frame_path), bgr)
            packets_f.write(json.dumps({
                "seq": packet.seq,
                "t_capture": packet.t_capture,
                "gyro": packet.gyro.tolist(),
                "gyro_valid": packet.gyro_valid,
                "frame_file": frame_path.name,
                "cmd_linear_x": control._cmd_linear_x,
                "cmd_linear_y": control._cmd_linear_y,
                "cmd_angular_z": control._cmd_angular_z,
                "position": position,
                "velocity": velocity,
                "attitude_quat": attitude_quat,
                "odom_valid": odom_valid,
            }) + "\n")
            saved_count += 1

        control.guided_arm_takeoff(args.altitude)
        if args.mode == "yaw_only":
            control.hold_yaw_rate(args.yaw_rate, args.duration, on_tick=capture_tick)
        elif args.mode == "roll_pitch_wiggle":
            control.hold_roll_pitch_wiggle(
                args.wiggle_speed, args.wiggle_period, args.duration,
                on_tick=capture_tick,
            )
        else:
            control.fly_forward(args.speed, args.duration, on_tick=capture_tick)
        # Drain a few more ticks in case frames are still arriving.
        for _ in range(20):
            capture_tick()
            time.sleep(0.05)

    t_elapsed = time.time() - t_start
    if odom_sync_deltas:
        odom_delta_mean = sum(odom_sync_deltas) / len(odom_sync_deltas)
        odom_delta_max = max(odom_sync_deltas)
    else:
        odom_delta_mean = odom_delta_max = float("nan")
    print(f"saved {saved_count} frames over {t_elapsed:.1f}s wall-clock "
          f"to {out_dir} (dropped_frames={source.dropped_frames}, "
          f"invalid_gyro={invalid_gyro_count}, invalid_odom={invalid_odom_count}, "
          f"odom_sync_delta_s mean={odom_delta_mean:.4f} max={odom_delta_max:.4f})")

    source.close()
    control.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
