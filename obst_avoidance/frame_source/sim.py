import threading
import time
from collections import deque
from typing import Optional

import numpy as np
from cv_bridge import CvBridge
from pymavlink import mavutil
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

from .base import FrameSource
from .packet import FramePacket


class SimFrameSource(FrameSource, Node):
    """FrameSource backed by the Gazebo sim: image from the ROS2-bridged
    camera topic, gyro streamed directly from ArduPilot SITL over raw
    MAVLink (bypassing mavros, per the Step D verification).

    All sim-specific plumbing lives here -- callers only see FrameSource.

    Timestamp bases (see Step A/D verification):
      - Image stamps come from the ROS Image message header, which is
        sim-time (ROS /clock, driven by Gazebo).
      - MAVLink ATTITUDE.time_boot_ms is the flight controller's own
        clock (lockstep-synced to sim time under ArduPilot SITL's
        synthetic_clock, but a DIFFERENT epoch/zero reference). It is
        converted to the same sim-time base via a ROLLING offset estimate
        (see _mav_loop): the minimum (t_sim - t_mav) observed over a
        sliding ~5s window, updated on every ATTITUDE message.

        History, because this went through two wrong designs before this
        one: a one-time startup calibration (averaging several samples)
        was tried first and produced deltas up to ~500ms. Switching the
        average to a minimum (the standard NTP one-sided-latency trick)
        was tried next and still produced a calibration ~240ms off from
        the true offset measured moments later -- root-caused to
        get_clock().now() being stale by however long the *previous*
        recv_match() blocked, because nothing was spinning this node
        between calibration samples except the calibration loop itself.
        A rolling per-message estimate computed from a *continuously
        spun* clock (see the dedicated spin thread below) doesn't have
        that problem -- get_clock().now() is always fresh to within the
        spin thread's own poll interval (a few ms), so a minimum-filtered
        rolling window now actually filters transport latency instead of
        picking out the most stale sample.

    Gyro frame: body-frame NED (raw MAVLink ATTITUDE convention) --
    rollspeed/pitchspeed/yawspeed, rad/s. This is NOT the ENU convention
    mavros/ROS topics use elsewhere in this package (see Step D: a +ENU
    commanded yaw rate produces a -NED yawspeed here).

    This gyro is the AIRFRAME's angular rate, not the camera's -- using it
    to de-rotate frames is only valid if the camera rotates 1:1 with the
    airframe. Step B confirmed the gimbal in this world is RIGID, not
    stabilizing (joint angles measured constant under a real ~59deg
    commanded airframe yaw), so that assumption holds here. It would NOT
    hold against a stabilizing gimbal -- that would need the gimbal's own
    joint-state rates composed with (or substituted for) this gyro.

    Gyro matching is LINEAR INTERPOLATION between the two buffered
    samples bracketing t_capture, not nearest-neighbor -- see
    _interpolated_gyro. If t_capture isn't bracketed (buffer empty, or
    t_capture is beyond the newest sample), falls back to nearest-sample
    and sets gyro_valid=False if that nearest sample is further than
    gyro_staleness_s away. gyro is still populated (zeros, if the buffer
    is empty) in the invalid case -- callers must check gyro_valid rather
    than treating a zero rate as "not rotating", which is a
    plausible-looking value that silently breaks de-rotation and
    everything downstream of it.

    gyro_staleness_s defaults to 0.05s (50ms). An earlier version of this
    class raised this to 0.25s after finding 50ms flagged ~100% of frames
    invalid -- that was masking a real, fixable sync problem rather than
    solving it (see the offset-estimate history above and the freshness
    note below), so it's back at 50ms now that the underlying sync is
    actually fixed rather than tolerated.

    Image color order is pinned to rgb8 explicitly (not left as
    "passthrough", which would silently hand back whatever encoding the
    publisher happens to use). If a consumer needs BGR (e.g. most OpenCV
    display/IO calls), convert explicitly -- this class does not.

    Uses its own private SingleThreadedExecutor rather than the module-
    level rclpy.spin_once(self, ...) convenience call, to keep its
    spinning fully isolated from whatever executor a caller uses for its
    own nodes (good hygiene, though see the note below for what the
    actual Step F bug turned out to be).

    Freshness / continuous spinning: a single rclpy spin_once() call
    processes at most ONE ready callback, and only advances this node's
    cached view of sim time (get_clock().now()) if a /clock message
    happens to be the thing it processed. Calling spin_once() only from
    read() (as an earlier version did) meant: (a) if a caller's own loop
    stalled while several images arrived, read() would hand out the
    OLDEST queued image, not the latest -- fixed by capping the image
    subscription's QoS to depth=1 (KEEP_LAST) so old frames are discarded
    at the transport level; and (b) get_clock().now() would go stale for
    however long it had been since read() was last called, which is
    exactly what broke the one-time offset calibration (see history
    above). Both problems share one root cause: nothing was spinning
    this node continuously. Fixed by moving all spinning to a dedicated
    daemon thread (_spin_loop), started in __init__ BEFORE any MAVLink
    connection or calibration work happens. read() and
    wait_for_intrinsics() no longer spin at all -- they only read state
    the spin thread keeps up to date in the background. Only one thread
    may ever call spin_once() on a given SingleThreadedExecutor; once
    _spin_loop owns that, nothing else in this class does.

    Default mavlink_url is port 14551, NOT 14550. This matters: 14550 is
    the port mavros binds to (see the launch file's fcu_url). SITL's
    mavproxy relay sends telemetry to *both* 14550 and 14551 (two
    separate --out targets), so 14551 is free for exclusive use here.
    Using 14550 was tried first and broke mavros: once this class's
    long-lived pymavlink socket also bound to 14550, mavros's own /state
    updates stopped arriving entirely after the first message (almost
    certainly a UDP port-sharing/flow-hashing conflict -- e.g. SO_REUSEPORT
    sticking mavproxy's single outbound flow to whichever of the two
    sockets "won" it, starving the other). This was hard to track down
    because the symptom looked identical to an executor-starvation bug
    (a plausible-looking but ultimately wrong first theory, isolated and
    ruled out by testing with fully private executors on both sides and
    seeing the same failure) -- the actual fix was simply not sharing the
    port. 14550 remains available as an explicit override if the caller
    knows what they're doing.
    """

    def __init__(
        self,
        image_topic: str = "/camera/image",
        camera_info_topic: str = "/camera/camera_info",
        mavlink_url: str = "udp:127.0.0.1:14551",
        gyro_hz: float = 50.0,
        gyro_buffer_size: int = 100,
        gyro_staleness_s: float = 0.05,
        offset_window_s: float = 5.0,
        heartbeat_timeout_s: float = 10.0,
        startup_timeout_s: float = 5.0,
    ):
        if gyro_hz <= 0 or heartbeat_timeout_s <= 0 or startup_timeout_s <= 0:
            raise ValueError("gyro rate and startup timeouts must be positive")
        Node.__init__(
            self,
            "sim_frame_source",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._bridge = CvBridge()
        self._gyro_staleness_s = gyro_staleness_s
        self._offset_window_s = offset_window_s
        self._gyro_hz = gyro_hz

        # -- image state (written by ROS callback, read by read()) --
        self._image_lock = threading.Lock()
        self._latest_image = None  # (t_capture, np.ndarray, seq)
        self._image_seq = 0
        self._consumed_seq = -1
        self._dropped = 0
        # depth=1 (KEEP_LAST default history for a plain int) -- see
        # "Freshness / continuous spinning" in the class docstring.
        self.create_subscription(Image, image_topic, self._on_image, 1)

        # -- intrinsics (written once by ROS callback) --
        self._camera_info_lock = threading.Lock()
        self._camera_info: Optional[CameraInfo] = None
        self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, 1
        )

        # -- gyro / offset state (written by MAVLink recv thread) --
        self._gyro_lock = threading.Lock()
        self._gyro_deque: deque = deque(maxlen=gyro_buffer_size)
        self._offset_lock = threading.Lock()
        self._offset_window: deque = deque()  # (wall_time, offset_sample)
        self._time_offset: Optional[float] = None
        self.last_sync_delta_s: Optional[float] = None  # diagnostic only

        # Own all spin_once() calls on self._executor from here on -- see
        # "Freshness / continuous spinning". Started before the MAVLink
        # connection/calibration so get_clock().now() is never read stale.
        self._spin_stop = threading.Event()
        self._mav_stop = threading.Event()
        self._mav = None
        self._mav_thread = None
        self._closed = False
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        try:
            self._connect_mavlink(mavlink_url, heartbeat_timeout_s, startup_timeout_s)
        except BaseException:
            self.close()
            raise

    def _connect_mavlink(self, mavlink_url, heartbeat_timeout_s, startup_timeout_s):
        self._mav = mavutil.mavlink_connection(mavlink_url)
        if self._mav.wait_heartbeat(timeout=heartbeat_timeout_s) is None:
            raise TimeoutError(f"no SITL heartbeat at {mavlink_url} within {heartbeat_timeout_s}s")

        # Step 4 finding: MAV_CMD_SET_MESSAGE_INTERVAL alone (below) was
        # observed capped around ~19Hz despite requesting 50Hz, until the
        # underlying stream-group rate was also raised. That group rate
        # is MAV1_EXTRA1 (the modern name -- SR0_EXTRA1 from ArduPilot
        # docs/tutorials is stale, doesn't exist in this build, and
        # PARAM_REQUEST_READ for it just times out silently). It defaults
        # to 4Hz. Raising both together reached ~38Hz at RTF~0.88 (close
        # to the 50Hz request once scaled by RTF). No ack wait here,
        # matching the existing SET_MESSAGE_INTERVAL call below -- best
        # effort, not required for correctness (a slower stream still
        # works, just with wider interpolation brackets).
        self._mav.mav.param_set_send(
            self._mav.target_system, self._mav.target_component,
            b"MAV1_EXTRA1", float(self._gyro_hz),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            int(1e6 / self._gyro_hz),
            0, 0, 0, 0, 0,
        )
        self._assert_initial_offset(timeout_s=startup_timeout_s)

        self._mav_thread = threading.Thread(target=self._mav_loop, daemon=True)
        self._mav_thread.start()

    def _spin_loop(self):
        while not self._spin_stop.is_set():
            self._executor.spin_once(timeout_sec=0.01)

    def _assert_initial_offset(self, timeout_s: float = 5.0):
        """Blocks (by polling, NOT spinning -- the dedicated spin thread
        owns that) for the sim clock to come up, then takes one ATTITUDE
        sample to seed the rolling offset window and sanity-check it.
        ArduPilot SITL and Gazebo share a sim-time epoch under
        synthetic_clock, so the true offset should be ~0; a large value
        here means something is wrong upstream of this class (SITL not
        actually lockstepped, wrong clock topic, etc), not something this
        class can fix by filtering harder."""
        deadline = time.monotonic() + timeout_s
        while self.get_clock().now().nanoseconds == 0:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"no nonzero simulation /clock within {timeout_s}s")
            time.sleep(0.02)

        remaining = max(0.0, deadline - time.monotonic())
        msg = self._mav.recv_match(type="ATTITUDE", blocking=True, timeout=remaining)
        if msg is None:
            raise RuntimeError(
                f"no ATTITUDE message received within {timeout_s}s at startup"
            )
        t_sim = self.get_clock().now().nanoseconds * 1e-9
        t_mav = msg.time_boot_ms / 1000.0
        offset = t_sim - t_mav
        now_wall = time.time()
        with self._offset_lock:
            self._offset_window.append((now_wall, offset))
            self._time_offset = offset

        if abs(offset) > 0.05:
            self.get_logger().warn(
                f"startup time offset {offset*1000:+.1f}ms exceeds the "
                f"expected ~0 (|offset|<50ms) -- ArduPilot SITL and Gazebo "
                f"are supposed to share a sim-time epoch under "
                f"synthetic_clock. Something upstream may be wrong."
            )
        else:
            self.get_logger().info(
                f"startup time offset {offset*1000:+.1f}ms (within 50ms, as expected)"
            )

    # ---- image side ---------------------------------------------------

    def _on_image(self, msg: Image):
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        t_capture = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._image_lock:
            if self._latest_image is not None and self._consumed_seq != self._image_seq:
                self._dropped += 1
            self._image_seq += 1
            self._latest_image = (t_capture, img, self._image_seq)

    def _on_camera_info(self, msg: CameraInfo):
        with self._camera_info_lock:
            self._camera_info = msg

    # ---- gyro side ------------------------------------------------------

    def _mav_loop(self):
        # Empirically, the SET_MESSAGE_INTERVAL request from __init__
        # doesn't stick: measured rate booms to ~28Hz right after it's
        # sent, then decays to ~3Hz within 5-10s and stays there. Most
        # likely cause: mavros shares the same underlying FC link (via
        # mavproxy's single master connection) and periodically re-asserts
        # its own (lower) stream-rate config, which -- since ArduPilot
        # tracks message-interval state per (message, channel), not per
        # remote client -- silently overrides whatever we asked for.
        # Fighting that from the mavros/launch side wasn't pursued (out of
        # this class's scope); simplest fix that stays self-contained is
        # to keep re-claiming the rate periodically instead of once.
        last_rate_refresh = 0.0
        rate_refresh_period_s = 0.5

        while not self._mav_stop.is_set():
            now_wall = time.time()
            if now_wall - last_rate_refresh > rate_refresh_period_s:
                self._mav.mav.command_long_send(
                    self._mav.target_system, self._mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                    int(1e6 / self._gyro_hz),
                    0, 0, 0, 0, 0,
                )
                last_rate_refresh = now_wall

            msg = self._mav.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            t_mav = msg.time_boot_ms / 1000.0
            # Safe to read every message now (not just at calibration
            # time): the dedicated spin thread keeps this continuously
            # fresh, so it's no longer biased by how long THIS recv_match
            # call happened to block.
            t_sim = self.get_clock().now().nanoseconds * 1e-9
            offset_sample = t_sim - t_mav
            now_wall = time.time()
            gyro = np.array(
                [msg.rollspeed, msg.pitchspeed, msg.yawspeed], dtype=np.float64
            )
            with self._offset_lock:
                self._offset_window.append((now_wall, offset_sample))
                while (self._offset_window and
                       now_wall - self._offset_window[0][0] > self._offset_window_s):
                    self._offset_window.popleft()
                # min-filter: correct now that transport latency (one-
                # sided, never negative) is the only remaining error --
                # see class docstring history.
                current_offset = min(s for _, s in self._offset_window)
                self._time_offset = current_offset
            with self._gyro_lock:
                self._gyro_deque.append((t_mav + current_offset, gyro))

    def _interpolated_gyro(self, t_capture: float):
        """Returns (gyro, valid). Linearly interpolates between the two
        buffered samples bracketing t_capture when possible (valid=True
        unconditionally in that case). Falls back to nearest-sample if no
        bracket exists (t_capture outside the buffered range), with
        valid=False if that nearest sample is further than
        gyro_staleness_s away."""
        with self._gyro_lock:
            if not self._gyro_deque:
                self.last_sync_delta_s = None
                return None, False
            samples = list(self._gyro_deque)  # append-only -> time-ordered

        lo = None
        hi = None
        for t, g in samples:
            if t <= t_capture:
                lo = (t, g)
            if t >= t_capture and hi is None:
                hi = (t, g)

        if lo is not None and hi is not None and hi[0] > lo[0]:
            t0, g0 = lo
            t1, g1 = hi
            frac = (t_capture - t0) / (t1 - t0)
            gyro = g0 + frac * (g1 - g0)
            self.last_sync_delta_s = t1 - t0  # bracket width, diagnostic
            return gyro, True
        if lo is not None and hi is not None and hi[0] == lo[0]:
            self.last_sync_delta_s = 0.0
            return lo[1], True

        # No bracket: t_capture is outside the buffered range (usually
        # newer than the newest sample). Fall back to nearest.
        nearest_t, nearest_g = min(samples, key=lambda item: abs(item[0] - t_capture))
        delta = abs(t_capture - nearest_t)
        self.last_sync_delta_s = delta
        return nearest_g, delta <= self._gyro_staleness_s

    # ---- FrameSource interface -----------------------------------------

    def read(self) -> Optional[FramePacket]:
        # No spinning here -- the dedicated spin thread keeps
        # _latest_image (and everything else) up to date continuously.
        # See "Freshness / continuous spinning" in the class docstring.
        with self._image_lock:
            if self._latest_image is None:
                return None
            t_capture, img, seq = self._latest_image
            if seq == self._consumed_seq:
                return None
            self._consumed_seq = seq

        gyro, gyro_valid = self._interpolated_gyro(t_capture)
        if gyro is None:
            gyro = np.zeros(3, dtype=np.float64)

        return FramePacket(
            image=img, gyro=gyro, t_capture=t_capture, seq=seq,
            gyro_valid=gyro_valid,
        )

    @property
    def dropped_frames(self) -> int:
        with self._image_lock:
            return self._dropped

    @property
    def intrinsics(self) -> Optional[dict]:
        """{fx, fy, cx, cy, width, height} decoded from camera_info's K
        matrix, or None if camera_info hasn't arrived yet. This is the
        quick-access form for geometry code. For the full raw
        camera_info (K/D/distortion_model, e.g. for persisting a
        calibration file), use get_intrinsics()/wait_for_intrinsics()."""
        with self._camera_info_lock:
            msg = self._camera_info
        if msg is None:
            return None
        k = msg.k
        return {
            "fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5],
            "width": msg.width, "height": msg.height,
        }

    def get_intrinsics(self) -> Optional[dict]:
        """Latest camera_info in full (K/D/distortion_model), or None if
        none has arrived yet. Static for the session (a camera doesn't
        re-calibrate mid-flight), so callers needing this typically want
        wait_for_intrinsics() once at startup rather than polling this
        per-frame. See also the intrinsics property for the decoded
        fx/fy/cx/cy quick-access form."""
        with self._camera_info_lock:
            msg = self._camera_info
        if msg is None:
            return None
        return {
            "width": msg.width,
            "height": msg.height,
            "k": list(msg.k),
            "d": list(msg.d),
            "distortion_model": msg.distortion_model,
        }

    def wait_for_intrinsics(self, timeout_s: float = 5.0) -> dict:
        # Polls, does not spin -- the dedicated spin thread owns
        # spin_once() (see class docstring).
        t_end = time.time() + timeout_s
        while self.get_intrinsics() is None:
            if time.time() > t_end:
                raise TimeoutError(
                    f"no camera_info received within {timeout_s}s"
                )
            time.sleep(0.05)
        return self.get_intrinsics()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._mav_stop.set()
        if self._mav_thread is not None:
            self._mav_thread.join(timeout=2.0)
        self._spin_stop.set()
        self._spin_thread.join(timeout=2.0)
        if self._mav is not None:
            self._mav.close()
        self._executor.remove_node(self)
        self._executor.shutdown()
        self.destroy_node()
