"""VehicleInterface -- what the orchestrator (Step AR) needs from "the
vehicle," kept deliberately minimal (send a command, read state) so any
concrete implementation (mavros/SITL today, a real FC later, an offline
stub for controller-only testing) satisfies it without the orchestrator
or SectorController ever needing to know which.

CRITICAL DESIGN CONSTRAINT -- setpoint publishing is decoupled from the
perception loop. ArduPilot GUIDED mode fails safe (drops out of GUIDED)
if velocity setpoints stop arriving within its own timeout. Perception
runs ~25Hz today; HeavyStage will stall the loop 30-45ms per call, worse
under the low real-time-factor this project has repeatedly measured
during evaluation (RTF as low as 0.15 -- see README's RTF history). A
single stall long enough to blow through mavros's setpoint timeout would
look exactly like a control bug and take a long time to trace -- this
project has already paid that exact tax once, for a different reason
(see frame_source/sim.py's "Freshness / continuous spinning" history:
tying a node's spin cadence to a caller's loop broke things in a way
that took real effort to root-cause).

The fix, reusing that same lesson: MavrosVehicle spins itself on a
PRIVATE, dedicated daemon thread, started in __init__ before anything
else touches this node -- identical architecture to SimFrameSource, for
the identical reason (nothing about this node's own liveness should
depend on what thread or loop cadence a caller uses). A ROS timer on
that node, firing at a FIXED PUBLISH_RATE_HZ independent of anything
else, republishes whatever ControlCommand was last given to send() --
send() only UPDATES the stored command, it never publishes directly.
After command_timeout_s without send(), the steady-clock timer publishes
zero translation and yaw until fresh commands arrive. This operational
watchdog is independent of capture-time controller hysteresis.
"""
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Protocol, runtime_checkable

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from ..control.types import ControlCommand

PUBLISH_RATE_HZ = 15.0  # fixed, independent of perception cadence -- see module docstring


@dataclass
class VehicleState:
    """Latest vehicle telemetry as reported by the platform -- a
    snapshot, NOT interpolated to any particular frame timestamp
    (compare record_pass.FlightControl.state_at(), which IS
    time-aligned to a packet's t_capture -- that alignment is a
    perception-adjacent concern for whoever wires the orchestrator's
    logging, not this minimal interface's job). Field set matches what
    record_pass.py already established as this project's standard
    telemetry (from /odometry and /state)."""

    connected: bool
    armed: bool
    mode: Optional[str]
    position: Optional[tuple]        # (x, y, z)
    velocity: Optional[tuple]        # (vx, vy, vz)
    attitude_quat: Optional[tuple]   # (x, y, z, w)


@runtime_checkable
class VehicleInterface(Protocol):
    def send(self, cmd: ControlCommand) -> None:
        """Updates the command this platform will act on. Does NOT
        publish it immediately or guarantee it reaches the vehicle
        before this call returns -- see MavrosVehicle's fixed-rate
        republishing, which owns actual delivery."""
        ...

    def state(self) -> VehicleState:
        """Latest known vehicle state."""
        ...


class MavrosVehicle(Node):
    """VehicleInterface backed by mavros -- GUIDED-mode ArduPilot SITL
    today, a real FC later if it speaks the same topics. Structurally
    satisfies VehicleInterface (send/state) via duck typing, per
    typing.Protocol -- no explicit inheritance needed or used.

    yaw_rate sign: published to /setpoint_velocity/cmd_vel's angular.z
    with NO conversion -- control/sector.py's documented ENU convention
    (positive=left) is exactly what record_pass.py already established
    mavros's setpoint_velocity plugin expects, so this class trusts the
    value it's given rather than re-deriving or flipping a sign.

    Velocity frame: MAVROS's setpoint_velocity.mav_frame must be
    LOCAL_NED, whose ROS input is world ENU. ControlCommand.fwd_vel is
    body-forward, so it is rotated by the latest vehicle yaw before
    publishing. With missing attitude, translation stays zero until a
    heading is available. The caller must verify the MAVROS frame before
    enabling autonomous commands.
    """

    def __init__(self, node_name: str = "mavros_vehicle", publish_rate_hz: float = PUBLISH_RATE_HZ,
                 odom_buffer_size: int = 200, odom_staleness_s: float = 0.05, command_timeout_s: float = 1.0):
        if not math.isfinite(command_timeout_s) or command_timeout_s <= 0:
            raise ValueError("command_timeout_s must be finite and positive")
        self._command_timeout_s = command_timeout_s
        self._last_command_wall = None
        self.command_expired = False
        super().__init__(node_name, parameter_overrides=[Parameter("use_sim_time", value=True)])
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

        self._vel_pub = self.create_publisher(TwistStamped, "/setpoint_velocity/cmd_vel", 10)
        self._set_mode_cli = self.create_client(SetMode, "/set_mode")
        self._arm_cli = self.create_client(CommandBool, "/cmd/arming")
        self._takeoff_cli = self.create_client(CommandTOL, "/cmd/takeoff")

        self._state_lock = threading.Lock()
        self._connected = False
        self._armed = False
        self._mode: Optional[str] = None
        self._position: Optional[tuple] = None
        self._velocity: Optional[tuple] = None
        self._attitude_quat: Optional[tuple] = None

        # Buffered (t, position, velocity, attitude_quat) samples for
        # state_at() -- mirrors record_pass.FlightControl.state_at()'s
        # interpolation logic exactly (same bracket/nearest-fallback/
        # staleness contract, same NLERP for attitude), intentionally
        # NOT shared code with that class: FlightControl has no
        # dedicated spin thread (the caller drives its spin_once()
        # cadence directly), so its buffer needs no lock; this class
        # does have one (see module docstring), so its buffer does.
        # CheapStage.infer()'s `odom` parameter is documented as
        # expecting exactly this shape -- this is what makes MavrosVehicle
        # usable as its live source. Known, accepted duplication: a fix
        # to one implementation's interpolation math should be mirrored
        # to the other.
        self._odom_lock = threading.Lock()
        self._odom_deque: deque = deque(maxlen=odom_buffer_size)
        self._odom_staleness_s = odom_staleness_s
        self.last_odom_sync_delta_s: Optional[float] = None  # diagnostic only

        self._cmd_lock = threading.Lock()
        self._last_cmd: Optional[ControlCommand] = None
        self._closed = False

        self.create_subscription(Odometry, "/odometry", self._on_odom, 10)
        self.create_subscription(State, "/state", self._on_state, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_timer_cb,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        # Dedicated spin thread, started here -- before anything else
        # can use this node -- so the republish timer above fires
        # reliably at publish_rate_hz regardless of what thread or
        # cadence the perception loop runs on. See module docstring.
        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self):
        while not self._spin_stop.is_set():
            self._executor.spin_once(timeout_sec=0.01)

    # ---- subscriptions (spin thread) -----------------------------------

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        position = (p.x, p.y, p.z)
        velocity = (v.x, v.y, v.z)
        attitude_quat = (q.x, q.y, q.z, q.w)
        with self._state_lock:
            self._connected = True
            self._position = position
            self._velocity = velocity
            self._attitude_quat = attitude_quat
        t_odom = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._odom_lock:
            self._odom_deque.append((t_odom, position, velocity, attitude_quat))

    def _on_state(self, msg: State):
        with self._state_lock:
            self._connected = msg.connected
            self._armed = msg.armed
            self._mode = msg.mode

    # ---- time-aligned odometry (caller's thread) -------------------------
    # NOT part of VehicleInterface's minimal Protocol -- an addition for
    # whoever wires the orchestrator (Step AR), matching exactly what
    # CheapStage.infer()'s `odom` parameter is documented to expect. See
    # the buffer's declaration in __init__ for why this duplicates
    # record_pass.FlightControl.state_at() rather than sharing code with
    # it.

    @staticmethod
    def _lerp3(a, b, frac):
        return tuple(a[i] + frac * (b[i] - a[i]) for i in range(3))

    @staticmethod
    def _nlerp4(a, b, frac):
        """Linear interpolation + renormalize -- a standard cheap
        approximation to slerp, valid when the inter-sample rotation is
        small (holds here: /odometry's rate is high relative to the
        rotation accrued between consecutive samples -- same assumption
        record_pass.FlightControl._nlerp4/SimFrameSource's gyro
        interpolation make about their own buffers)."""
        raw = tuple(a[i] + frac * (b[i] - a[i]) for i in range(4))
        norm = sum(c * c for c in raw) ** 0.5
        if norm == 0.0:
            return a
        return tuple(c / norm for c in raw)

    def state_at(self, t_capture: float):
        """Returns (position, velocity, attitude_quat, odom_valid),
        interpolated to t_capture from the buffered /odometry samples --
        identical bracket/nearest-fallback/staleness contract to
        record_pass.FlightControl.state_at() (see that method's
        docstring for the full reasoning). odom_valid=False must not be
        read as 'no motion' -- the returned values are still populated
        from the nearest sample, just outside odom_staleness_s."""
        with self._odom_lock:
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

    # ---- fixed-rate republish timer (spin thread) -----------------------

    def _publish_timer_cb(self):
        # Hold the command lock through publication: landing's explicit
        # zero publish cannot be followed by an older command that the
        # timer copied before send() replaced it.
        with self._cmd_lock:
            cmd = self._last_cmd
            if cmd is None:
                # Monitoring and the period after entering LAND do not
                # publish velocity commands.
                return
            self.command_expired = (self._last_command_wall is None or
                time.monotonic() - self._last_command_wall >= self._command_timeout_s)
            if self.command_expired:
                cmd = ControlCommand(0.0, 0.0, "blind", None)
            with self._state_lock:
                quat = self._attitude_quat
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            if (quat is not None and all(math.isfinite(c) for c in quat)
                    and any(c != 0.0 for c in quat)):
                x, y, z, w = quat
                yaw = math.atan2(2.0 * (w * z + x * y),
                                 1.0 - 2.0 * (y * y + z * z))
                twist.twist.linear.x = cmd.fwd_vel * math.cos(yaw)
                twist.twist.linear.y = cmd.fwd_vel * math.sin(yaw)
            twist.twist.angular.z = cmd.yaw_rate
            self._vel_pub.publish(twist)

    # ---- VehicleInterface (caller's thread) ------------------------------

    def send(self, cmd: ControlCommand) -> None:
        if not all(math.isfinite(v) for v in (cmd.fwd_vel, cmd.yaw_rate)):
            cmd = ControlCommand(0.0, 0.0, "blind", None)
        with self._cmd_lock:
            self._last_cmd = cmd
            self._last_command_wall = time.monotonic()
            self.command_expired = False

    def state(self) -> VehicleState:
        with self._state_lock:
            return VehicleState(
                connected=self._connected,
                armed=self._armed,
                mode=self._mode,
                position=self._position,
                velocity=self._velocity,
                attitude_quat=self._attitude_quat,
            )

    # ---- convenience beyond the Protocol: getting airborne ---------------
    # NOT part of VehicleInterface -- arm/takeoff is a one-time startup
    # sequence outside the closed control loop, not something
    # SectorController or the orchestrator's per-frame path touches.
    # Reuses record_pass.FlightControl.guided_arm_takeoff()'s proven
    # logic/timeouts (including its wall-clock use, which is correct
    # HERE specifically because this is a real-world startup handshake
    # with its own service-call timeouts, not the deterministic,
    # t_capture-driven per-frame control path Step AN's "no wall clock"
    # rule governs) rather than re-deriving it differently and risking
    # a new, subtly different bug in something already validated.

    @staticmethod
    def _deadline(timeout_s):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout must be finite and positive")
        return time.monotonic() + timeout_s

    @staticmethod
    def _remaining(deadline, operation):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"timed out waiting for {operation}")
        return remaining

    @staticmethod
    def _check_cancel(cancel_event):
        if cancel_event is not None and cancel_event.is_set():
            raise RuntimeError("takeoff cancelled")

    def _wait_for_service(self, client, name, timeout_sec=2.0, *, deadline=None,
                          cancel_event=None):
        if deadline is None:
            deadline = self._deadline(timeout_sec)
        while True:
            self._check_cancel(cancel_event)
            remaining = self._remaining(deadline, name)
            if client.wait_for_service(timeout_sec=min(0.2, remaining)):
                return

    def _wait_until(self, predicate, name, deadline, cancel_event=None):
        while True:
            self._check_cancel(cancel_event)
            remaining = self._remaining(deadline, name)
            if predicate():
                return
            if self._spin_stop.is_set():
                raise RuntimeError(f"vehicle closed while waiting for {name}")
            time.sleep(min(0.05, remaining))

    def _request(self, client, request, name, deadline, cancel_event=None):
        self._wait_for_service(client, name, deadline=deadline, cancel_event=cancel_event)
        self._check_cancel(cancel_event)
        self._remaining(deadline, name)
        future = client.call_async(request)
        # The background thread exclusively owns executor spinning. A
        # second spin_until_future_complete here races its spin_once().
        self._wait_until(future.done, f"{name} response", deadline, cancel_event)
        response = future.result()
        if response is None:
            raise RuntimeError(f"{name} returned no response")
        return response

    def guided_arm_takeoff(self, altitude: float, timeout_s: float = 30.0,
                           cancel_event=None):
        """Enter GUIDED, arm, and reach altitude within one wall deadline."""
        if not math.isfinite(altitude) or altitude <= 0:
            raise ValueError("altitude must be finite and positive")
        deadline = self._deadline(timeout_s)
        response = self._request(
            self._set_mode_cli, SetMode.Request(custom_mode="GUIDED"),
            "/set_mode GUIDED", deadline, cancel_event,
        )
        if not response.mode_sent:
            raise RuntimeError(f"GUIDED mode request failed: {response}")
        self._wait_until(lambda: self.state().mode == "GUIDED", "GUIDED mode", deadline, cancel_event)

        if not self.state().armed:
            response = self._request(
                self._arm_cli, CommandBool.Request(value=True), "/cmd/arming", deadline, cancel_event,
            )
            if not response.success:
                raise RuntimeError(f"arm failed: {response}")
            self._wait_until(lambda: self.state().armed, "armed telemetry", deadline, cancel_event)

        response = self._request(
            self._takeoff_cli, CommandTOL.Request(min_pitch=0.0, yaw=0.0, altitude=altitude),
            "/cmd/takeoff", deadline, cancel_event,
        )
        if not response.success:
            raise RuntimeError(f"takeoff failed: {response}")

        def reached_altitude():
            position = self.state().position
            return position is not None and abs(position[2] - altitude) < 0.5

        self._wait_until(reached_altitude, f"altitude {altitude} m", deadline, cancel_event)

    def land_and_wait(self, timeout_s: float = 90.0):
        """Publish a stop, enter LAND, then await disarm within one deadline.

        Call before close() after an autonomous run, including failed
        takeoff attempts. No forced disarm is sent while airborne.
        """
        deadline = self._deadline(timeout_s)
        self.send(ControlCommand(fwd_vel=0.0, yaw_rate=0.0,
                                 mode="cruise", target_sector=None))
        self._publish_timer_cb()  # synchronously deliver stop before LAND
        response = self._request(
            self._set_mode_cli, SetMode.Request(custom_mode="LAND"),
            "/set_mode LAND", deadline,
        )
        if not response.mode_sent:
            raise RuntimeError(f"LAND mode request failed: {response}")
        self._wait_until(lambda: self.state().mode == "LAND" or not self.state().armed,
                         "LAND mode", deadline)
        with self._cmd_lock:
            self._last_cmd = None
        self._wait_until(lambda: not self.state().armed, "landing/disarm", deadline)

    # ---- teardown ---------------------------------------------------------

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._spin_stop.set()
        self._spin_thread.join(timeout=2.0)
        self._executor.remove_node(self)
        self._executor.shutdown()
        self.destroy_node()
