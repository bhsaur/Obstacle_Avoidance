"""Vehicle lifecycle and world-frame publication without a live ROS graph."""
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from obst_avoidance.control.types import ControlCommand
from obst_avoidance.platform import vehicle as module


class Clock:
    def __init__(self):
        self.now = 0.0
        self.on_sleep = lambda: None

    def sleep(self, duration):
        self.now += duration
        self.on_sleep()


@pytest.fixture
def fake(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock.now,
                                                       sleep=clock.sleep))
    vehicle = module.MavrosVehicle.__new__(module.MavrosVehicle)
    vehicle._state_lock = threading.Lock()
    vehicle._cmd_lock = threading.Lock()
    vehicle._connected = True
    vehicle._armed = False
    vehicle._mode = "STABILIZE"
    vehicle._position = (0.0, 0.0, 0.0)
    vehicle._velocity = (0.0, 0.0, 0.0)
    vehicle._attitude_quat = (0.0, 0.0, 0.0, 1.0)
    vehicle._last_cmd = None
    vehicle._last_command_wall = None
    vehicle._command_timeout_s = 1.0
    vehicle.command_expired = False
    vehicle._spin_stop = threading.Event()
    vehicle._closed = False
    vehicle._executor = Mock()
    vehicle._spin_thread = Mock()
    vehicle._vel_pub = Mock()
    # Supply the real generated stamp type without initializing ROS.
    from builtin_interfaces.msg import Time
    vehicle.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    vehicle.destroy_node = Mock()
    vehicle._set_mode_cli = Mock()
    vehicle._arm_cli = Mock()
    vehicle._takeoff_cli = Mock()
    for client in (vehicle._set_mode_cli, vehicle._arm_cli, vehicle._takeoff_cli):
        client.wait_for_service.return_value = True
    return vehicle, clock


def future(response, *, ready=lambda: True, finish=lambda: None):
    def result():
        finish()
        return response
    return SimpleNamespace(done=ready, result=result)


def successful_takeoff(vehicle, clock, delay=0.0):
    def mode(request):
        ready_at = clock.now + delay
        return future(SimpleNamespace(mode_sent=True), ready=lambda: clock.now >= ready_at,
                      finish=lambda: setattr(vehicle, "_mode", request.custom_mode))

    def arm(_):
        ready_at = clock.now + delay
        return future(SimpleNamespace(success=True), ready=lambda: clock.now >= ready_at,
                      finish=lambda: setattr(vehicle, "_armed", True))

    def takeoff(request):
        ready_at = clock.now + delay
        return future(SimpleNamespace(success=True), ready=lambda: clock.now >= ready_at,
                      finish=lambda: setattr(vehicle, "_position", (0.0, 0.0, request.altitude)))

    vehicle._set_mode_cli.call_async.side_effect = mode
    vehicle._arm_cli.call_async.side_effect = arm
    vehicle._takeoff_cli.call_async.side_effect = takeoff


def test_takeoff_uses_background_executor_only(fake):
    vehicle, clock = fake
    successful_takeoff(vehicle, clock, delay=0.1)
    vehicle.guided_arm_takeoff(3.0, timeout_s=1.0)
    assert vehicle.state().armed
    assert vehicle.state().position[2] == 3.0
    assert not vehicle._executor.mock_calls


def test_takeoff_deadline_covers_all_service_stages(fake):
    vehicle, clock = fake
    successful_takeoff(vehicle, clock, delay=0.3)
    with pytest.raises(TimeoutError, match="takeoff response"):
        vehicle.guided_arm_takeoff(3.0, timeout_s=0.7)
    assert clock.now == pytest.approx(0.7)
    assert vehicle.state().position[2] == 0.0
    assert not vehicle._executor.mock_calls


def test_absent_service_is_bounded(fake):
    vehicle, clock = fake
    def unavailable(timeout_sec):
        clock.sleep(timeout_sec)
        return False
    vehicle._set_mode_cli.wait_for_service.side_effect = unavailable
    with pytest.raises(TimeoutError, match="set_mode"):
        vehicle.guided_arm_takeoff(3.0, timeout_s=0.4)
    assert clock.now == pytest.approx(0.4)
    vehicle._set_mode_cli.call_async.assert_not_called()


def test_mode_rejection_prevents_arming(fake):
    vehicle, _ = fake
    vehicle._set_mode_cli.call_async.return_value = future(SimpleNamespace(mode_sent=False))
    with pytest.raises(RuntimeError, match="GUIDED mode request failed"):
        vehicle.guided_arm_takeoff(3.0)
    vehicle._arm_cli.call_async.assert_not_called()


def test_ack_without_mode_telemetry_is_bounded(fake):
    vehicle, clock = fake
    vehicle._set_mode_cli.call_async.return_value = future(SimpleNamespace(mode_sent=True))
    with pytest.raises(TimeoutError, match="GUIDED mode"):
        vehicle.guided_arm_takeoff(3.0, timeout_s=0.2)
    assert clock.now == pytest.approx(0.2)
    vehicle._arm_cli.call_async.assert_not_called()


def test_takeoff_cancellation_stops_pending_future(fake):
    vehicle, clock = fake
    cancelled = threading.Event()
    clock.on_sleep = cancelled.set
    vehicle._set_mode_cli.call_async.return_value = future(None, ready=lambda: False)
    with pytest.raises(RuntimeError, match="cancelled"):
        vehicle.guided_arm_takeoff(3.0, cancel_event=cancelled)
    assert clock.now == pytest.approx(0.05)
    vehicle._arm_cli.call_async.assert_not_called()


def test_land_publishes_zero_before_mode_and_waits_for_disarm(fake):
    vehicle, clock = fake
    vehicle._armed = True
    vehicle._mode = "GUIDED"
    vehicle.send(ControlCommand(fwd_vel=0.8, yaw_rate=0.4, mode="avoid", target_sector=1))
    events = []
    def publish(message):
        assert message.twist.linear.x == 0.0
        assert message.twist.linear.y == 0.0
        assert message.twist.angular.z == 0.0
        events.append("stop")
    vehicle._vel_pub.publish.side_effect = publish

    def mode(request):
        assert request.custom_mode == "LAND"
        events.append("LAND")
        return future(SimpleNamespace(mode_sent=True),
                      finish=lambda: setattr(vehicle, "_mode", "LAND"))
    vehicle._set_mode_cli.call_async.side_effect = mode
    def on_sleep():
        assert vehicle._last_cmd is None
        vehicle._armed = False
    clock.on_sleep = on_sleep
    vehicle.land_and_wait(timeout_s=1.0)
    assert events == ["stop", "LAND"]
    assert not vehicle.state().armed
    assert vehicle._last_cmd is None
    assert not vehicle._executor.mock_calls


def test_landing_disarm_wait_is_bounded(fake):
    vehicle, clock = fake
    vehicle._mode = "LAND"
    vehicle._armed = True
    vehicle._set_mode_cli.call_async.return_value = future(SimpleNamespace(mode_sent=True))
    with pytest.raises(TimeoutError, match="landing/disarm"):
        vehicle.land_and_wait(timeout_s=0.2)
    assert clock.now == pytest.approx(0.2)
    assert vehicle._last_cmd is None
    vehicle._arm_cli.call_async.assert_not_called()  # no forced airborne disarm


@pytest.mark.parametrize("yaw", [math.pi / 2, -math.pi / 3])
def test_body_forward_is_rotated_to_world_enu(fake, yaw):
    vehicle, _ = fake
    vehicle._attitude_quat = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    vehicle.send(ControlCommand(fwd_vel=0.8, yaw_rate=0.3, mode="avoid", target_sector=1))
    vehicle._publish_timer_cb()
    message = vehicle._vel_pub.publish.call_args.args[0]
    assert message.twist.linear.x == pytest.approx(0.8 * math.cos(yaw), abs=1e-10)
    assert message.twist.linear.y == pytest.approx(0.8 * math.sin(yaw), abs=1e-10)
    assert message.twist.angular.z == pytest.approx(0.3)


@pytest.mark.parametrize("attitude", [None, (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, float("nan"), 1.0)])
def test_unknown_attitude_does_not_invent_translation(fake, attitude):
    vehicle, _ = fake
    vehicle._attitude_quat = attitude
    vehicle.send(ControlCommand(fwd_vel=0.8, yaw_rate=0.0, mode="blind", target_sector=None))
    vehicle._publish_timer_cb()
    message = vehicle._vel_pub.publish.call_args.args[0]
    assert message.twist.linear.x == 0.0
    assert message.twist.linear.y == 0.0


def test_monitoring_publishes_nothing_and_close_is_idempotent(fake):
    vehicle, _ = fake
    vehicle._publish_timer_cb()
    vehicle._vel_pub.publish.assert_not_called()
    vehicle.close()
    vehicle.close()
    vehicle._executor.shutdown.assert_called_once()
    vehicle.destroy_node.assert_called_once()


def test_command_expiry_publishes_zero_without_new_inference(fake):
    vehicle, clock = fake
    vehicle.send(ControlCommand(0.8, 0.4, 'avoid', 1))
    clock.now = 0.9
    vehicle._publish_timer_cb()
    assert vehicle._vel_pub.publish.call_args.args[0].twist.linear.x == 0.8
    clock.now = 1.0
    vehicle._publish_timer_cb()
    twist = vehicle._vel_pub.publish.call_args.args[0].twist
    assert twist.linear.x == twist.linear.y == twist.angular.z == 0.0
    assert vehicle.command_expired
    vehicle.send(ControlCommand(0.3, -0.2, 'avoid', 1))
    vehicle._publish_timer_cb()
    assert vehicle._vel_pub.publish.call_args.args[0].twist.angular.z == -0.2
    assert not vehicle.command_expired
