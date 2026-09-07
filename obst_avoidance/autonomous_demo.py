"""One bounded Gazebo-only CheapStage flight with the live sector display."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import cv2
import numpy as np

from .control import SectorController, SectorControllerConfig
from .live_viewer import draw_overlay, _window_closed, WINDOW
from .perception import CheapStage, geometry
from .runtime import Orchestrator, OrchestratorConfig, ZONE_SEGMENTS
from .runtime.processes import stop_process_group


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--zone', choices=ZONE_SEGMENTS, default='zone_A')
    parser.add_argument('--spawn-y', type=float, default=-3.8,
                        help='Default aims at textured treeA1 from the Zone A start')
    parser.add_argument('--altitude', type=float, default=3.0)
    parser.add_argument('--max-wall-time', type=float, default=240.0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--gazebo-gui', action='store_true')
    parser.add_argument('--controller', choices=('baseline', 'pathtrack', 'pathtrack_v2'), default='baseline',
                        help='baseline = fixed world +x goal (unchanged); pathtrack = '
                             'pure-pursuit lookahead on the straight spawn->goal route')
    parser.add_argument('--lookahead-m', type=float, default=3.0,
                        help='pathtrack only: lookahead distance along the reference route')
    args = parser.parse_args(argv)
    if not np.isfinite([args.spawn_y, args.altitude, args.max_wall_time, args.lookahead_m]).all():
        parser.error('flight parameters must be finite')
    if args.lookahead_m <= 0:
        parser.error('--lookahead-m must be positive')
    # The existing obstacle-footprint evaluation assumes the 3m cruise plane.
    if args.altitude != 3.0 or args.max_wall_time <= 0:
        parser.error('this demo requires altitude 3m and a positive duration')
    if args.output_dir.exists():
        parser.error('choose a new output directory; existing runs are preserved')
    args.output_dir.mkdir(parents=True)
    segment = ZONE_SEGMENTS[args.zone]
    config = dict(zone=args.zone, spawn_x=segment['start_x'], spawn_y=args.spawn_y,
                  goal_x=segment['goal_x_m'], altitude=args.altitude,
                  max_wall_time=args.max_wall_time, speed_mps=0.8, sectors=11,
                  feature_count=168, perception='cheap', seed=0,
                  velocity_frame='body forward converted to world ENU / LOCAL_NED',
                  controller=args.controller, lookahead_m=args.lookahead_m,
                  visual_recording_fps=25.0)
    (args.output_dir / 'config.json').write_text(json.dumps(config, indent=2))
    cancel = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: cancel.set())
    stack = source = vehicle = writer = node_ros = None
    preview_previous = None
    last_image = None
    window_open = not args.headless
    summary = dict(status='starting', config=config, landing='not_needed')
    display_count = 0
    stage = None
    takeoff_attempted = False
    stack_log = (args.output_dir / 'simulator.log').open('x')

    def requested_stop():
        nonlocal window_open
        if window_open and _window_closed():
            window_open = False
            cancel.set()
        if stack is not None and stack.poll() is not None:
            cancel.set()
        return cancel.is_set()

    def show(packet, belief, features, odom, label):
        nonlocal writer, last_image, display_count
        position = odom[0]
        if position is not None:
            label += f' | x={position[0]:.1f} y={position[1]:.1f} z={position[2]:.1f}'
        last_image = draw_overlay(packet.image, belief, features, stage.last_tracks, label)
        if writer is None:
            writer = cv2.VideoWriter(str(args.output_dir / 'camera.avi'),
                                     cv2.VideoWriter_fourcc(*'MJPG'), 25.0,
                                     (last_image.shape[1], last_image.shape[0]))
            if not writer.isOpened():
                raise RuntimeError('Could not create the camera recording')
        writer.write(last_image)
        display_count += 1
        if window_open:
            cv2.imshow(WINDOW, last_image)
        if display_count % 100 == 0:
            print(f'CAMERA frame={display_count} {label} valid={belief.valid.sum()}/11', flush=True)

    def preview(phase):
        nonlocal preview_previous
        packet = source.read()
        if packet is not None:
            if preview_previous is not None:
                odom = vehicle.state_at(packet.t_capture)
                belief, features = stage.infer(packet, preview_previous, odom)
                show(packet, belief, features, odom, phase)
            preview_previous = packet
        requested_stop()
        time.sleep(0.005)

    def with_camera(action, phase):
        outcome = []
        def work():
            try:
                action()
            except BaseException as error:
                outcome.append(error)
        thread = threading.Thread(target=work, daemon=True)
        thread.start()
        preview_error = None
        while thread.is_alive():
            if preview_error is None:
                try:
                    preview(phase)
                except BaseException as error:
                    preview_error = error
                    cancel.set()
            else:
                time.sleep(0.02)
        thread.join()
        if preview_error is not None:
            raise preview_error
        if outcome:
            raise outcome[0]

    try:
        # cv2 (imported above for the live camera window) sets
        # QT_QPA_PLATFORM_PLUGIN_PATH to OpenCV's own bundled Qt plugins on
        # import. If the Gazebo GUI subprocess inherits that, its xcb plugin
        # load fails ("Could not load the Qt platform plugin xcb") and the
        # whole sim dies -> SITL never connects -> "No JSON sensor message".
        # Strip the cv2 Qt vars for the gz subprocess ONLY, so Gazebo uses the
        # system Qt plugins; this process keeps cv2's env for its own imshow
        # window, so BOTH the 3D Gazebo GUI and the camera window work.
        gz_env = dict(os.environ)
        for _qt_var in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_PLUGIN_PATH'):
            gz_env.pop(_qt_var, None)
        stack = subprocess.Popen([
            'ros2', 'launch', 'obst_avoidance', 'env_zones.launch.py',
            f'use_gui:={str(args.gazebo_gui).lower()}',
            f'spawn_x:={segment["start_x"]}', f'spawn_y:={args.spawn_y}',
        ], cwd=args.output_dir, stdout=stack_log, stderr=subprocess.STDOUT,
            start_new_session=True, env=gz_env)
        print(f'SIMULATOR pid/pgid={stack.pid}; outputs={args.output_dir}', flush=True)
        import rclpy
        from rcl_interfaces.srv import GetParameters
        from .frame_source import SimFrameSource
        from .platform import MavrosVehicle
        node_ros = rclpy
        rclpy.init()
        vehicle = MavrosVehicle(node_name='autonomous_demo_vehicle')
        # Startup timeouts return as soon as the heartbeat/intrinsics arrive, so
        # a large value only adds patience. The 3D Gazebo GUI (--gazebo-gui) is
        # much slower to bring the SITL stack up than headless; 60s was too tight
        # and produced "no SITL heartbeat within 60s". 180s covers GUI startup
        # without slowing a fast headless connect. (Claude, FLIGHT-001.)
        source = SimFrameSource(heartbeat_timeout_s=180, startup_timeout_s=180)
        source.wait_for_intrinsics(timeout_s=90)
        stage = CheapStage(source.intrinsics)
        if window_open:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, 960, 915)
            cv2.imshow(WINDOW, np.zeros((610, 640, 3), dtype=np.uint8))
        # Verify the real plugin contract before arming, using its own ROS
        # parameter service. The background vehicle thread alone spins ROS.
        frame_client = vehicle.create_client(GetParameters, '/setpoint_velocity/get_parameters')
        def verify_frame():
            if not frame_client.wait_for_service(timeout_sec=30):
                raise RuntimeError('MAVROS velocity-frame parameter service unavailable')
            future = frame_client.call_async(GetParameters.Request(names=['mav_frame']))
            deadline = time.monotonic() + 10
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done() or future.result() is None:
                raise RuntimeError('Could not verify MAVROS velocity frame')
            values = future.result().values
            if len(values) != 1 or values[0].string_value != 'LOCAL_NED':
                raise RuntimeError(f'Expected LOCAL_NED velocity plugin, got {values}')
        with_camera(verify_frame, 'CHECKING SIMULATOR')
        # SITL pre-arm checks (accels-inconsistent, GPS-still-configuring) need
        # ~30-60s to settle after the sim starts; the previous 3 attempts / 10s
        # gave up at ~22s, before GPS even finished configuring (~21s). Widened
        # to 6 attempts / 20s (~100s budget) to match the escalating-retry
        # patience used by the historical batch scripts. (Claude, FLIGHT-001.)
        takeoff_attempts = 6
        takeoff_retry_wait_s = 20
        for attempt in range(1, takeoff_attempts + 1):
            if cancel.is_set():
                raise InterruptedError('Demo cancelled before takeoff')
            print(f'TAKEOFF attempt={attempt}, altitude={args.altitude}m', flush=True)
            try:
                takeoff_attempted = True
                with_camera(lambda: vehicle.guided_arm_takeoff(
                    args.altitude, timeout_s=120, cancel_event=cancel), 'TAKEOFF')
                break
            except RuntimeError as error:
                if attempt == takeoff_attempts or vehicle.state().armed:
                    raise
                print(f'Waiting for startup readiness after: {error}', flush=True)
                until = time.monotonic() + takeoff_retry_wait_s
                while time.monotonic() < until and not requested_stop():
                    preview('WAITING FOR EKF')

        stage.reset()
        bearings = tuple(geometry.sector_bearings_rad(
            source.intrinsics['width'], source.intrinsics['fx'], source.intrinsics['cx'], n_sectors=11))
        controller = SectorController(SectorControllerConfig(n_sectors=11, sector_bearings_rad=bearings,
            robust_hysteresis=args.controller == 'pathtrack_v2'))
        # CTRL-COMPARE: same SectorController for both arms; ONLY the goal-heading
        # source differs. Baseline = fixed world +x (goal_provider=None). Variant =
        # pure-pursuit lookahead on the straight route through the spawn toward the
        # goal (A=spawn, B=(goal_x, spawn_y)), so the intended return path is
        # unambiguous. Perception, speed, sectors and gains are IDENTICAL.
        goal_provider = None
        controller_version = 'baseline_world_bearing'
        if args.controller in ('pathtrack', 'pathtrack_v2'):
            from .control.reference_path import ReferencePathGoal
            goal_provider = ReferencePathGoal(
                ax=segment['start_x'], ay=args.spawn_y,
                bx=segment['goal_x_m'], by=args.spawn_y, lookahead_m=args.lookahead_m)
            controller_version = f'{args.controller}_lookahead_{args.lookahead_m:g}m'
        def observed(packet, belief, features, odom, cmd):
            show(packet, belief, features, odom, f'AUTONOMOUS {cmd.mode.upper()} | sector={cmd.target_sector}')
        orchestrator = Orchestrator(source, stage, controller, vehicle,
            OrchestratorConfig(goal_x_m=segment['goal_x_m'], max_wall_time_s=args.max_wall_time,
                               log_path=str(args.output_dir / 'frames.jsonl'), frame_timeout_s=5,
                               goal_provider=goal_provider, controller_version=controller_version,
                               scene_version='env_zones.sdf'),
            on_frame=observed, should_stop=requested_stop)
        print(f'FLIGHT: CheapStage + shared SectorController active [{controller_version}]', flush=True)
        result = orchestrator.run()
        summary.update(status='finished', result=asdict(result))
        print(f'RESULT: {result.stop_reason}, frames={result.n_frames}, '
              f'sim_duration={result.duration_s:.1f}s, modes={result.mode_counts}', flush=True)
    except BaseException as error:
        summary.update(status='interrupted' if isinstance(error, (InterruptedError, KeyboardInterrupt)) else 'error',
                       error=f'{type(error).__name__}: {error}')
        print(f'DEMO: {summary["error"]}', flush=True)
    finally:
        if vehicle is not None and (takeoff_attempted or vehicle.state().armed):
            print('LANDING: stopping forward commands, landing and waiting for disarm', flush=True)
            try:
                if source is not None and stage is not None:
                    with_camera(lambda: vehicle.land_and_wait(timeout_s=120), 'LANDING')
                else:
                    vehicle.land_and_wait(timeout_s=120)
                summary['landing'] = 'disarmed'
            except BaseException as error:
                summary['landing'] = f'failed: {error}'
        if vehicle is not None:
            try:
                summary['final_vehicle_state'] = asdict(vehicle.state())
            except Exception as error:
                summary.setdefault('cleanup_errors', []).append(str(error))
        for resource in (writer, vehicle, source):
            if resource is not None:
                try:
                    resource.release() if resource is writer else resource.close()
                except Exception as error:
                    summary.setdefault('cleanup_errors', []).append(str(error))
        cleanup_actions = []
        if node_ros is not None:
            cleanup_actions.append(lambda: node_ros.shutdown() if node_ros.ok() else None)
        if not args.headless:
            cleanup_actions.append(cv2.destroyAllWindows)
        if last_image is not None:
            cleanup_actions.append(lambda: cv2.imwrite(str(args.output_dir / 'last_frame.png'), last_image))
        if stack is not None:
            cleanup_actions.append(lambda: stop_process_group(stack))
        cleanup_actions.append(stack_log.close)
        for action in cleanup_actions:
            try:
                action()
            except Exception as error:
                summary.setdefault('cleanup_errors', []).append(str(error))
        summary['displayed_frames'] = display_count
        (args.output_dir / 'result.json').write_text(json.dumps(summary, indent=2))
        print(f'SAVED: {args.output_dir / "result.json"}; cleanup errors={summary.get("cleanup_errors", [])}', flush=True)
    return 0 if summary['status'] == 'finished' and summary['landing'] == 'disarmed' and not summary.get('cleanup_errors') else 1


if __name__ == '__main__':
    raise SystemExit(main())
