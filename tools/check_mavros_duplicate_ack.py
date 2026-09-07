#!/usr/bin/env python3
"""Exercise the real MAVROS command plugin with a local fake FCU.

Run manually after sourcing the workspace. Uses an isolated ROS domain,
loopback UDP with ephemeral ports, and a fake (not real/SITL) flight
controller. Requests only autopilot capabilities. Does not launch Gazebo.
"""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--requests', type=int, default=25)
    args = parser.parse_args()
    if args.requests <= 0 or not args.binary.is_file():
        parser.error('require an existing binary and a positive request count')
    if args.log.exists():
        parser.error('log already exists')

    # This domain is for the bounded diagnostic only; no live vehicle topics.
    os.environ['ROS_DOMAIN_ID'] = '87'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from mavros_msgs.srv import CommandLong
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    stop = threading.Event()
    received = []
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 0))
    udp.settimeout(0.1)
    fake_port = udp.getsockname()[1]

    def fake_fcu():
        encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        decoder = mavlink.MAVLink(None)
        peer = None
        last_heartbeat = 0.0
        while not stop.is_set():
            try:
                data, peer = udp.recvfrom(65535)
            except socket.timeout:
                data = b''
            for msg in decoder.parse_buffer(data) or []:
                if msg.get_type() == 'COMMAND_LONG':
                    received.append(msg.command)
                    ack = mavlink.MAVLink_command_ack_message(
                        msg.command, mavlink.MAV_RESULT_ACCEPTED,
                        target_system=msg.get_srcSystem(),
                        target_component=msg.get_srcComponent())
                    # A burst lets duplicate callbacks race transaction removal,
                    # matching the crash recovered from the actual core dump.
                    udp.sendto(ack.pack(encoder) * 200, peer)
                    if msg.command == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                        version = mavlink.MAVLink_autopilot_version_message(
                            0, 0x04080000, 0, 0, 0, [0] * 8, [0] * 8,
                            [0] * 8, 0, 0, 0)
                        udp.sendto(version.pack(encoder), peer)
            now = time.monotonic()
            if peer and now - last_heartbeat > 0.5:
                hb = mavlink.MAVLink_heartbeat_message(
                    mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    0, 0, mavlink.MAV_STATE_STANDBY, 3)
                udp.sendto(hb.pack(encoder), peer)
                last_heartbeat = now

    worker = threading.Thread(target=fake_fcu, daemon=True)
    process = node = None
    rclpy.init()
    try:
        with args.log.open('x') as log:
            process = subprocess.Popen([
                str(args.binary.resolve()), '--ros-args',
                '-r', '__ns:=/duplicate_ack_test',
                '-p', 'use_sim_time:=false',
                '-p', f'fcu_url:=udp://127.0.0.1:0@127.0.0.1:{fake_port}',
                '-p', 'plugin_allowlist:=[command, sys_status]',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            worker.start()
            node = rclpy.create_node('duplicate_ack_probe')
            client = node.create_client(CommandLong, '/duplicate_ack_test/cmd/command')
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError('MAVROS command service did not start')
            # Let startup's capability request finish before our own calls.
            until = time.monotonic() + 3
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.1)
            for i in range(args.requests):
                request = CommandLong.Request()
                request.command = mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                request.confirmation = 1
                request.param1 = 1.0
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=8)
                if not future.done() or future.result() is None or not future.result().success:
                    raise RuntimeError(f'Command {i + 1} failed; MAVROS exit={process.poll()}')
                if process.poll() is not None:
                    raise RuntimeError(f'MAVROS crashed: {process.returncode}')
                # Drain this burst before reusing the command ID: MAVLink
                # COMMAND_ACK has no per-request transaction identifier.
                time.sleep(0.15)
            if len(received) < args.requests:
                raise RuntimeError('Fake FCU did not receive all requested commands')
            log.flush()
            hits = args.log.read_text().count('already processed')
            if not hits:
                raise RuntimeError('Commands passed, but duplicate-race guard was not exercised')
            print(f'PASS: {args.requests} requests completed; {hits} duplicate ACKs caught; '
                  f'MAVROS alive; fake FCU saw {len(received)} commands', flush=True)
    finally:
        stop.set()
        if worker.ident is not None:
            worker.join(timeout=1)
        udp.close()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
