"""Array-identity test: a frame replayed through LogFrameSource must be
byte-identical to what SimFrameSource produced for it at record time
(same dtype, shape, channel order, values). Getting this wrong silently
breaks every offline/online comparison downstream (Step K).

Requires a LIVE sim (Gazebo + ArduPilot SITL + the obst_avoidance camera
bridge) reachable the same way SimFrameSource always connects -- this is
an integration test, not a unit test, and is skipped automatically if no
sim is reachable within a short timeout so it doesn't break `colcon test`
in environments without one running.
"""
import json
import shutil
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy


def _try_connect_sim():
    from obst_avoidance.frame_source import SimFrameSource
    try:
        return SimFrameSource(heartbeat_timeout_s=5.0)
    except Exception:
        return None


def test_log_frame_source_array_identity():
    from obst_avoidance.frame_source import LogFrameSource

    rclpy.init()
    try:
        source = _try_connect_sim()
        if source is None:
            pytest.skip("no live sim reachable -- this is an integration test")

        # Capture one real frame from SimFrameSource.
        sim_packet = None
        for _ in range(200):  # a few seconds' worth of polling
            sim_packet = source.read()
            if sim_packet is not None:
                break
        assert sim_packet is not None, "never got a frame from SimFrameSource"

        # Write it out exactly the way record_pass.py does: RGB->BGR,
        # then cv2.imwrite -- this is the conversion under test.
        tmp_dir = Path(tempfile.mkdtemp(prefix="log_frame_source_identity_"))
        frames_dir = tmp_dir / "frames"
        frames_dir.mkdir()
        frame_name = f"frame_{sim_packet.seq:06d}.png"
        bgr = cv2.cvtColor(sim_packet.image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(frames_dir / frame_name), bgr)

        (tmp_dir / "packets.jsonl").write_text(json.dumps({
            "seq": sim_packet.seq,
            "t_capture": sim_packet.t_capture,
            "gyro": sim_packet.gyro.tolist(),
            "gyro_valid": sim_packet.gyro_valid,
            "frame_file": frame_name,
        }) + "\n")

        intrinsics = source.get_intrinsics()
        if intrinsics is not None:
            (tmp_dir / "intrinsics.json").write_text(json.dumps(intrinsics))

        try:
            log_source = LogFrameSource(str(tmp_dir))
            log_packet = log_source.read()

            assert log_packet is not None
            assert log_packet.image.dtype == sim_packet.image.dtype
            assert log_packet.image.shape == sim_packet.image.shape
            assert np.array_equal(log_packet.image, sim_packet.image), (
                "LogFrameSource image does not byte-match the original "
                "SimFrameSource image -- the RGB<->BGR round trip is broken"
            )
            assert np.array_equal(log_packet.gyro, sim_packet.gyro)
            assert log_packet.t_capture == sim_packet.t_capture
            assert log_packet.seq == sim_packet.seq
            assert log_packet.gyro_valid == sim_packet.gyro_valid
            assert log_source.dropped_frames == 0
        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)

        source.close()
    finally:
        rclpy.shutdown()
