"""Offline rendering/replay checks; no ROS or vehicle initialization."""
import json
import os
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

from obst_avoidance.live_viewer import PANEL_HEIGHT, draw_overlay, _window_closed
from obst_avoidance.perception import CheapStage
from obst_avoidance.frame_source import FramePacket


def test_overlay_preserves_inputs_and_unknown_scores():
    rgb = np.full((480, 640, 3), (20, 40, 80), dtype=np.uint8)
    stage = CheapStage(dict(width=640, height=480, fx=205, fy=205, cx=320, cy=240))
    a = FramePacket(rgb, np.zeros(3), 0, 0)
    b = FramePacket(rgb, np.zeros(3), 0.04, 1)
    belief, features = stage.infer(b, a, (None, None, None, False))
    saved = rgb.copy()
    output = draw_overlay(rgb, belief, features)
    assert output.shape == (480 + PANEL_HEIGHT, 640, 3)
    np.testing.assert_array_equal(rgb, saved)
    assert np.isnan(belief.scores).all() and not belief.valid.any()
    np.testing.assert_array_equal(output[200, 20], [80, 40, 20])


def test_replay_terminates_without_ros_import(tmp_path):
    recording = tmp_path / 'recording'
    (recording / 'frames').mkdir(parents=True)
    rows = []
    for seq in range(3):
        name = f'frame_{seq:06d}.png'
        cv2.imwrite(str(recording / 'frames' / name), np.zeros((480, 640, 3), np.uint8))
        rows.append(dict(seq=seq, t_capture=seq * .04, frame_file=name, gyro=[0, 0, 0], gyro_valid=True))
    (recording / 'packets.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    (recording / 'intrinsics.json').write_text(json.dumps(dict(width=640, height=480, k=[205, 0, 320, 0, 205, 240, 0, 0, 1])))
    output = tmp_path / 'view.png'
    script = ('import sys; from obst_avoidance.live_viewer import main; '
              'main(sys.argv[1:]); assert "rclpy" not in sys.modules')
    result = subprocess.run([sys.executable, '-B', '-c', script, '--recording', str(recording),
                             '--headless', '--output', str(output)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert '2 frames' in result.stdout
    assert cv2.imread(str(output)).shape == (610, 640, 3)


def test_window_x_button_is_a_clean_exit(monkeypatch):
    monkeypatch.setattr(cv2, 'waitKey', lambda _: -1)
    def destroyed(*args):
        raise cv2.error('NULL guiReceiver')
    monkeypatch.setattr(cv2, 'getWindowProperty', destroyed)
    assert _window_closed()
