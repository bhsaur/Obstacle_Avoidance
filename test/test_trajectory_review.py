import json

from tools.plot_trajectories import load_run, _status


def test_direct_frames_path_uses_result_and_preserves_unknown_mode(tmp_path):
    frames = tmp_path / 'frames.jsonl'
    frames.write_text(json.dumps(dict(seq=1, vehicle_position=[1, 2, 3], cmd={})) + '\n')
    (tmp_path / 'result.json').write_text(json.dumps({'result': {'stop_reason': 'goal_reached'}}))
    run = load_run(str(frames))
    assert run['modes'] == ['unknown']
    assert _status(run['result']) == 'goal_x_crossed'


def test_empty_position_log_is_not_plotted(tmp_path):
    frames = tmp_path / 'frames.jsonl'
    frames.write_text(json.dumps(dict(seq=1, vehicle_position=None)) + '\n')
    assert load_run(str(frames)) is None
