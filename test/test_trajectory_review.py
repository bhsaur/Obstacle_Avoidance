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


def test_custom_scene_clearance_uses_recorded_geometry():
    from tools.plot_trajectories import min_clearance,clearance_series
    obstacles=[dict(kind='box',x=17,y=0,half_x=1,half_y=1.5)]
    assert min_clearance([17],[2],obstacles)==.5
    assert clearance_series([17],[2],obstacles).tolist()==[.5]
