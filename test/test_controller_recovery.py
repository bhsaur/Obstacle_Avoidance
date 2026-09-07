import math
import numpy as np
import pytest
from obst_avoidance.control import SectorController, SectorControllerConfig, ControlState
from obst_avoidance.control.reference_path import ReferencePathGoal
from obst_avoidance.perception.types import SectorBelief


def belief(confidence=1.0, valid=True):
    return SectorBelief(np.ones(11), np.full(11, valid), confidence, 'cheap', 0.0, ttc_s=np.full(11, 30.0))


def ctrl():
    return SectorController(SectorControllerConfig(n_sectors=11, goal_recovery=True, robust_hysteresis=True))


def test_small_heading_error_is_corrected_without_sector_quantization():
    cmd, _ = ctrl().step(belief(), -0.03, 0.03, 1.0, ControlState(0.03, 1, 'cheap', .9))
    assert cmd.yaw_rate == pytest.approx(-0.03)


def test_recovers_route_in_ideal_kinematic_vehicle():
    c, g, s = ctrl(), ReferencePathGoal(0, 0, 30, 0), ControlState(None, 0, None, None)
    x,y,yaw = 5., 3., .3
    for i in range(400):
        cmd,s = c.step(belief(), g.goal_heading((x,y),yaw).goal_heading_body, yaw, i*.05,s)
        yaw += cmd.yaw_rate*.05
        x += cmd.fwd_vel*math.cos(yaw)*.05
        y += cmd.fwd_vel*math.sin(yaw)*.05
    assert abs(y) < .1


@pytest.mark.parametrize('b', [belief(valid=False),belief(confidence=0),belief(confidence=float('nan'))])
def test_unknown_perception_stops_forward_motion(b):
    cmd,_ = ctrl().step(b,0,0,1,ControlState(None,0,None,None))
    assert cmd.fwd_vel == cmd.yaw_rate == 0


def test_low_confidence_and_goal_behind_reduce_translation():
    c,s = ctrl(),ControlState(None,0,None,None)
    slow,_=c.step(belief(confidence=.05),0,0,1,s)
    assert slow.fwd_vel == pytest.approx(.16)
    turn,_=c.step(belief(),2,0,1,s)
    assert turn.fwd_vel == 0
    assert turn.yaw_rate > 0


def test_uniform_high_rank_does_not_override_low_absolute_ttc():
    b = belief()
    b.ttc_s[:] = 0.5
    cmd,_ = ctrl().step(b,0,0,1,ControlState(None,0,None,None))
    assert cmd.fwd_vel == 0
    assert cmd.mode == 'blind'


def test_metric_depth_admission_uses_same_controller_without_source_branch():
    b = belief()
    b.ttc_s = None
    b.forward_depth_m = np.full(11, 0.5)
    b.source = 'heavy'
    cmd,_ = ctrl().step(b,0,0,1,ControlState(None,0,None,None))
    assert cmd.fwd_vel == 0
    b.forward_depth_m[:] = 4.0
    cmd,_ = ctrl().step(b,0,0,1,ControlState(None,0,None,None))
    assert cmd.fwd_vel > 0


def test_opening_steering_does_not_chase_best_score_inside_one_opening():
    c=SectorController(SectorControllerConfig(n_sectors=11,goal_recovery=True,
                        robust_hysteresis=True,opening_steering=True))
    s=ControlState(None,0,None,None)
    for i in range(10):
        b=belief()
        b.scores[:]=.1
        b.scores[2 if i%2 else 8]=1.0
        cmd,s=c.step(b,0.,0.,i*.1,s)
        assert cmd.yaw_rate == 0
        assert cmd.target_sector == 5


def test_opening_steering_never_interpolates_across_unknown_sector():
    c=SectorController(SectorControllerConfig(n_sectors=11,goal_recovery=True,
                        robust_hysteresis=True,opening_steering=True))
    b=belief();b.valid[5]=False
    cmd,s=c.step(b,0,0,.1,ControlState(None,0,None,0))
    assert cmd.target_sector != 5
    assert abs(s.target_world_bearing) > .01


def test_yaw_reversal_is_bounded_and_translation_waits_for_correct_turn():
    c=SectorController(SectorControllerConfig(n_sectors=11,goal_recovery=True,
                        robust_hysteresis=True,opening_steering=True))
    s=ControlState(0.,1.,'cheap',0.,previous_yaw_rate=.4)
    cmd,s=c.step(belief(),-.4,0.,.1,s)
    assert cmd.yaw_rate == pytest.approx(.3)
    assert cmd.fwd_vel == 0
    cmd,s=c.step(belief(valid=False),-.4,0.,.2,s)
    assert cmd.yaw_rate == cmd.fwd_vel == 0


def reorient_controller():
    return SectorController(SectorControllerConfig(n_sectors=11,goal_recovery=True,
          robust_hysteresis=True,opening_steering=True,reorient_to_goal=True))


def test_goal_outside_view_turns_toward_goal_even_without_flow():
    c=reorient_controller()
    cmd,s=c.step(belief(valid=False),-1.2,0,.1,ControlState(None,0,None,0))
    assert cmd.fwd_vel == 0
    assert cmd.yaw_rate < 0
    assert s.reorienting
    assert cmd.telemetry.switch_reason == 'reorient_to_goal'


def test_reorientation_exits_only_in_central_view_then_requires_perception():
    c=reorient_controller()
    _,s=c.step(belief(),1.2,0,.1,ControlState(None,0,None,0))
    cmd,s=c.step(belief(),.5,.7,.2,s)
    assert s.reorienting and cmd.fwd_vel == 0
    cmd,s=c.step(belief(valid=False),.1,1.1,.3,s)
    assert not s.reorienting
    assert cmd.mode == 'blind' and cmd.fwd_vel == 0


def test_stationary_reorientation_converges_from_both_sides():
    for yaw in (-2.,2.):
        c=reorient_controller();s=ControlState(None,0,None,0)
        entered=False
        for i in range(1,250):
            cmd,s=c.step(belief(valid=False),-yaw,yaw,i*.04,s)
            if s.reorienting:
                entered=True
                assert cmd.fwd_vel == 0
            yaw+=cmd.yaw_rate*.04
        assert entered
        assert abs(yaw)<.35
