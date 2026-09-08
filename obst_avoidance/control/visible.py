"""Explicit detection-only policy for a large visible-obstacle demonstration.

Absence of a silhouette is NOT certified free space. This policy deliberately
supports a narrower task than navigation through unknown/unobservable hazards.
"""
import math
from .types import ControlCommand,ControlState,ControlTelemetry


def step_visible(belief,goal,yaw,t,state,cfg):
    wrap=lambda a:(a+math.pi)%(2*math.pi)-math.pi
    dt=max(0.,t-state.last_t) if state.last_t is not None else 0.
    spans=belief.obstacle_spans
    if spans is None:
        return ControlCommand(0.,0.,'blind',None),ControlState(None,0.,belief.source,t)
    world_spans=state.visible_world_spans
    last_seen=state.visible_last_seen
    if spans:
        world_spans=tuple((yaw+lo,yaw+hi) for lo,hi in spans)
        last_seen=t
    elif last_seen is not None and 0 <= t-last_seen <= 2.:
        spans=tuple((wrap(lo-yaw),wrap(hi-yaw)) for lo,hi in world_spans
                    if abs(wrap((lo+hi)/2-yaw)) < 1.5)
    else:
        world_spans=()
    padding=cfg.visible_padding_rad
    blocked=[(lo-padding,hi+padding) for lo,hi in spans]
    within=lambda a:any(lo<=a<=hi for lo,hi in blocked)
    limit=.9
    target=max(-limit,min(limit,goal))
    reason='visible_goal'
    if abs(goal)>limit:
        target=goal
        speed=0.
        reason='visible_reorient'
    else:
        speed=cfg.fwd_vel
        if within(target):
            choices=[a for lo,hi in blocked for a in (lo-.02,hi+.02) if -limit<=a<=limit and not within(a)]
            if not choices:
                return ControlCommand(0.,0.,'blind',None),ControlState(None,0.,belief.source,t)
            held=wrap(state.target_world_bearing-yaw) if state.target_world_bearing is not None else 0.
            # Mild continuity cost prevents swapping obstacle sides on a tie.
            target=min(choices,key=lambda a:abs(a-goal)+.35*abs(a-held))
            reason='visible_dodge'
    rate=max(-.6,min(.6,1.5*target))
    previous=state.previous_yaw_rate if dt>0 else 0.
    rate=max(previous-dt,min(previous+dt,rate))
    if rate*target<0:speed=0.
    speed*=max(.2,math.cos(target))
    world=wrap(yaw+target)
    cmd=ControlCommand(speed,rate,'avoid' if spans or reason=='visible_reorient' else 'cruise',None,
        ControlTelemetry({},None,None,world,target,reason,True,None))
    return cmd,ControlState(world,0.,belief.source,t,previous_yaw_rate=rate,
        visible_world_spans=world_spans,visible_last_seen=last_seen)
