import math
import cv2
import numpy as np
import pytest
from obst_avoidance.perception.visible import obstacle_spans,track_pair_balanced
from obst_avoidance.perception.types import SectorBelief
from obst_avoidance.control import SectorController,SectorControllerConfig,ControlState
from obst_avoidance.control.reference_path import ReferencePathGoal


def test_large_plain_box_is_detected_without_internal_texture():
    image=np.full((480,640,3),210,np.uint8)
    image[240:]=80
    cv2.rectangle(image,(270,180),(370,370),(150,150,150),-1)
    spans,boxes=obstacle_spans(image,205.47,320,240)
    assert any(lo<0<hi for lo,hi in spans)
    assert boxes


def test_horizon_alone_is_not_an_obstacle():
    image=np.full((480,640,3),210,np.uint8);image[240:]=80
    assert obstacle_spans(image,205.47,320,240)[0]==()


def test_balanced_tracks_follow_translation():
    rng=np.random.default_rng(17)
    gray=rng.integers(0,256,(480,640),dtype=np.uint8)
    other=cv2.warpAffine(gray,np.array([[1,0,3],[0,1,0]],np.float32),(640,480))
    a,b,_,_=track_pair_balanced(gray,other)
    assert len(a)>100
    assert np.median(b[:,0]-a[:,0])==pytest.approx(3.,abs=.15)
    assert set(range(1,10)).issubset(set((a[:,0]*11/640).astype(int)))


@pytest.mark.parametrize('sy',[-.7,0,.7])
def test_visual_policy_passes_box_in_ideal_projection(sy):
    c=SectorController(SectorControllerConfig(visible_avoidance=True,fwd_vel=.5))
    state=ControlState(None,0,None,None);g=ReferencePathGoal(10,sy,25,sy)
    x,y,yaw=10.,sy,0.;margin=100.
    for i in range(2000):
        angles=[]
        for ox,oy in ((16,-1.5),(18,-1.5),(18,1.5),(16,1.5)):
            dx,dy=ox-x,oy-y
            bx=dx*math.cos(yaw)+dy*math.sin(yaw);by=-dx*math.sin(yaw)+dy*math.cos(yaw)
            if bx>0:angles.append(math.atan2(by,bx))
        spans=((max(-1.,min(angles)),min(1.,max(angles))),) if angles and min(angles)<1 and max(angles)>-1 else ()
        b=SectorBelief(np.full(5,np.nan),np.zeros(5,bool),0.,'cheap',0.,obstacle_spans=spans)
        cmd,state=c.step(b,g.goal_heading((x,y),yaw).goal_heading_body,yaw,i*.05,state)
        yaw+=cmd.yaw_rate*.05;x+=cmd.fwd_vel*math.cos(yaw)*.05;y+=cmd.fwd_vel*math.sin(yaw)*.05
        d=math.hypot(max(abs(x-17)-1,0),max(abs(y)-1.5,0))-.35
        margin=min(margin,d)
        if math.hypot(x-25,y-sy)<.75:break
    assert math.hypot(x-25,y-sy)<.75
    assert margin>.1


@pytest.mark.parametrize('left,right,top', [(0,220,75),(220,520,0),(410,639,80)])
def test_open_box_edges_at_image_and_rotor_border(left,right,top):
    image=np.full((480,640,3),220,np.uint8);image[240:]=70
    cv2.rectangle(image,(left,top),(right,400),(140,140,140),-1)
    spans,_=obstacle_spans(image,205.47,320,240)
    center=math.atan2(320-(left+right)/2,205.47)
    assert any(lo<=center<=hi for lo,hi in spans)


def test_missing_outline_does_not_immediately_return_to_blocked_goal():
    c=SectorController(SectorControllerConfig(visible_avoidance=True,fwd_vel=.5))
    state=ControlState(None,0,None,None)
    def belief(spans):
        return SectorBelief(np.full(11,np.nan),np.zeros(11,bool),0.,'cheap',0.,obstacle_spans=spans)
    _,state=c.step(belief(((-.2,.2),)),0.,0.,1.,state)
    cmd,state=c.step(belief(()),0.,0.,1.1,state)
    assert cmd.telemetry.switch_reason=='visible_dodge'
    assert abs(cmd.telemetry.heading_error)>.5
