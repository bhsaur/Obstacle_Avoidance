"""Large silhouette detections, not depth or a free-space certificate."""
import math
import cv2
import numpy as np


def obstacle_spans(image, fx, cx, cy):
    """Angular bounds of large closed shapes intersecting the flight horizon.

    Uses image contrast only. No object colour, map, distance or contour TTC.
    Intended for the explicit basic-visible-obstacle experiment.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    h, w = gray.shape
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 25, 75)
    edges[:int(.12*h)] = 0  # camera/rotor border
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
    contours,_ = cv2.findContours(closed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    boxes=[]
    for c in contours:
        x,y,bw,bh=cv2.boundingRect(c)
        area=cv2.contourArea(c)
        # Reject texture specks, horizon/ground boundaries, and distant details.
        if bw<.035*w or bh<.08*h or area<.002*w*h or area<.22*bw*bh:
            continue
        if bw>.85*w or bh>.9*h:
            continue
        # Tall basic obstacles crossing cruise height, with limited pitch slack.
        if y>cy+.10*h or y+bh<cy-.10*h:
            continue
        boxes.append((x,y,bw,bh))
    # Open silhouettes at the image/rotor boundary have no enclosed contour.
    # Long upright edges crossing the horizon still provide angular occupancy.
    upright = edges.copy()
    upright[:int(.22*h)] = 0
    lines = cv2.HoughLinesP(upright, 1, np.pi/180, 35,
                           minLineLength=int(.18*h), maxLineGap=12)
    sides = []
    if lines is not None:
        for x0,y0,x1,y1 in lines[:,0]:
            if abs(y1-y0) < .18*h or abs(x1-x0) > .3*abs(y1-y0):
                continue
            if min(y0,y1)>cy+.1*h or max(y0,y1)<cy-.1*h:
                continue
            sides.append((int((x0+x1)/2), min(y0,y1), max(y0,y1)))
    if sides:
        left, right = min(p[0] for p in sides), max(p[0] for p in sides)
        if right-left < .035*w:
            # One clipped side: conservatively occupy the nearer image edge.
            if (left+right)/2 < cx: left=0
            else: right=w-1
        top,bottom=min(p[1] for p in sides),max(p[2] for p in sides)
        boxes.append((left,top,right-left+1,bottom-top+1))
    intervals=sorted((math.atan2(cx-(x+bw),fx),math.atan2(cx-x,fx)) for x,y,bw,bh in boxes)
    merged=[]
    for lo,hi in intervals:
        if merged and lo<=merged[-1][1]: merged[-1]=(merged[-1][0],max(hi,merged[-1][1]))
        else: merged.append((lo,hi))
    return tuple(merged),boxes


def track_pair_balanced(gray0,gray1,n_sectors=11):
    """Spatial corner quotas and forward/backward LK consistency.

    Keeps fast foreground flow when it is track-consistent. Does not remove
    the globally fastest tracks, which may belong to the approaching obstacle.
    """
    from .cheap import LK_PARAMS
    h,w=gray0.shape
    points=[]
    for x0,x1 in zip(np.linspace(0,w,n_sectors+1,dtype=int)[:-1],np.linspace(0,w,n_sectors+1,dtype=int)[1:]):
        for y0,y1 in zip(np.linspace(101,h,4,dtype=int)[:-1],np.linspace(101,h,4,dtype=int)[1:]):
            p=cv2.goodFeaturesToTrack(gray0[y0:y1,x0:x1],maxCorners=12,qualityLevel=.01,minDistance=4,blockSize=5)
            if p is not None:
                p[:,:,0]+=x0;p[:,:,1]+=y0;points.append(p)
    if not points:return None
    p0=np.concatenate(points).astype(np.float32)
    p1,ok1,err=cv2.calcOpticalFlowPyrLK(gray0,gray1,p0,None,**LK_PARAMS)
    if p1 is None:return None
    back,ok2,_=cv2.calcOpticalFlowPyrLK(gray1,gray0,p1,None,**LK_PARAMS)
    if back is None:return None
    a,b=p0.reshape(-1,2),p1.reshape(-1,2)
    keep=(ok1.ravel()>0)&(ok2.ravel()>0)&(np.linalg.norm(back.reshape(-1,2)-a,axis=1)<1.)
    keep &= np.isfinite(b).all(axis=1)&(b[:,0]>=0)&(b[:,0]<w)&(b[:,1]>100)&(b[:,1]<h)
    if keep.sum()<6:return None
    return a[keep],b[keep],err.ravel()[keep],len(a)
