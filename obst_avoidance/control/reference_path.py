"""Goal-heading providers for the orchestrator (Step CTRL-COMPARE).

The SectorController is UNCHANGED and stays the baseline: it consumes a
`goal_heading` (body-frame bearing to where the vehicle "wants" to go) and
runs its VFH cost `w_tau*(1-score) + w_goal*|theta_i - goal_heading|`
exactly as before. This module only changes WHERE that goal_heading comes
from, so the two arms differ in one variable (target reference), not in
normalization, yaw gain, speed, or target-selection logic.

Two providers:

- WorldBearingGoal  (BASELINE, current behaviour): goal is a FIXED WORLD
  DIRECTION (+x corridor). goal_heading = wrap(goal_world_bearing - yaw).
  Identical to orchestrator._goal_heading_body. No reference path, so no
  cross-track objective -- a sideways detour is never pulled back.

- ReferencePathGoal (VARIANT): pure-pursuit lookahead on an EXPLICIT
  straight reference route A->B (for the first comparison, A = spawn,
  B = (goal_x, spawn_y), so the intended return path is unambiguous).
  goal_heading points at a lookahead point ON the route, so when the
  vehicle is pushed off the line by an obstacle detour, the goal bearing
  swings back toward the route and the SAME VFH argmin recovers cross
  track -- an objective the baseline structurally lacks.

Sign convention matches control/sector.py and orchestrator._quat_to_yaw:
theta / yaw ENU, positive = CCW = left. A positive goal_heading_body means
"goal is to my left" -> controller prefers left sectors -> positive
yaw_rate (turn left). Verified by test_reference_path.py.
"""
import math
from dataclasses import dataclass, field
from typing import Optional


def _wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class GoalInfo:
    goal_heading_body: float
    cross_track_error: Optional[float] = None   # signed lateral offset from the reference
                                                 # route (m); + = vehicle is LEFT of the
                                                 # A->B direction. None for the baseline
                                                 # (no reference path exists).
    lookahead_xy: Optional[tuple] = None         # world (x,y) of the pursued point, or None
    progress_m: Optional[float] = None           # arc length of the foot-point along A->B
    reference: dict = field(default_factory=dict)  # {"kind","A","B",...} for logging


class WorldBearingGoal:
    """Baseline: fixed world bearing (the +x corridor). Position-independent."""

    def __init__(self, goal_world_bearing_rad: float = 0.0):
        self.goal_world_bearing_rad = goal_world_bearing_rad

    def goal_heading(self, position, yaw: float) -> GoalInfo:
        gh = _wrap_to_pi(self.goal_world_bearing_rad - yaw)
        return GoalInfo(goal_heading_body=gh, cross_track_error=None,
                        reference={"kind": "world_bearing",
                                   "goal_world_bearing_rad": self.goal_world_bearing_rad})


class ReferencePathGoal:
    """Pure-pursuit lookahead on a straight route A->B."""

    def __init__(self, ax: float, ay: float, bx: float, by: float,
                 lookahead_m: float = 3.0):
        if not all(math.isfinite(v) for v in (ax, ay, bx, by, lookahead_m)):
            raise ValueError("reference path parameters must be finite")
        if lookahead_m <= 0:
            raise ValueError("lookahead_m must be positive")
        self.A = (float(ax), float(ay))
        self.B = (float(bx), float(by))
        seg = (self.B[0] - self.A[0], self.B[1] - self.A[1])
        self.length = math.hypot(*seg)
        if self.length < 1e-6:
            raise ValueError("reference path A and B coincide")
        self.tangent = (seg[0] / self.length, seg[1] / self.length)  # unit A->B
        self.lookahead_m = float(lookahead_m)

    def goal_heading(self, position, yaw: float) -> GoalInfo:
        # Fall back to plain tangent-following if odometry is missing this
        # frame (never fabricate a position); heads along the route direction.
        if position is None:
            gh = _wrap_to_pi(math.atan2(self.tangent[1], self.tangent[0]) - yaw)
            return GoalInfo(goal_heading_body=gh, cross_track_error=None,
                            reference=self._ref())
        px, py = position[0], position[1]
        ax, ay = self.A
        tx, ty = self.tangent
        # foot point: project P-A onto the tangent
        s = (px - ax) * tx + (py - ay) * ty          # arc length along A->B
        s_clamped = max(0.0, min(self.length, s))
        # signed cross-track: left-normal is (-ty, tx); + = vehicle left of A->B
        cte = -(px - ax) * ty + (py - ay) * tx
        # lookahead point: s_clamped + L along the route, clamped to B
        s_look = min(self.length, s_clamped + self.lookahead_m)
        lx = ax + tx * s_look
        ly = ay + ty * s_look
        desired_world = math.atan2(ly - py, lx - px)
        gh = _wrap_to_pi(desired_world - yaw)
        return GoalInfo(goal_heading_body=gh, cross_track_error=cte,
                        lookahead_xy=(lx, ly), progress_m=s_clamped,
                        reference=self._ref())

    def _ref(self):
        return {"kind": "reference_path", "A": list(self.A), "B": list(self.B),
                "lookahead_m": self.lookahead_m}
