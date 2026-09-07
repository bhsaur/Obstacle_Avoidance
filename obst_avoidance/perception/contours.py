"""Contour / silhouette looming channel (Step DI) -- a SECOND, LK-free
time-to-contact source for CheapStage.

Motivation: LK optical flow needs interior texture to place and track
points. On an untextured surface filling the view (Zone B's box) LK
gets ~0 points and the sector goes blind (README, Phase 5 / Step DK).
But an untextured object still has a SILHOUETTE against the background,
and that silhouette GROWS as the camera approaches -- the published
"growth in feature/region size" looming cue, which needs no interior
texture at all. This module extracts closed silhouette regions, tracks
each region's AREA over a short window, and turns area-growth into a
looming time-to-contact:

    tau_area = 2 * A / (dA/dt)

The factor 2 is geometric, not a fudge: a frontal surface projects with
area A proportional to 1/z^2, so (dA/dt)/A = -2 (dz/dt)/z = 2/tau_z,
giving tau_area = tau_z (true time-to-contact) when approaching.

ROBUSTNESS (learned the hard way from the LK tau estimator, README's
"tau estimator fix"): raw frame-to-frame dA/dt is garbage -- blob area
is noisy. dA/dt here is the SLOPE OF A LINE FIT to (t, area) over a
short window, never a consecutive difference. A region needs
MIN_WINDOW_POINTS samples before any tau is emitted.

This module is an ADD-ON. It never touches the LK path; CheapStage
merges its per-sector output alongside the LK belief (Step DJ). It
holds cross-frame state (region tracks), so one instance per CheapStage
instance, updated once per infer() call.
"""
from dataclasses import dataclass, field
from typing import List, Optional

import cv2
import numpy as np

from . import geometry


# -- detection / tracking tunables (first-cut, documented; not swept) --
BLUR_KSIZE = 5             # Gaussian blur before edge detection -- suppress rotor/sensor noise
CANNY_LO = 40
CANNY_HI = 120
CLOSE_KSIZE = 7           # morphological close: connect edge fragments into CLOSED boundaries
                          # so findContours returns filled regions, not edge pixels (Step DI:
                          # "not Canny alone -- we want closed regions").
MIN_AREA_FRAC = 0.002     # regions smaller than this fraction of the frame are noise/texture specks
MAX_AREA_FRAC = 0.60      # regions larger than this ARE the world (ground+sky fill), not an obstacle
WINDOW_SECONDS = 0.6      # area-history window for the slope fit
MIN_WINDOW_POINTS = 4     # need at least this many (t, area) samples in-window to emit a tau
MATCH_MAX_CENTROID_FRAC = 0.15  # region identity: centroid must be within this frac of width frame-to-frame
MATCH_MAX_AREA_RATIO = 3.0      # ...and area within this ratio, to be the SAME region
TAU_AREA_CAP_S = 30.0     # cap, matching CheapStage.TAU_CAP_S, so a barely-growing region doesn't
                          # emit an absurd finite tau; growth below the implied rate -> invalid


@dataclass
class Region:
    """One tracked silhouette region's CURRENT-frame observation."""
    region_id: int
    sector: int
    cx: float
    cy: float
    area: float
    growth_rate: float          # dA/dt from the window line fit (px^2/s); nan if too few samples
    tau_area: float             # 2A/(dA/dt) if growing; nan/inf otherwise


@dataclass
class _Track:
    region_id: int
    ts: List[float] = field(default_factory=list)
    areas: List[float] = field(default_factory=list)
    cx: float = 0.0
    cy: float = 0.0
    last_t: float = 0.0


class ContourLoomingChannel:
    def __init__(self, intrinsics: dict, n_sectors: int = geometry.N_SECTORS,
                 blur_ksize: int = BLUR_KSIZE, canny_lo: int = CANNY_LO, canny_hi: int = CANNY_HI,
                 close_ksize: int = CLOSE_KSIZE, min_area_frac: float = MIN_AREA_FRAC,
                 max_area_frac: float = MAX_AREA_FRAC, window_seconds: float = WINDOW_SECONDS,
                 min_window_points: int = MIN_WINDOW_POINTS, tau_area_cap_s: float = TAU_AREA_CAP_S):
        self.width = int(intrinsics["width"])
        self.height = int(intrinsics["height"])
        self.n_sectors = n_sectors
        self.blur_ksize = blur_ksize
        self.canny_lo = canny_lo
        self.canny_hi = canny_hi
        self.close_ksize = close_ksize
        self.frame_area = float(self.width * self.height)
        self.min_area = min_area_frac * self.frame_area
        self.max_area = max_area_frac * self.frame_area
        self.window_seconds = window_seconds
        self.min_window_points = min_window_points
        self.tau_area_cap_s = tau_area_cap_s
        self._tracks: List[_Track] = []
        self._next_id = 0
        self._last_t = None

    def reset(self):
        """Start a new timestamp epoch without carrying area-growth history."""
        self._tracks = []
        self._next_id = 0
        self._last_t = None

    def _detect(self, gray: np.ndarray):
        """Returns list of (cx, cy, area) for closed regions passing the
        area filter."""
        k = self.blur_ksize
        blurred = cv2.GaussianBlur(gray, (k, k), 0)
        edges = cv2.Canny(blurred, self.canny_lo, self.canny_hi)
        ck = self.close_ksize
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck, ck))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area or area > self.max_area:
                continue
            M = cv2.moments(c)
            if M["m00"] <= 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            out.append((cx, cy, area))
        return out

    def _match(self, detections, t):
        """Greedy nearest-centroid + area-ratio match of detections to
        existing tracks. Returns list of (track, cx, cy, area) and
        starts new tracks for unmatched detections. Prunes stale
        tracks."""
        max_cd = MATCH_MAX_CENTROID_FRAC * self.width
        used = set()
        matched = []
        for (cx, cy, area) in detections:
            best = None
            best_d = None
            for tr in self._tracks:
                if id(tr) in used:
                    continue
                d = np.hypot(cx - tr.cx, cy - tr.cy)
                if d > max_cd:
                    continue
                prev_area = tr.areas[-1] if tr.areas else area
                ratio = max(area, prev_area) / max(1.0, min(area, prev_area))
                if ratio > MATCH_MAX_AREA_RATIO:
                    continue
                if best_d is None or d < best_d:
                    best_d = d
                    best = tr
            if best is None:
                best = _Track(region_id=self._next_id)
                self._next_id += 1
                self._tracks.append(best)
            used.add(id(best))
            matched.append((best, cx, cy, area))
        # prune tracks not updated this frame if they've gone stale
        self._tracks = [tr for tr in self._tracks
                        if id(tr) in used or (t - tr.last_t) <= self.window_seconds]
        return matched

    def update(self, gray: np.ndarray, t_capture: float) -> List[Region]:
        """Detect + track + emit per-region looming observations for THIS
        frame. gray: single-channel uint8 image (caller converts; keeps
        this module from assuming a colour order)."""
        if not np.isfinite(t_capture):
            raise ValueError("contour timestamp must be finite")
        if self._last_t is not None and t_capture <= self._last_t:
            self.reset()
        self._last_t = t_capture
        detections = self._detect(gray)
        matched = self._match(detections, t_capture)
        regions: List[Region] = []
        for tr, cx, cy, area in matched:
            tr.cx, tr.cy, tr.last_t = cx, cy, t_capture
            tr.ts.append(t_capture)
            tr.areas.append(area)
            # keep only the in-window history
            while len(tr.ts) > 1 and (t_capture - tr.ts[0]) > self.window_seconds:
                tr.ts.pop(0)
                tr.areas.pop(0)

            growth = np.nan
            tau_area = np.nan
            if len(tr.ts) >= self.min_window_points:
                ts = np.array(tr.ts)
                areas = np.array(tr.areas)
                # slope of a line fit to (t, area) -- robust dA/dt, NOT a
                # consecutive difference (see module docstring).
                slope = float(np.polyfit(ts - ts[0], areas, 1)[0])
                growth = slope
                # looming tau only when the region is GROWING (approaching).
                # tau below what TAU_AREA_CAP_S implies -> treat as invalid
                # (barely growing / receding / pure rotation).
                min_slope = 2.0 * area / self.tau_area_cap_s
                if slope > min_slope:
                    tau_area = 2.0 * area / slope
            sector = int(geometry.sector_index(np.array([cx]), self.width, self.n_sectors)[0])
            regions.append(Region(region_id=tr.region_id, sector=sector, cx=cx, cy=cy,
                                  area=area, growth_rate=growth, tau_area=tau_area))
        return regions

    def aggregate_per_sector(self, regions: List[Region]):
        """Reduce per-region observations to per-sector arrays CheapStage
        merges into its belief/features. Returns a dict of length-n_sectors
        arrays:
          tau_area          -- MIN tau_area over valid regions in the sector (nearest wins);
                               NaN if no valid looming region there
          region_count      -- number of tracked regions whose centroid is in the sector
          total_area        -- summed region area in the sector (px^2)
          largest_growth    -- growth_rate of the largest-area region in the sector (px^2/s)
        """
        n = self.n_sectors
        tau_area = np.full(n, np.nan)
        region_count = np.zeros(n)
        total_area = np.zeros(n)
        largest_area = np.full(n, -1.0)
        largest_growth = np.full(n, np.nan)
        for r in regions:
            s = r.sector
            region_count[s] += 1
            total_area[s] += r.area
            if r.area > largest_area[s]:
                largest_area[s] = r.area
                largest_growth[s] = r.growth_rate
            if not np.isnan(r.tau_area):
                if np.isnan(tau_area[s]) or r.tau_area < tau_area[s]:
                    tau_area[s] = r.tau_area
        return {
            "tau_area": tau_area,
            "region_count": region_count,
            "total_area": total_area,
            "largest_growth": largest_growth,
        }
