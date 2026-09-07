"""HeavyStage -- Stage 2 of the two-stage adaptive perception pipeline:
Depth Anything V2 Small, METRIC checkpoint (Step BW), frozen, pretrained.
We do NOT improve depth estimation -- that is explicitly not this
project's contribution. This file only wraps the model's output into
the same SectorBelief contract CheapStage produces, so the two are
comparable and a gate can defer between them.

CHECKPOINT -- METRIC, NOT RELATIVE (Step BW). Every prior HeavyStage
failure (Step AY's per-frame normalization, Step BC's rolling window,
Step BQ/BR's percentile/sector-count tuning) was a normalization
problem downstream of one root choice: the RELATIVE-depth checkpoint
has no fixed scale, so recovering anything comparable across frames
required inventing a normalization scheme, and every scheme tried
either collapsed (an obstacle filling the frame leaves no local
contrast to normalize against) or needed an obstacle to occupy an
implausibly large fraction of its own sector to be reachable at all
(Step BP). The METRIC checkpoint (`depth-anything/Depth-Anything-V2-
Metric-Outdoor-Small-hf` -- the transformers-pipeline-compatible port
of the VKITTI-trained outdoor model; the bare
"...-Metric-VKITTI-Small" name some docs use is the ORIGINAL, non-"-hf"
checkpoint format and is NOT loadable through this same
transformers.pipeline() call) outputs depth directly in METRES.
Verified explicitly against three frames of a known-ground-truth
recording before writing any code here: at true distance 6.98m, centre
raw depth ~8.6-77m; at true distance 0.32m (just before collision),
centre raw depth ~2.8-3.8m -- unambiguously LARGER=FARTHER, the
OPPOSITE of the old relative checkpoint's LARGER=CLOSER (see git
history/README Step BW for the full check). NO per-frame min-max, NO
rolling multi-frame window, NO calibration fit -- none of that
machinery exists in this file any more; a metric value from one frame
is already directly comparable to a metric value from any other frame,
which is the entire point of using this checkpoint.

SCALE -- METRIC, BUT NOT NECESSARILY ACCURATE. "Metric" means the
output is in real-world units by construction (trained to regress
actual metres), not that it is CALIBRATED for this specific sim/camera.
VKITTI is real-world outdoor driving footage; this project's obstacles
and camera are a synthetic Gazebo corridor. Step BX in README measures
the actual absolute error against ground-truth vehicle poses on
existing recordings -- read that before assuming any particular
accuracy, and do not silently assume metres-in-sim equal metres-VKITTI
without checking.

AGGREGATION -- SAME STRATEGY, FLIPPED NUMBER (Step BW keeps this per
the task: "keep p80 aggregation... as they are"). The intent preserved
is "top 20% closest pixels dominate the sector's danger reading" (Step
AI's reasoning, carried into HeavyStage at Step BP). Under the OLD
relative convention (higher=closer) that was `np.percentile(raw, 80)`
-- the boundary above which the closest 20% of values lie. Under THIS
metric convention (LOWER=closer), the same "closest 20%" boundary is
`np.percentile(raw, 20)` -- the boundary BELOW which the closest 20% of
values lie. `NEAREST_PERCENTILE` is therefore 20 here, not 80; the
number changed, the meaning did not. Getting this backwards (using 80
directly under the new sign convention) would silently aggregate the
FARTHEST 20% of a sector as if it were the nearest -- exactly the kind
of sign bug this module's docstrings have warned about since Step 0b.

Sectoring reuses geometry.sector_index() -- the SAME function CheapStage
uses, imported, not reimplemented -- so sector boundaries can never
silently drift between the two stages. geometry.N_SECTORS (11, Step BR)
is unchanged by this step.

ROW BAND -- excludes sky/ground, not tuning (Step CI/CJ). Step CE/CF/CG's
diagnosis was conclusive: edge sectors (0, 10) read as the closest
sector in the belief at EVERY distance tested, independent of any
obstacle -- traced to the FULL COLUMN aggregation pooling in sky rows
(unreliably low Z at this camera's 114.6deg HFOV, wider than whatever
camera produced this checkpoint's VKITTI training data) and ground rows
(genuinely close, but never an obstacle -- it's surface the flight path
crosses over, not into). Confirmed empirically on `boxB1_approach`:
rows below ~320 read nearly IDENTICAL Z across all 11 sectors and stay
stable (~6.7-7.2m) regardless of the vehicle's actual distance to the
box (checked at 7m, 4m, 2m out) -- a geometric ground-plane signature,
not scene content. Excluding sky/ground rows is the same move as
`cheap.py`'s `VIGNETTE_Y_MIN=100` (excludes rotor-blur rows track_pair()
cannot use) -- pixels the model demonstrably cannot estimate usefully,
not a value picked to make any particular obstacle score better.

Band derived from geometry, not literals: a 1.5m-tall obstacle centred
at the camera's own altitude (this project's convention, e.g. Zone B's
box, Step BG) subtends `2*fy*0.75/d` pixels at range d, centred on the
horizon row (`cy`, since this project's camera flies level -- Step W).
That band SHRINKS as d grows, so the band at the CLOSEST range of
interest (`ROW_BAND_MIN_RANGE_M=2.0m`, matching Step BH/BX's own
"informative window" framing, and comfortably past the already-
established <0.5m close-range breakdown region, Step AY/AZ) is the
WIDEST one and contains all the narrower bands computed at farther
range -- so it's the correct single fixed band to use, not an arbitrary
choice among several. See ROW_LO/ROW_HI below for the actual computed
values.
"""
import time

import numpy as np

from . import geometry
from .types import SectorBelief

MODEL_NAME = "depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf"
NEAREST_PERCENTILE = 20  # Step BW: metric convention (lower=closer) -- see module docstring's
                          # AGGREGATION section. This is the SIGN-FLIPPED equivalent of the old
                          # relative-checkpoint's NEAREST_PERCENTILE=80; same "top 20% closest
                          # pixels" semantic, opposite-direction percentile call.
OBSTACLE_HALF_HEIGHT_M = 0.75  # half-height of the obstacle of interest (Zone B's 1.5m box,
                                 # Step BG) -- see module docstring's ROW BAND section.
ROW_BAND_MIN_RANGE_M = 2.0  # closest range of interest for the row-band derivation -- gives
                              # the WIDEST (hence binding) row band; see module docstring.
Z_CAP_M = 20.0            # metres -- distance beyond which a sector reads as fully safe
                          # (score=1.0). First-cut choice, not empirically tuned: order-of-
                          # magnitude comparable to CheapStage's TAU_CAP_S=30s at this project's
                          # current cruise speed (0.8 m/s * 30s = 24m). Unlike tau, this is a
                          # PURE distance cap -- HeavyStage.infer() has no velocity/odom input
                          # (see infer()'s own docstring for why), so it cannot compute a
                          # time-to-contact the way CheapStage does. Whether a distance-only cap
                          # is genuinely comparable to CheapStage's time-based score under Step
                          # AP's contract has NOT been re-verified with the same rigor Step
                          # BA/BE applied after earlier scoring changes -- flagged as an open
                          # item, not silently assumed to hold.


class HeavyStage:
    """Depth Anything V2 Small, METRIC checkpoint (Step BW) -> per-sector
    SectorBelief. Model loaded once in __init__ (expensive; this class
    is meant to be constructed once per run, matching CheapStage's own
    lifecycle). No per-instance normalization state any more -- a
    metric reading needs none."""

    def __init__(self, intrinsics: dict, n_sectors: int = geometry.N_SECTORS,
                 model_name: str = MODEL_NAME, nearest_percentile: float = NEAREST_PERCENTILE,
                 z_cap_m: float = Z_CAP_M, obstacle_half_height_m: float = OBSTACLE_HALF_HEIGHT_M,
                 row_band_min_range_m: float = ROW_BAND_MIN_RANGE_M):
        from transformers import pipeline
        self._pipe = pipeline(task="depth-estimation", model=model_name)
        self.width = intrinsics["width"]
        self.height = intrinsics["height"]
        self.n_sectors = n_sectors
        self.nearest_percentile = nearest_percentile
        self.z_cap_m = z_cap_m

        # Step CJ -- row band derived from camera geometry (fy, cy), not a literal --
        # see module docstring's ROW BAND section for the full derivation.
        fy, cy = intrinsics["fy"], intrinsics["cy"]
        half_band_px = fy * obstacle_half_height_m / row_band_min_range_m
        self.row_lo = max(0, int(cy - half_band_px))
        self.row_hi = min(self.height, int(cy + half_band_px) + 1)

    def infer(self, packet) -> SectorBelief:
        """packet: FramePacket. Unlike CheapStage.infer(), this needs
        only ONE frame -- depth estimation is monocular and instantaneous,
        no flow between frames required. No `odom` parameter either:
        this stage has no forward-velocity input, so it cannot compute a
        time-to-contact the way CheapStage's tau does -- see Z_CAP_M's
        docstring for the consequence (a pure distance cap, not a time
        cap). Metric Z IS now available per-sector (unlike the old
        relative checkpoint) -- see geometry.py's module docstring and
        Step BY for how this restores metric geometry (miss_distance/
        cone_weight) for HeavyStage specifically."""
        import cv2
        from PIL import Image

        t0 = time.perf_counter()
        n = self.n_sectors

        # packet.image is documented rgb8 (frame_source/packet.py) --
        # PIL interprets an (H,W,3) uint8 array as RGB directly with no
        # conversion needed, matching how this project's own
        # tools/check_depth.py already established PIL.Image.convert("RGB")
        # composes with rgb8 with no channel-order surprises.
        image = Image.fromarray(packet.image, mode="RGB")
        result = self._pipe(image)
        raw_depth = result["predicted_depth"]
        raw = np.asarray(raw_depth, dtype=np.float64)
        if raw.ndim == 3:  # some pipeline versions return (1,H,W)/(H,W,1) -- squeeze defensively
            raw = raw.squeeze()

        # DA-V2's output resolution does not always match the input
        # image's -- resize to the camera's actual (width, height) so
        # geometry.sector_index() (which expects real pixel-column
        # coordinates) applies correctly, exactly as CheapStage's own
        # pixel-space conventions do.
        if raw.shape != (self.height, self.width):
            raw = cv2.resize(raw, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

        scores = np.full(n, np.nan)
        valid = np.zeros(n, dtype=bool)

        xs = np.arange(self.width)
        sec_idx = geometry.sector_index(xs.astype(float), self.width, n)
        for s in range(n):
            cols = np.where(sec_idx == s)[0]
            if cols.size == 0:
                continue
            # Step CJ -- row band excludes sky/ground rows this camera's wide FOV makes
            # unreliable (module docstring's ROW BAND section) -- NOT the full column any
            # more.
            sector_pixels = raw[self.row_lo:self.row_hi, cols]
            # Lower metric depth = closer = more dangerous (module docstring's AGGREGATION
            # section) -- the low percentile IS the near/dangerous reading directly, no
            # danger/1-danger flip needed the way the relative checkpoint required.
            nearest_z_m = float(np.percentile(sector_pixels, self.nearest_percentile))
            score = min(max(nearest_z_m / self.z_cap_m, 0.0), 1.0)
            scores[s] = score
            valid[s] = True

        # Step CG -- stretch to fill [0,1] across sectors WITHIN this frame (not across
        # time -- see geometry.normalize_scores_to_span()'s docstring for why that's a
        # different, safe axis). Fixes Step CE's root cause: this checkpoint's typical
        # sector-to-sector spread (~0.44-0.53) was narrower than SectorController's
        # hysteresis margin could ever clear, regardless of what the obstacle did.
        scores = geometry.normalize_scores_to_span(scores, valid)

        latency_ms = (time.perf_counter() - t0) * 1000.0
        confidence = float(np.mean(valid)) if n > 0 else 0.0

        return SectorBelief(
            scores=scores,
            valid=valid,
            confidence=confidence,
            source="heavy",
            latency_ms=latency_ms,
        )
