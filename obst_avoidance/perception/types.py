from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class SectorBelief:
    """Common output type for both perception stages -- CheapStage today,
    HeavyStage later. Both MUST return this identical type so the
    learned gate (CheapStage vs HeavyStage, framed as learning-to-defer)
    is comparing apples to apples, not two different result shapes it
    has to reconcile itself.

    The `valid` mask is NOT optional, and is not a minor detail: R2
    found the untextured Zone B wall's centre sector had ZERO trackable
    points -- not "low confidence," genuinely no signal at all. An
    invalid sector must never be silently represented as a plausible
    number:
      - Infinite/very-large tau (== "looks completely clear") would
        make a controller steer INTO the very obstacle that defeated
        tracking, which is the single worst thing this type could do.
      - Zero/very-low tau (== "abort now") would hard-code an
        avoidance policy inside the perception stage that the gate
        (and eventually the controller) is supposed to learn, not
        inherit as an unexamined default.
    Consumers MUST check `valid[i]` before reading `scores[i]`. Where
    valid[i] is False, scores[i] is a placeholder (see CheapStage/
    cheap.py: currently NaN) and must not be used for anything --
    including comparisons, since NaN's own comparison semantics already
    make that structurally awkward on purpose.

    SCORE COMPARABILITY CONTRACT (Step AP): `scores` MUST be normalised
    to [0, 1] traversability, higher = safer, on the SAME scale
    regardless of source. This is not a suggestion -- CheapStage emits
    tau (a physical duration, seconds); HeavyStage will emit something
    derived from relative inverse depth, a completely different
    physical quantity with its own range. If either stage emitted its
    native units unconverted, the target sector picked by
    control/sector.py's argmin would jump every time the gate switched
    which stage it was reading from, and it would look and log exactly
    like controller instability -- a phantom bug hunted in the wrong
    module -- rather than what it actually is, a units mismatch at the
    producer. Every stage owns its own conversion to this scale
    internally (see CheapStage's `tau_to_score()` for the reference
    implementation and its literal [0,1] enforcement); nothing outside
    perception/ should ever need to know what physical quantity a given
    source measured to produce its scores.
    """

    scores: np.ndarray    # shape (n_sectors,) float -- per-sector traversability in [0, 1], higher = safer, SAME scale regardless of source (see the comparability contract above). Only meaningful where valid[i] is True.
    valid: np.ndarray     # shape (n_sectors,) bool -- False where too few tracked points existed to estimate this sector's score at all.
    confidence: float     # overall belief confidence in [0, 1] -- see the producing stage for how it's derived (e.g. CheapStage de-rates this on gyro_valid/odom_valid=False).
    source: str           # "cheap" | "heavy" -- which stage produced this belief.
    latency_ms: float     # wall-clock inference time for this call, milliseconds. Feeds the gate's cost side of the learning-to-defer tradeoff.

    # Optional physical estimates BEFORE within-frame score normalization.
    # Unknown stays None/NaN. These are noisy estimates, not clearance proofs.
    ttc_s: Optional[np.ndarray] = None  # LK time proxy (magnitude flow, not calibrated closing TTC)
    forward_depth_m: Optional[np.ndarray] = None  # per-sector metric camera-z depth
