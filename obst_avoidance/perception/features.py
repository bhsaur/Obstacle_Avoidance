"""Feature-vector layout for CheapStage's second infer() return value --
the ONE place its indices and meanings are documented. FEATURE_SPEC is
the single source of truth: extract_features() builds the vector
mechanically from it (via a name-keyed dict, so a missing key raises
KeyError immediately rather than silently shifting every later index --
this can't drift out of sync with FEATURE_SPEC the way a hand-written
parallel list could).

Every entry corresponds to a documented ask in the CheapStage Phase 2
task (see obst_avoidance/README.md's "Pre-CheapStage fixes" section for
the empirical findings these features exist to expose -- O2's sector
reliability finding, R2's ground-contamination and untextured-wall
findings, Step U/V's confirmed mappings). Two ambiguous cases in the
original spec are resolved explicitly below, not silently:

  - "temporal derivatives: d(feature count)/dt, d(tau)/dt" is
    implemented PER-SECTOR (5+5 values) here, distinct from the later
    "global feature count + its rate of change" (2 scalar values) --
    the spec calls the second one "global" explicitly, implying the
    first is not.
  - "across-sector tau variance" appears twice in the spec: once
    implicitly under "min tau, tau variance" (paired with per-sector
    tau/valid, i.e. variance across all 5 sectors) and once explicitly
    named under the "global" bullet, flagged there specifically for its
    early-wide-wall-signal value. Both are currently the SAME computed
    quantity (variance of the 5 raw per-sector tau values) --
    tau_variance and across_sector_tau_variance are intentionally
    duplicated for now so the feature vector's structure matches the
    spec's literal bullet list one-to-one and is easy to audit against
    it. If a different variance definition was actually intended for
    one of the two (e.g. population vs. sample variance, or one of them
    meant to be the SMOOTHED-tau variance rather than raw), that's a
    one-line fix in cheap.py's infer() -- flagged here so it isn't lost.

Ground-truth units: tau in seconds, flow magnitude in px, gradient
energy in px (Sobel magnitude units), feature counts as integers (cast
to float for a uniform dtype), fractions/ratios/valid/gyro_valid/
odom_valid as 0.0/1.0 floats, cone_effective_count as a float (it's a
soft/weighted count, sum of cone weights in (0,1], not an integer).
Every "undefined" value (too few points, no previous frame to diff
against, etc.) is NaN, propagating the same "never fabricate a number"
rule SectorBelief.valid enforces -- a consumer that averages a feature
column without checking for NaN will find out immediately, not get a
silently wrong answer.
"""
from typing import Dict, List, Tuple

import numpy as np

from .geometry import N_SECTORS


def _per_sector_spec(name_fmt: str, desc_fmt: str, n_sectors: int) -> List[Tuple[str, str]]:
    return [(name_fmt.format(i), desc_fmt.format(i)) for i in range(n_sectors)]


def build_feature_spec(n_sectors: int = N_SECTORS) -> List[Tuple[str, str]]:
    """Returns the ordered (name, description) list that defines the
    feature vector's layout. A function rather than a bare module
    constant so it stays correct if n_sectors is ever configured
    differently than the default -- CheapStage must be constructed with
    the SAME n_sectors it later calls extract_features() with, or the
    dict-driven construction below will KeyError rather than silently
    misalign, which is the point."""
    spec: List[Tuple[str, str]] = []
    spec += _per_sector_spec(
        "tau_sector_{0}", "sector {0}: EMA-smoothed, tau-capped time-to-contact (s); "
        "meaningless if valid_sector_{0}=0", n_sectors)
    spec += _per_sector_spec(
        "valid_sector_{0}", "sector {0}: 1.0 if >= CheapStage.min_points_per_sector "
        "tracked points AND at least one had a defined (expanding) tau, else 0.0", n_sectors)
    spec += [
        ("min_tau", "min tau over sectors with valid_sector_i=1; NaN if none valid"),
        ("tau_variance", "variance of the (raw, pre-smoothing) tau values across all "
         "valid sectors; NaN if fewer than 2 valid"),
        ("centre_mean_tau", "mean raw tau over centre sectors (indices 1..n_sectors-2) "
         "that are valid; NaN if none valid"),
        ("centre_min_tau", "min raw tau over the same centre sectors; NaN if none valid"),
        ("cone_min_tau", "min per-point tau among points with cone_weight > "
         "CheapStage.CONE_IN_THRESHOLD (i.e. miss < r_safe); NaN if no such point had a "
         "defined tau"),
        ("cone_effective_count", "sum of per-point cone_weight over ALL tracked points "
         "this frame (a soft/weighted count, not an integer point count)"),
        ("cone_density_ratio", "cone_effective_count normalized to an assumed cone pixel "
         "area (CheapStage.NOMINAL_CONE_AREA_FRACTION of the frame -- a documented "
         "placeholder, not empirically derived, see cheap.py), divided by the full-frame "
         "point density; >1 means tracked points are concentrated toward the flight path "
         "relative to the frame average, <1 means sparse there"),
    ]
    spec += _per_sector_spec(
        "count_sector_{0}", "sector {0}: number of tracked (post-outlier-rejection) points", n_sectors)
    spec += _per_sector_spec(
        "flow_mag_sector_{0}", "sector {0}: mean residual (de-rotated, translational) flow "
        "magnitude, px; NaN if count_sector_{0}=0", n_sectors)
    spec += _per_sector_spec(
        "ground_frac_sector_{0}", "sector {0}: fraction of tracked points with pixel row "
        ">= cy (below the horizon, i.e. plausibly ground-plane) -- see Step R2; NOT masked "
        "out anywhere in this pipeline, fed to the gate as a distrust signal instead. NaN "
        "if count_sector_{0}=0", n_sectors)
    spec += _per_sector_spec(
        "count_rate_sector_{0}", "sector {0}: d(count_sector_{0})/dt vs. the previous "
        "infer() call, points/s; 0.0 on the first call (no previous frame to diff against)", n_sectors)
    spec += _per_sector_spec(
        "tau_rate_sector_{0}", "sector {0}: d(tau)/dt vs. the previous call's raw tau for "
        "this sector, s/s; NaN if either this or the previous call had this sector invalid, "
        "or on the first call", n_sectors)
    spec += _per_sector_spec(
        "grad_energy_sector_{0}", "sector {0}: mean Sobel gradient magnitude over the "
        "WHOLE sector image region (not just at tracked points) -- deliberately independent "
        "of corner count; see the module docstring, sky vs. wall discrimination", n_sectors)
    spec += [
        ("global_feature_count", "total tracked points this frame, across all sectors"),
        ("global_feature_count_rate", "d(global_feature_count)/dt vs. the previous call, "
         "points/s; 0.0 on the first call"),
        ("across_sector_tau_variance", "SAME quantity as tau_variance above -- duplicated "
         "deliberately, see module docstring"),
        ("lk_residual", "median LK tracking error (px) across all points surviving "
         "track_pair()'s three-stage cleaning this frame; NaN if zero points survived"),
        ("frac_features_lost", "1 - (points surviving all three cleaning stages) / "
         "(raw corner count from goodFeaturesToTrack); NaN if goodFeaturesToTrack itself "
         "found nothing (track_pair returned None before n_initial was even meaningful)"),
        ("gyro_valid", "1.0 if packet.gyro_valid, else 0.0 -- see frame_source's "
         "gyro-staleness handling"),
        ("odom_valid", "1.0 if the odom tuple's odom_valid flag was True, else 0.0 -- see "
         "Step V's odometry interpolation"),
    ]
    # Step DI/DJ -- contour/looming channel, APPENDED so indices 0..(pre-DI end)
    # are unchanged (the pre-DI 54-element layout is a stable prefix). These are
    # the SECOND, LK-free TTC source (perception/contours.py) merged per sector.
    # All NaN/0.0 when the contour channel is disabled or finds no region in a
    # sector -- same "never fabricate a number" NaN convention as the LK features.
    spec += _per_sector_spec(
        "tau_area_sector_{0}", "sector {0}: contour looming TTC = 2A/(dA/dt) from the "
        "silhouette region whose centroid is in this sector (MIN over regions if several); "
        "NaN if no region there is growing. LK-free (Step DI). Step DN: INFORMATIONAL ONLY "
        "-- unreliable (tracker identity swaps, DM) so it does NOT enter SectorBelief.scores; "
        "kept as a gate feature.", n_sectors)
    spec += _per_sector_spec(
        "contour_region_count_sector_{0}", "sector {0}: number of tracked silhouette regions "
        "whose centroid falls in this sector this frame", n_sectors)
    spec += _per_sector_spec(
        "contour_total_area_sector_{0}", "sector {0}: summed silhouette-region area (px^2) in "
        "this sector; 0.0 if none", n_sectors)
    spec += _per_sector_spec(
        "contour_growth_sector_{0}", "sector {0}: dA/dt (px^2/s, window line-fit slope) of the "
        "LARGEST region in this sector; NaN if too few in-window samples to fit", n_sectors)
    spec += _per_sector_spec(
        "ttc_source_sector_{0}", "sector {0}: TTC source flag -- 0 neither, 1 LK-only, "
        "2 contour-only == 'detected, unmeasured' (Step DN: belief stays valid=False, "
        "controller avoids as UNKNOWN), 3 both present (belief LK-measured, contour "
        "corroborates). Only LK backs the belief after DN; this flag tells the gate whether "
        "a silhouette was seen even where LK could not measure it.", n_sectors)
    spec += _per_sector_spec(
        "tau_agreement_sector_{0}", "sector {0}: agreement between the two TTC sources where "
        "BOTH are valid = min(tau_lk,tau_area)/max(tau_lk,tau_area) in (0,1] (1=agree); NaN "
        "unless both valid. Disagreement is itself an unreliability signal (Step DJ).", n_sectors)
    return spec


def feature_names(n_sectors: int = N_SECTORS) -> List[str]:
    return [name for name, _ in build_feature_spec(n_sectors)]


def extract_features(computed: Dict[str, float], n_sectors: int = N_SECTORS) -> np.ndarray:
    """Builds the feature vector from a name-keyed dict (produced by
    CheapStage.infer()) in exactly the order build_feature_spec()
    documents. Raises KeyError if `computed` is missing any expected
    key -- deliberately, so a mismatch between CheapStage's internals
    and this module's spec fails loudly at the point it happens rather
    than producing a vector whose indices don't mean what a consumer
    thinks they mean."""
    spec = build_feature_spec(n_sectors)
    return np.array([float(computed[name]) for name, _ in spec], dtype=np.float64)
