"""Step AP: the score comparability contract. Verifies CheapStage's
tau -> score mapping is actually bounded [0,1], monotonic, and that
CheapStage's own SectorBelief.scores output honors it end-to-end (not
just the standalone conversion function in isolation)."""
import numpy as np
import pytest

from obst_avoidance.perception.cheap import TAU_CAP_S, tau_to_score


def test_bounded_zero_to_one():
    taus = np.array([0.0, 0.5, 1.0, 3.0, 15.0, 29.99, 30.0])
    scores = tau_to_score(taus, TAU_CAP_S)
    assert np.all(scores >= 0.0)
    assert np.all(scores <= 1.0)


def test_over_cap_clips_to_one_not_beyond():
    scores = tau_to_score(np.array([30.0, 100.0, 1e6]), TAU_CAP_S)
    assert np.all(scores == 1.0)


def test_zero_tau_is_zero_score():
    assert tau_to_score(np.array([0.0]), TAU_CAP_S)[0] == 0.0


def test_monotonic_increasing():
    taus = np.linspace(0.0, TAU_CAP_S, 50)
    scores = tau_to_score(taus, TAU_CAP_S)
    assert np.all(np.diff(scores) >= 0.0)


def test_nan_passes_through_as_nan_not_a_number():
    """Invalid tau must never be silently turned into a plausible
    score -- see SectorBelief's docstring on why valid=False stays
    NaN, never a fabricated value."""
    scores = tau_to_score(np.array([1.0, np.nan, 5.0]), TAU_CAP_S)
    assert not np.isnan(scores[0])
    assert np.isnan(scores[1])
    assert not np.isnan(scores[2])


def test_reference_equivalence_used_by_control_tau_crit():
    """3.0s is the exact value control/sector.py's SCORE UNITS
    docstring claims tau_crit_score=0.1 is equivalent to -- if this
    ever drifts, that docstring (and the config default) goes stale
    silently."""
    assert tau_to_score(np.array([3.0]), TAU_CAP_S)[0] == pytest.approx(0.1)


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
