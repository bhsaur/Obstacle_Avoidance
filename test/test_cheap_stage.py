"""Regression checks for CheapStage timing and belief/feature contracts."""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from obst_avoidance.perception import cheap
from obst_avoidance.perception.contours import ContourLoomingChannel
from obst_avoidance.perception.features import feature_names
from obst_avoidance.perception.geometry import sector_index

INTR = dict(width=640, height=480, fx=205.47, fy=205.47, cx=320, cy=240)
ODOM = ((0, 0, 3), (1, 0, 0), (0, 0, 0, 1), True)


def packet(t, gyro_valid=True):
    return SimpleNamespace(image=np.zeros((480, 640, 3), np.uint8),
                           t_capture=t, seq=int(t * 100), gyro=np.zeros(3), gyro_valid=gyro_valid)


def tracks(count, dt, tau):
    p0 = np.column_stack((np.linspace(360, 420, count), np.full(count, 280.0)))
    p1 = p0 + (p0 - [320, 240]) * dt / tau
    return p0, p1, np.zeros(count), count


def as_dict(features, n=1):
    return dict(zip(feature_names(n), features))


def test_derivatives_use_elapsed_inference_time(monkeypatch):
    calls = iter([tracks(10, 0.1, 5), tracks(20, 0.1, 3)])
    monkeypatch.setattr(cheap, 'track_pair', lambda *args: next(calls))
    stage = cheap.CheapStage(INTR, n_sectors=1, use_contours=False)
    stage.infer(packet(1), packet(0.9), ODOM)
    _, features = stage.infer(packet(2), packet(1.9), ODOM)
    values = as_dict(features)
    assert values['count_rate_sector_0'] == pytest.approx(10)
    assert values['global_feature_count_rate'] == pytest.approx(10)
    assert values['tau_rate_sector_0'] == pytest.approx(-2)


def test_reversed_time_clears_old_smoothing(monkeypatch):
    calls = iter([tracks(10, 0.1, 20), tracks(10, 0.1, 2)])
    monkeypatch.setattr(cheap, 'track_pair', lambda *args: next(calls))
    stage = cheap.CheapStage(INTR, n_sectors=1, use_contours=False)
    stage.infer(packet(5), packet(4.9), ODOM)
    _, features = stage.infer(packet(1), packet(0.9), ODOM)
    assert as_dict(features)['tau_sector_0'] == pytest.approx(2)


def test_duplicate_pair_never_runs_contour_fit(monkeypatch):
    stage = cheap.CheapStage(INTR)
    monkeypatch.setattr(stage._contours, 'update', lambda *args: pytest.fail('duplicate passed to contours'))
    belief, features = stage.infer(packet(1), packet(1), ODOM)
    assert not belief.valid.any()
    assert np.isnan(belief.scores).all()
    assert len(features) == 168


def test_contour_repeated_timestamp_resets_fit(monkeypatch):
    channel = ContourLoomingChannel(INTR)
    monkeypatch.setattr(channel, '_detect', lambda gray: [(320, 240, 5000)])
    for _ in range(10):
        regions = channel.update(np.zeros((480, 640), np.uint8), 1)
        assert np.isnan(regions[0].growth_rate)


def test_previous_gyro_validity_affects_confidence(monkeypatch):
    monkeypatch.setattr(cheap, 'track_pair', lambda *args: tracks(10, 0.1, 5))
    stage = cheap.CheapStage(INTR, n_sectors=1, use_contours=False)
    belief, _ = stage.infer(packet(1), packet(0.9, gyro_valid=False), ODOM)
    assert belief.valid[0]
    assert belief.confidence == pytest.approx(0.5)


def test_gradient_sector_boundaries_match_geometry():
    gray = np.random.default_rng(42).integers(0, 256, (20, 13), dtype=np.uint8)
    gradient = cv2.magnitude(cv2.Sobel(gray, cv2.CV_32F, 1, 0), cv2.Sobel(gray, cv2.CV_32F, 0, 1))
    sectors = sector_index(np.arange(13, dtype=float), 13, 5)
    expected = [gradient[:, sectors == i].mean() for i in range(5)]
    np.testing.assert_allclose(cheap._gradient_energy_per_sector(gray, 5), expected, rtol=1e-6)


def test_contours_do_not_change_belief(monkeypatch):
    monkeypatch.setattr(cheap, 'track_pair', lambda *args: tracks(12, 0.1, 3))
    plain = cheap.CheapStage(INTR, n_sectors=1, use_contours=False)
    contours = cheap.CheapStage(INTR, n_sectors=1, use_contours=True)
    for t in (1, 1.1, 1.2):
        a, _ = plain.infer(packet(t), packet(t - 0.1), ODOM)
        b, _ = contours.infer(packet(t), packet(t - 0.1), ODOM)
        np.testing.assert_array_equal(a.valid, b.valid)
        np.testing.assert_array_equal(a.scores, b.scores)
        assert np.all((a.scores[a.valid] >= 0) & (a.scores[a.valid] <= 1))
