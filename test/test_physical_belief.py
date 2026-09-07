"""Physical estimate contract tested without loading a depth checkpoint."""
from types import SimpleNamespace
import numpy as np
from obst_avoidance.perception.heavy import HeavyStage


def test_metric_depth_survives_score_normalization_in_separate_field():
    stage = HeavyStage.__new__(HeavyStage)
    stage.width, stage.height, stage.n_sectors = 4, 2, 2
    stage.row_lo, stage.row_hi = 0, 2
    stage.nearest_percentile, stage.z_cap_m = 20, 24
    stage._pipe = lambda image: {'predicted_depth': np.array([[2.,2.,8.,8.]] * 2)}
    b = stage.infer(SimpleNamespace(image=np.zeros((2,4,3),dtype=np.uint8)))
    np.testing.assert_allclose(b.scores,[0.,1.])
    np.testing.assert_allclose(b.forward_depth_m,[2.,8.])
    assert b.ttc_s is None
