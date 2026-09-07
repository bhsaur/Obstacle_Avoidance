from dataclasses import dataclass

import numpy as np


@dataclass
class FramePacket:
    """One synchronized (image, gyro) sample. No processing applied to
    either field. FramePacket carries no encoding metadata of its own --
    image is whatever encoding the PRODUCING SOURCE documents itself as
    using, and callers must check that source's docstring, not assume.
    For SimFrameSource specifically: rgb8, i.e. channel order is R,G,B --
    NOT BGR. (OpenCV's imwrite/imshow/most IO expects BGR; convert
    explicitly if you need it, this class never does.) gyro is raw
    angular rate in whatever frame the source's gyro uses (see the
    source's docstring; SimFrameSource's is body-frame NED, rad/s)."""

    image: np.ndarray
    gyro: np.ndarray  # shape (3,): [roll_rate, pitch_rate, yaw_rate]
    t_capture: float  # seconds, source-defined epoch (see source docstring)
    seq: int
    # False when the source could not pair a genuinely fresh gyro sample
    # with this frame (no sample yet, or nearest one is stale -- see the
    # source's staleness threshold). gyro is still populated (zeros) in
    # that case so downstream code doesn't need a None-check, but must
    # not silently treat zeros as "not rotating" -- check this flag first.
    gyro_valid: bool = True
