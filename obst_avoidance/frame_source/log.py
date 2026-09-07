import json
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from .base import FrameSource
from .packet import FramePacket


class LogFrameSource(FrameSource):
    """FrameSource that replays a recording made by record_pass.py --
    frames/*.png plus packets.jsonl -- instead of a live sim connection.
    Same read() contract as SimFrameSource: FramePacket, or None (here,
    meaning the recording is exhausted rather than "no new frame yet").

    gyro_valid is READ FROM packets.jsonl, never recomputed -- whatever
    SimFrameSource decided at record time (nearest-sample staleness,
    interpolation bracket, etc.) is what a replay must reproduce. This
    class has no gyro deque, no staleness threshold, nothing to
    recompute from.

    dropped_frames is always 0: replay reads every frame in the log by
    construction, there's no live transport to lose frames in.

    Array identity (this is the part that had to be gotten right, not
    assumed correct): record_pass.py converts SimFrameSource's rgb8
    array to BGR before cv2.imwrite (cv2.imwrite assumes BGR). Loading
    with cv2.imread also returns BGR, and PNG is lossless -- so this
    class converts BGR back to RGB on load, undoing exactly the
    conversion record_pass.py applied. Net effect: the array a caller
    gets from LogFrameSource.read() is byte-identical to the array
    SimFrameSource.read() produced for that frame at record time (same
    dtype uint8, same shape, same rgb8 channel order, same values) --
    verified explicitly in test/test_log_frame_source_identity.py, not
    just assumed from the theory above.
    """

    def __init__(self, recording_dir: str, realtime: bool = False):
        self._dir = Path(recording_dir)
        self._frames_dir = self._dir / "frames"

        packets = []
        with open(self._dir / "packets.jsonl") as f:
            for line in f:
                line = line.strip()
                if line:
                    packets.append(json.loads(line))
        # packets.jsonl is written in capture order already, but sort by
        # seq explicitly rather than assuming file order is trustworthy.
        packets.sort(key=lambda row: row["seq"])
        self._packets = packets
        self._idx = 0

        intrinsics_path = self._dir / "intrinsics.json"
        self._intrinsics = json.loads(intrinsics_path.read_text()) if intrinsics_path.exists() else None

        self._realtime = realtime
        self._t0_wall: Optional[float] = None
        self._t0_capture: Optional[float] = None

    def read(self) -> Optional[FramePacket]:
        if self._idx >= len(self._packets):
            return None
        row = self._packets[self._idx]
        self._idx += 1

        if self._realtime:
            now = time.time()
            if self._t0_wall is None:
                self._t0_wall = now
                self._t0_capture = row["t_capture"]
            target_wall = self._t0_wall + (row["t_capture"] - self._t0_capture)
            if target_wall > now:
                time.sleep(target_wall - now)

        bgr = cv2.imread(str(self._frames_dir / row["frame_file"]))
        if bgr is None:
            raise FileNotFoundError(
                f"could not read frame {row['frame_file']} listed in packets.jsonl"
            )
        # Undo record_pass.py's RGB->BGR conversion -- see class
        # docstring. Must match SimFrameSource's rgb8 exactly.
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        return FramePacket(
            image=rgb,
            gyro=np.array(row["gyro"], dtype=np.float64),
            t_capture=row["t_capture"],
            seq=row["seq"],
            gyro_valid=row["gyro_valid"],
        )

    @property
    def dropped_frames(self) -> int:
        return 0

    @property
    def intrinsics(self) -> Optional[dict]:
        """{fx, fy, cx, cy, width, height} -- same decoded form as
        SimFrameSource.intrinsics, decoded from the stored K matrix."""
        if self._intrinsics is None:
            return None
        k = self._intrinsics["k"]
        return {
            "fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5],
            "width": self._intrinsics["width"],
            "height": self._intrinsics["height"],
        }

    def get_intrinsics(self) -> Optional[dict]:
        """Full stored camera_info (K/D/distortion_model), same shape as
        SimFrameSource.get_intrinsics()."""
        return self._intrinsics

    def wait_for_intrinsics(self, timeout_s: float = 5.0) -> dict:
        """Same method name as SimFrameSource for interchangeability.
        Recordings either have intrinsics.json or they don't -- there's
        nothing to wait for here, no timeout is actually possible."""
        if self._intrinsics is None:
            raise TimeoutError(
                f"no intrinsics.json found in recording {self._dir}"
            )
        return self._intrinsics

    def close(self):
        pass
