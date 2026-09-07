from abc import ABC, abstractmethod
from typing import Optional

from .packet import FramePacket


class FrameSource(ABC):
    """Common interface for anything that produces FramePackets --
    SimFrameSource today, RealFrameSource / LogFrameSource later. Callers
    depend only on this interface, never on a concrete source."""

    @abstractmethod
    def read(self) -> Optional[FramePacket]:
        """Return the latest available FramePacket, or None if no new
        frame has arrived since the last read()."""
        raise NotImplementedError

    @property
    @abstractmethod
    def dropped_frames(self) -> int:
        """Count of frames that arrived but were overwritten before being
        read() -- i.e. the caller fell behind the source's frame rate."""
        raise NotImplementedError
