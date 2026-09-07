from .packet import FramePacket
from .base import FrameSource
from .log import LogFrameSource

__all__ = ["FramePacket", "FrameSource", "SimFrameSource", "LogFrameSource"]


def __getattr__(name):
    """Keep offline replay usable without importing ROS or MAVLink."""
    if name == "SimFrameSource":
        from .sim import SimFrameSource
        return SimFrameSource
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
