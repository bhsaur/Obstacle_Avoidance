#!/usr/bin/env python3
"""Compatibility entry point for the CheapStage monitor (no flight commands)."""
from obst_avoidance.live_viewer import main

if __name__ == '__main__':
    raise SystemExit(main())
