#!/usr/bin/env python3
"""position_tracker.py – Phase-1 stub
Combines wheel odometry, compass, and (future) landmark vision.
Currently simulates position by simple dead-reckoning with constant velocity 0.
"""
from __future__ import annotations

import random
from typing import Tuple

class PositionTracker:
    def __init__(self) -> None:
        # Internal state in metres (x East, y North)
        self._x: float = 0.0
        self._y: float = 0.0
        self._heading: float = 0.0  # radians (0 = East)

    # ------------------------------------------------------------- properties
    @property
    def position(self) -> Tuple[float, float]:
        return self._x, self._y

    @property
    def heading(self) -> float:
        return self._heading

    # --------------------------------------------------------------- public
    def update(self, dt: float) -> None:
        """Update pose estimate. For Phase-1 we just add small noise."""
        # Placeholder: drift slowly
        self._x += random.uniform(-0.005, 0.005)
        self._y += random.uniform(-0.005, 0.005)
        # Heading noise
        self._heading += random.uniform(-0.001, 0.001)

    # External setters for ROS bridge
    def set_position(self, x: float, y: float) -> None:
        self._x, self._y = x, y

    def set_heading(self, heading_rad: float) -> None:
        self._heading = heading_rad

    # ------------------------------------------------------ sensor callbacks
    # TODO: integrate real wheel-odometry / compass / vision updates 