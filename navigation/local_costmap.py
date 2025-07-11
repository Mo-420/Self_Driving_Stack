#!/usr/bin/env python3
"""local_costmap.py
Simple 2-D occupancy grid centred on the robot.
Size: 10 × 10 m (fixed), resolution 0.1 m → 100 × 100 cells.
index (0,0) is bottom-left of grid; robot sits at centre cell (50,50).
"""
from __future__ import annotations

from typing import List, Tuple
import numpy as np

GRID_SIZE = 10.0   # metres
RESOLUTION = 0.1   # metres per cell
GRID_CELLS = int(GRID_SIZE / RESOLUTION)  # 100
HALF = GRID_CELLS // 2

class LocalCostmap:
    def __init__(self):
        self._grid = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.uint8)  # 0 free, 1 occ

    # ------------------- coordinate helpers ----------------------------
    def _world_to_grid(self, x: float, y: float, robot_x: float, robot_y: float) -> Tuple[int, int] | None:
        gx = int((x - (robot_x - GRID_SIZE / 2)) / RESOLUTION)
        gy = int((y - (robot_y - GRID_SIZE / 2)) / RESOLUTION)
        if 0 <= gx < GRID_CELLS and 0 <= gy < GRID_CELLS:
            return gx, gy
        return None

    # ------------------- update sources --------------------------------
    def mark_obstacle(self, wx: float, wy: float, robot_pos: Tuple[float, float]):
        cell = self._world_to_grid(wx, wy, *robot_pos)
        if cell:
            self._grid[cell[1], cell[0]] = 1

    def clear(self):
        self._grid.fill(0)

    # ------------------- query ----------------------------------------
    def is_occupied(self, wx: float, wy: float, robot_pos: Tuple[float, float]) -> bool:
        cell = self._world_to_grid(wx, wy, *robot_pos)
        if cell is None:
            return True  # treat outside grid as occupied
        return self._grid[cell[1], cell[0]] == 1

    # expose grid for debug
    @property
    def grid(self):
        return self._grid 