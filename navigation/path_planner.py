#!/usr/bin/env python3
"""path_planner.py – Phase-1 stub
Provides minimal straight-line waypoint planner.
"""
from __future__ import annotations

from typing import List, Tuple
import math

class PathPlanner:
    def plan_path(self, start: Tuple[float, float], dst: Tuple[float, float], costmap=None) -> List[Tuple[float, float]]:
        """A* over 0.2-m grid if costmap given; else straight line."""
        if costmap is None:
            return self._straight_line(start, dst)

        # Build 0.2-m grid centred on robot for A*
        resolution = 0.2
        sx, sy = start
        gx, gy = dst
        max_range = 5.0
        half_cells = int(max_range / resolution)

        def world_to_cell(x, y):
            return int((x - sx) / resolution) + half_cells, int((y - sy) / resolution) + half_cells

        sx_c, sy_c = world_to_cell(sx, sy)
        gx_c, gy_c = world_to_cell(gx, gy)

        grid_size = half_cells * 2 + 1
        # A*
        open_set = [(0, (sx_c, sy_c))]
        came: dict[tuple[int, int], tuple[int, int]] = {}
        g_cost = { (sx_c, sy_c): 0.0 }

        def h(cx, cy):
            return math.hypot(cx - gx_c, cy - gy_c)

        while open_set:
            open_set.sort(key=lambda t: t[0])
            _, current = open_set.pop(0)
            if current == (gx_c, gy_c):
                break  # reached
            cx, cy = current
            for nx, ny in [(cx+1,cy), (cx-1,cy), (cx,cy+1), (cx,cy-1)]:
                if not (0 <= nx < grid_size and 0 <= ny < grid_size):
                    continue
                wx = sx + (nx - half_cells) * resolution
                wy = sy + (ny - half_cells) * resolution
                if costmap.is_occupied(wx, wy, (sx, sy)):
                    continue
                new_g = g_cost[current] + resolution
                if (nx, ny) not in g_cost or new_g < g_cost[(nx, ny)]:
                    g_cost[(nx, ny)] = new_g
                    prio = new_g + h(nx, ny)
                    open_set.append((prio, (nx, ny)))
                    came[(nx, ny)] = current

        # reconstruct
        path_cells: list[tuple[int,int]] = []
        cur = (gx_c, gy_c)
        while cur != (sx_c, sy_c) and cur in came:
            path_cells.append(cur)
            cur = came[cur]
        path_cells.reverse()
        if not path_cells:
            return self._straight_line(start, dst)  # fallback

        waypoints = []
        for cx, cy in path_cells:
            wx = sx + (cx - half_cells) * resolution
            wy = sy + (cy - half_cells) * resolution
            waypoints.append((wx, wy))
        return waypoints

    def _straight_line(self, start, dst):
        sx, sy = start
        dx, dy = dst
        total_dist = math.hypot(dx - sx, dy - sy)
        if total_dist == 0:
            return []
        steps = max(1, int(total_dist))
        return [(sx + (i/steps)*(dx-sx), sy + (i/steps)*(dy-sy)) for i in range(1, steps+1)] 