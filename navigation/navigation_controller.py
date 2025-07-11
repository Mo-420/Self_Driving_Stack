#!/usr/bin/env python3
"""navigation_controller.py
Phase-1 skeleton NavigationController.
This singleton orchestrates path planning, position tracking, and low-level driving.
It is *ROS-agnostic* for now – motor commands are printed to stdout.
"""
from __future__ import annotations

import threading
import time
from enum import Enum, auto
from typing import Dict

from navigation.position_tracker import PositionTracker
from navigation.path_planner import PathPlanner

class NavigationState(str, Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    PAUSED = "paused"
    ERROR = "error"

class NavigationController:
    _instance: "NavigationController | None" = None

    @classmethod
    def get_instance(cls) -> "NavigationController":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    # ---------------------------------------------------------------------
    def __init__(self) -> None:
        self._state: NavigationState = NavigationState.IDLE
        self._destination: tuple[float, float] | None = None  # lat, lon
        self._lock = threading.Lock()
        self._tracker = PositionTracker()
        self._planner = PathPlanner()
        self._current_waypoints: list[tuple[float, float]] = []
        self.costmap = None  # type: ignore  # LocalCostmap injected by builder
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._running = True
        self._thread.start()

    # ---------------------------------------------------------------------
    # Public API – called from Web server
    # ---------------------------------------------------------------------
    def start_navigation(self, lat: float, lon: float) -> None:
        with self._lock:
            self._destination = (lat, lon)
            self._current_waypoints = self._planner.plan_path(self._tracker.position, self._destination)
            self._state = NavigationState.NAVIGATING
            print(f"[Navigation] Started to {self._destination}, {len(self._current_waypoints)} waypoints")

    def stop_navigation(self) -> None:
        with self._lock:
            print("[Navigation] Stopped")
            self._state = NavigationState.IDLE
            self._current_waypoints.clear()
            self._destination = None

    def pause_navigation(self) -> None:
        with self._lock:
            if self._state == NavigationState.NAVIGATING:
                print("[Navigation] Paused")
                self._state = NavigationState.PAUSED

    def resume_navigation(self) -> None:
        with self._lock:
            if self._state == NavigationState.PAUSED:
                print("[Navigation] Resumed")
                self._state = NavigationState.NAVIGATING

    # ---------------------------------------------------------------------
    def get_status(self) -> Dict[str, object]:
        pos = self._tracker.position
        return {
            "state": self._state,
            "position": {"x": pos[0], "y": pos[1]},
            "destination": self._destination,
            "remaining_waypoints": len(self._current_waypoints),
        }

    # ------------------------------------------------------------------ loop
    def _loop(self) -> None:
        UPDATE_HZ = 10
        dt = 1.0 / UPDATE_HZ
        while self._running:
            time_start = time.perf_counter()
            self._tick(dt)
            elapsed = time.perf_counter() - time_start
            time.sleep(max(0.0, dt - elapsed))

    def _tick(self, dt: float) -> None:
        self._tracker.update(dt)  # sensor fusion

        with self._lock:
            if self._state != NavigationState.NAVIGATING:
                return
            # Check destination
            if not self._current_waypoints:
                print("[Navigation] Destination reached")
                self._state = NavigationState.IDLE
                return
            # Compute control to next waypoint
            wp_x, wp_y = self._current_waypoints[0]
            cur_x, cur_y = self._tracker.position
            dx, dy = wp_x - cur_x, wp_y - cur_y
            dist = (dx ** 2 + dy ** 2) ** 0.5
            if dist < 0.3:
                # reached waypoint
                self._current_waypoints.pop(0)
                return
            heading_cmd = time.time() % 0  # placeholder
            speed_cmd = 0.4  # placeholder constant speed
            # TODO: integrate with motor controller; for now print
            print(f"[Navigation] Drive towards WP ({wp_x:.2f},{wp_y:.2f}) dist={dist:.2f} speed={speed_cmd}") 