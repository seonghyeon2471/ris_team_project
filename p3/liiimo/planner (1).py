"""
Frenet Planner
==============
Generates candidate lateral offset paths in Frenet (s, d) coordinates,
scores them for collision-free passage, and returns the best path.

Reference: arXiv:2507.12449 §II-B1
"""

import logging
import numpy as np
from scipy.interpolate import CubicSpline

log = logging.getLogger("frenet_planner")


class FrenetPlanner:
    def __init__(self, cfg: dict):
        self.road_half_width  = cfg["road_half_width"]      # m
        self.num_samples      = cfg["num_lateral_samples"]  # candidate paths
        self.lookahead_s      = cfg["lookahead_s"]          # m along ref path
        self.obstacle_radius  = cfg["obstacle_radius"]      # m clearance

    # ──────────────────────────────────────────────────────
    def plan(self,
             ref_path:   np.ndarray,   # (N,2) global XY waypoints
             vehicle_pos: np.ndarray,  # (2,) ego position in same frame
             vehicle_yaw: float,       # radians
             obstacles:   list) -> np.ndarray | None:
        """
        Returns best path as (M, 2) XY array in vehicle frame,
        or None if no valid path found.
        """
        if len(ref_path) < 2:
            return None

        # ── 1. Frenet frame along reference path ──────────
        s_vals, e_ref = self._build_frenet_frame(ref_path)

        # ── 2. Find ego s-coordinate ──────────────────────
        s_ego = self._project_to_frenet(vehicle_pos, ref_path, s_vals)

        # ── 3. Sample candidate end lateral offsets ────────
        d_samples = np.linspace(
            -self.road_half_width, self.road_half_width, self.num_samples)

        # ── 4. Build & score each candidate path ──────────
        best_path  = None
        best_score = -np.inf

        for d_end in d_samples:
            path_xy = self._generate_path(
                ref_path, s_vals, s_ego, d_end)
            if path_xy is None:
                continue

            score = self._score_path(path_xy, obstacles, d_end)
            if score > best_score:
                best_score = score
                best_path  = path_xy

        if best_path is None:
            log.warning("Frenet: no collision-free path found")
        else:
            log.debug(f"Frenet: best d_end={best_score:.2f} score")

        return best_path

    # ──────────────────────────────────────────────────────
    # Build cumulative arc-length + tangent vectors
    # ──────────────────────────────────────────────────────
    def _build_frenet_frame(self, ref_path):
        diff  = np.diff(ref_path, axis=0)
        segs  = np.linalg.norm(diff, axis=1)
        s_vals = np.concatenate([[0.0], np.cumsum(segs)])

        # Unit tangent / normal per point
        tangents = np.vstack([diff, diff[-1]])
        norms = np.linalg.norm(tangents, axis=1, keepdims=True).clip(1e-6)
        tangents = tangents / norms
        normals  = np.column_stack([-tangents[:, 1], tangents[:, 0]])

        return s_vals, normals

    # ──────────────────────────────────────────────────────
    def _project_to_frenet(self, pos, ref_path, s_vals):
        """Find s of the closest point on ref_path to pos."""
        dists = np.linalg.norm(ref_path - pos, axis=1)
        idx   = int(np.argmin(dists))
        return s_vals[idx]

    # ──────────────────────────────────────────────────────
    def _generate_path(self, ref_path, s_vals, s_ego, d_end):
        """
        Quintic polynomial lateral profile from d=0 (ego lane centre)
        to d=d_end over lookahead_s metres.
        Returns (M, 2) XY points in vehicle frame.
        """
        s_start = s_ego
        s_finish = s_ego + self.lookahead_s

        if s_finish > s_vals[-1]:
            s_finish = s_vals[-1]
        if s_finish <= s_start:
            return None

        # Dense s samples along lookahead
        s_query = np.linspace(s_start, s_finish, 30)

        # Cubic spline to interpolate reference x, y
        try:
            cs_x = CubicSpline(s_vals, ref_path[:, 0])
            cs_y = CubicSpline(s_vals, ref_path[:, 1])
        except Exception:
            return None

        ref_x = cs_x(s_query)
        ref_y = cs_y(s_query)

        # Lateral profile: quintic polynomial d(s)
        # Boundary conditions: d(0)=0, d'(0)=0, d''(0)=0
        #                      d(S)=d_end, d'(S)=0, d''(S)=0
        S = s_finish - s_start
        t  = (s_query - s_start) / S
        # quintic: d(t) = d_end * (10t^3 - 15t^4 + 6t^5)
        d_vals = d_end * (10*t**3 - 15*t**4 + 6*t**5)

        # Normal direction from spline derivative
        dx = cs_x(s_query, 1)
        dy = cs_y(s_query, 1)
        norms = np.sqrt(dx**2 + dy**2).clip(1e-6)
        nx = -dy / norms    # left normal
        ny =  dx / norms

        path_x = ref_x + d_vals * nx
        path_y = ref_y + d_vals * ny

        return np.column_stack([path_x, path_y])

    # ──────────────────────────────────────────────────────
    def _score_path(self, path_xy, obstacles, d_end):
        """
        Higher score = better.
        Penalise: collision with obstacles, large lateral offset.
        """
        # Collision check
        for obs in obstacles:
            ox, oy = obs["vehicle_xyz"][0], obs["vehicle_xyz"][1]
            dists = np.linalg.norm(
                path_xy - np.array([ox, oy]), axis=1)
            if dists.min() < self.obstacle_radius:
                return -np.inf   # discard

        # Prefer small lateral deviation (comfort)
        lateral_penalty = abs(d_end) * 2.0

        # Prefer smooth path (curvature proxy: total length)
        path_len = np.sum(np.linalg.norm(np.diff(path_xy, axis=0), axis=1))
        smooth_penalty = path_len * 0.1

        return -(lateral_penalty + smooth_penalty)
