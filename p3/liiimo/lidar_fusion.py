"""
LiDAR Fusion Module — RPLIDAR C1
=================================
Reads 2D scan data from RPLIDAR C1 via rplidar library,
converts polar → Cartesian vehicle frame,
and outputs obstacle candidates that complement the camera.

Install:  pip install rplidar-roboticia
"""

import logging
import threading
import math
import numpy as np

log = logging.getLogger("lidar")

try:
    from rplidar import RPLidar
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False
    log.warning("rplidar package not found. LiDAR disabled. "
                "Install with: pip install rplidar-roboticia")


# Minimum and maximum distances to consider (metres)
MIN_DIST = 0.15
MAX_DIST = 6.0

# Field of view to consider (degrees) — front-facing ±90°
FOV_MIN = -90
FOV_MAX =  90


class LidarFusion:
    def __init__(self, port: str, baudrate: int):
        self.port     = port
        self.baudrate = baudrate
        self._lidar   = None
        self._lock    = threading.Lock()
        self._obstacles: list = []
        self._thread  = None
        self._running = False

    # ──────────────────────────────────────────────────────
    def start(self):
        if not RPLIDAR_AVAILABLE:
            log.warning("LiDAR not started (package missing)")
            return
        self._running = True
        self._thread  = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        log.info(f"LiDAR thread started on {self.port}")

    def stop(self):
        self._running = False
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.disconnect()
            except Exception:
                pass
        log.info("LiDAR stopped")

    def get_obstacles(self) -> list:
        """Return latest obstacle list (thread-safe)."""
        with self._lock:
            return list(self._obstacles)

    # ──────────────────────────────────────────────────────
    def _scan_loop(self):
        try:
            self._lidar = RPLidar(self.port, baudrate=self.baudrate)
            self._lidar.start_motor()

            for scan in self._lidar.iter_scans(max_buf_meas=500):
                if not self._running:
                    break
                obs = self._process_scan(scan)
                with self._lock:
                    self._obstacles = obs

        except Exception as e:
            log.error(f"LiDAR error: {e}")
        finally:
            if self._lidar:
                self._lidar.stop_motor()
                self._lidar.disconnect()

    def _process_scan(self, scan) -> list:
        """
        scan: list of (quality, angle_deg, distance_mm)

        Returns list of obstacle dicts compatible with camera detections.
        """
        obstacles = []
        for quality, angle_deg, dist_mm in scan:
            if quality < 5:
                continue
            dist_m = dist_mm / 1000.0
            if not (MIN_DIST <= dist_m <= MAX_DIST):
                continue

            # Normalise angle to [-180, 180)
            angle = (angle_deg + 180) % 360 - 180

            # Only consider front FOV
            if not (FOV_MIN <= angle <= FOV_MAX):
                continue

            # Polar → vehicle frame (X forward, Y left)
            rad = math.radians(angle)
            xv  =  dist_m * math.cos(rad)   # forward
            yv  = -dist_m * math.sin(rad)   # left (+) right (-)

            obstacles.append({
                "label": "lidar_point",
                "conf": 1.0,
                "bbox": None,
                "distance": dist_m,
                "cam_xyz": np.array([0.0, 0.0, dist_m]),
                "vehicle_xyz": np.array([xv, yv, 0.0]),
            })

        # Cluster nearby points to reduce noise
        return _cluster_obstacles(obstacles, radius=0.3)


# ──────────────────────────────────────────────────────────
# Simple greedy clustering
# ──────────────────────────────────────────────────────────
def _cluster_obstacles(points: list, radius: float) -> list:
    if not points:
        return []

    pts = np.array([p["vehicle_xyz"][:2] for p in points])
    used = np.zeros(len(pts), dtype=bool)
    clusters = []

    for i in range(len(pts)):
        if used[i]:
            continue
        dists = np.linalg.norm(pts - pts[i], axis=1)
        members = np.where(dists < radius)[0]
        used[members] = True

        centre = pts[members].mean(axis=0)
        dist   = float(np.linalg.norm(centre))
        clusters.append({
            "label": "lidar_cluster",
            "conf": 1.0,
            "bbox": None,
            "distance": dist,
            "cam_xyz": np.array([0.0, 0.0, dist]),
            "vehicle_xyz": np.array([centre[0], centre[1], 0.0]),
        })

    return clusters
