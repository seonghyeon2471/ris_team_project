"""
LiDAR Fusion Module — RPLIDAR C1 (개선 버전)
- iter_measures() 사용 (C1에서 더 안정적)
- 안전한 unpacking + 주기적 클러스터링
"""

import logging
import threading
import math
import time
import numpy as np

log = logging.getLogger("lidar")

try:
    from rplidar import RPLidar
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False
    log.warning("rplidar package not found. LiDAR disabled.")

MIN_DIST = 0.15
MAX_DIST = 6.0
FOV_MIN = -90
FOV_MAX =  90


class LidarFusion:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self._lidar = None
        self._lock = threading.Lock()
        self._obstacles: list = []
        self._thread = None
        self._running = False

    def start(self):
        if not RPLIDAR_AVAILABLE:
            log.warning("LiDAR not started (package missing)")
            return
        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        log.info(f"LiDAR thread started on {self.port}")

    def stop(self):
        self._running = False
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception:
                pass
        log.info("LiDAR stopped")

    def get_obstacles(self) -> list:
        with self._lock:
            return list(self._obstacles)

    def _scan_loop(self):
        try:
            self._lidar = RPLidar(self.port, baudrate=self.baudrate)
            self._lidar.start_motor()
            log.info("RPLIDAR motor started successfully")

            point_buffer = []
            last_cluster_ts = time.time()
            CLUSTER_INTERVAL = 0.15

            for measure in self._lidar.iter_measures():
                if not self._running:
                    break
                try:
                    if len(measure) >= 3:
                        quality, angle_deg, dist_mm = measure[0], measure[1], measure[2]
                    else:
                        continue

                    if quality < 5:
                        continue
                    dist_m = dist_mm / 1000.0
                    if not (MIN_DIST <= dist_m <= MAX_DIST):
                        continue

                    angle = (angle_deg + 180) % 360 - 180
                    if not (FOV_MIN <= angle <= FOV_MAX):
                        continue

                    rad = math.radians(angle)
                    xv = dist_m * math.cos(rad)
                    yv = -dist_m * math.sin(rad)

                    point_buffer.append({
                        "label": "lidar_point",
                        "conf": 1.0,
                        "bbox": None,
                        "distance": dist_m,
                        "cam_xyz": np.array([0.0, 0.0, dist_m]),
                        "vehicle_xyz": np.array([xv, yv, 0.0]),
                    })

                    now = time.time()
                    if (now - last_cluster_ts > CLUSTER_INTERVAL) or (len(point_buffer) > 150):
                        if point_buffer:
                            clustered = _cluster_obstacles(point_buffer, radius=0.35)
                            with self._lock:
                                self._obstacles = clustered
                        point_buffer = []
                        last_cluster_ts = now

                except Exception:
                    continue

        except Exception as e:
            log.error(f"LiDAR error: {e}")
        finally:
            if self._lidar:
                try:
                    self._lidar.stop_motor()
                    self._lidar.disconnect()
                except Exception:
                    pass
            log.info("LiDAR thread exiting")


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
        dist = float(np.linalg.norm(centre))
        clusters.append({
            "label": "lidar_cluster",
            "conf": 1.0,
            "bbox": None,
            "distance": dist,
            "cam_xyz": np.array([0.0, 0.0, dist]),
            "vehicle_xyz": np.array([centre[0], centre[1], 0.0]),
        })
    return clusters
