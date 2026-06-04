"""
Autonomous Obstacle Avoidance Pipeline
Based on: Vision-based Perception for Autonomous Vehicles (arXiv:2507.12449)

Hardware:
  - Raspberry Pi 4B (main compute)
  - HBVCAM USB 720p (camera)
  - RPLIDAR C1 (lidar, supplementary)
  - Arduino R4 Minima (motor/steering controller via Serial)

Pipeline:
  Camera → YOLOv11 detection → Depth Anything V2 depth estimation
  → Coordinate transform → Frenet path planning → Pure Pursuit tracking
  → Serial commands to Arduino
"""

import time
import threading
import logging
import argparse

import cv2
import numpy as np
import serial

from perception import PerceptionModule
from planner import FrenetPlanner
from controller import PurePursuitController
from lidar_fusion import LidarFusion
from arduino_comm import ArduinoComm

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(name)s][%(levelname)s] %(message)s"
)
log = logging.getLogger("main")


# ──────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────
CFG = {
    # Camera
    "camera_id": 0,                 # /dev/video0
    "cam_width": 1280,
    "cam_height": 720,
    "cam_fps": 30,
    # Camera intrinsics (calibrate your HBVCAM!)
    "fx": 640.0, "fy": 640.0,
    "cx": 640.0, "cy": 360.0,
    # YOLO
    "yolo_model": "yolo11n.pt",     # nano is fastest on Pi4
    "yolo_conf": 0.45,
    "yolo_classes": None,           # None = all COCO classes
    # Depth Anything V2
    "depth_encoder": "vits",        # vits/vitb/vitl  (vits fastest)
    # Obstacle safety
    "obstacle_radius": 1.0,         # metres, inflation around obstacle
    "danger_distance": 2.5,         # metres, emergency stop threshold
    # Frenet planner
    "road_half_width": 2.0,         # metres
    "num_lateral_samples": 7,       # candidate paths
    "lookahead_s": 5.0,             # metres along reference path
    # Pure Pursuit
    "lookahead_dist": 0.6,          # metres (바퀴 둘레 0.214m의 약 3배)
    "wheelbase": 0.18,              # metres — 좌우 바퀴 간격(트랙 폭) 실측 필요!
    "max_steer_deg": 30.0,
    # Speed control
    # 물리 한계: max RPM=150 → max speed ≈ 150/60 * 0.2136 ≈ 0.53 m/s
    "cruise_speed": 0.35,           # m/s  ≈ RPM 98
    "avoidance_speed": 0.20,        # m/s  ≈ RPM 56
    # RPLIDAR
    "lidar_port": "/dev/ttyUSB1",
    "lidar_baudrate": 460800,
    "lidar_use": True,
    # Arduino serial
    "arduino_port": "/dev/ttyUSB0",
    "arduino_baud": 115200,
    # Display
    "show_viz": True,
}


# ──────────────────────────────────────────────
# Main loop
# ──────────────────────────────────────────────
def main(cfg):
    log.info("Initialising subsystems…")

    # Arduino comm
    arduino = ArduinoComm(cfg["arduino_port"], cfg["arduino_baud"])
    arduino.connect()

    # Camera
    cap = cv2.VideoCapture(cfg["camera_id"], cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg["cam_width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["cam_height"])
    cap.set(cv2.CAP_PROP_FPS,          cfg["cam_fps"])
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    # Perception (YOLO + Depth)
    perception = PerceptionModule(cfg)

    # LIDAR fusion (optional)
    lidar = None
    if cfg["lidar_use"]:
        lidar = LidarFusion(cfg["lidar_port"], cfg["lidar_baudrate"])
        lidar.start()

    # Path planner
    planner = FrenetPlanner(cfg)

    # Path tracker
    controller = PurePursuitController(cfg)

    # Simple straight reference path (replace with GPS waypoints)
    ref_path = np.array([[i * 0.5, 0.0] for i in range(40)], dtype=float)

    log.info("Pipeline running. Press 'q' in window or Ctrl-C to stop.")

    try:
        while True:
            t0 = time.time()

            ret, frame = cap.read()
            if not ret:
                log.warning("Frame grab failed, skipping")
                continue

            # ── 1. PERCEPTION ──────────────────────────────
            detections, depth_map, annotated = perception.process(frame)
            # detections: list of {label, bbox, distance, cam_xyz, vehicle_xyz}

            # ── 2. LIDAR FUSION (supplement near-field) ────
            lidar_obstacles = []
            if lidar is not None:
                lidar_obstacles = lidar.get_obstacles()

            all_obstacles = _merge_obstacles(detections, lidar_obstacles)

            # ── 3. EMERGENCY STOP CHECK ────────────────────
            closest = _closest_obstacle(all_obstacles)
            if closest < cfg["danger_distance"]:
                log.warning(f"EMERGENCY STOP — obstacle at {closest:.2f} m")
                arduino.send_command(speed=0.0, steer_deg=0.0)
                if cfg["show_viz"]:
                    _show(annotated, all_obstacles, None, closest)
                continue

            # ── 4. FRENET PATH PLANNING ────────────────────
            vehicle_pos = np.array([0.0, 0.0])   # ego origin in vehicle frame
            vehicle_yaw = 0.0
            best_path = planner.plan(ref_path, vehicle_pos, vehicle_yaw,
                                     all_obstacles)

            # ── 5. PURE PURSUIT TRACKING (차동구동) ────────
            if best_path is not None and len(best_path) > 1:
                base_speed = (cfg["avoidance_speed"]
                              if closest < cfg["danger_distance"] * 2
                              else cfg["cruise_speed"])
                v_left, v_right, target_pt = controller.compute(
                    vehicle_pos, vehicle_yaw, best_path, base_speed)
                # HUD용 등가 조향각
                steer_deg, _ = controller.compute_steering(
                    vehicle_pos, vehicle_yaw, best_path)
                speed = base_speed
            else:
                log.warning("No valid path — stopping")
                v_left, v_right, steer_deg, speed = 0.0, 0.0, 0.0, 0.0
                target_pt = None

            # ── 6. SEND TO ARDUINO ─────────────────────────
            # 차동구동: 좌우 속도를 speed + steer로 인코딩
            # Arduino에서 다시 믹싱하므로, 등가 steer_deg 전송
            arduino.send_command(speed=speed, steer_deg=steer_deg)

            # ── 7. VISUALISATION ───────────────────────────
            if cfg["show_viz"]:
                vis = _draw_overlay(annotated, all_obstacles, best_path,
                                    target_pt, steer_deg, speed, closest)
                cv2.imshow("Autonomous Avoidance", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            dt = time.time() - t0
            log.debug(f"Loop {dt*1000:.1f} ms | steer={steer_deg:.1f}° "
                      f"speed={speed:.2f} m/s | obs={len(all_obstacles)}")

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt — shutting down")
    finally:
        arduino.send_command(0.0, 0.0)
        arduino.close()
        cap.release()
        if lidar:
            lidar.stop()
        cv2.destroyAllWindows()
        log.info("Shutdown complete.")


# ──────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────
def _merge_obstacles(cam_obs, lidar_obs):
    """Combine camera detections and lidar obstacles into unified list."""
    merged = list(cam_obs)
    for lo in lidar_obs:
        # Only add lidar point if no camera detection within 0.5 m
        dist_to_cam = min(
            (np.linalg.norm(lo["vehicle_xyz"][:2] -
                            np.array(o["vehicle_xyz"][:2]))
             for o in cam_obs), default=999)
        if dist_to_cam > 0.5:
            merged.append(lo)
    return merged


def _closest_obstacle(obstacles):
    if not obstacles:
        return 999.0
    return min(o["distance"] for o in obstacles)


def _draw_overlay(frame, obstacles, path, target_pt,
                  steer_deg, speed, closest_dist):
    h, w = frame.shape[:2]
    vis = frame.copy()

    # HUD background
    cv2.rectangle(vis, (0, 0), (300, 90), (0, 0, 0), -1)
    cv2.putText(vis, f"Speed  : {speed:.2f} m/s",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 120), 2)
    cv2.putText(vis, f"Steer  : {steer_deg:+.1f} deg",
                (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
    cv2.putText(vis, f"Closest: {closest_dist:.2f} m",
                (8, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

    # Bird's-eye mini-map (bottom-right 200×200)
    MAP_SIZE = 200
    MAP_SCALE = 20   # pixels per metre
    mx0, my0 = w - MAP_SIZE - 10, h - MAP_SIZE - 10
    cv2.rectangle(vis, (mx0, my0), (mx0+MAP_SIZE, my0+MAP_SIZE),
                  (30, 30, 30), -1)
    ego_x, ego_y = mx0 + MAP_SIZE//2, my0 + MAP_SIZE - 20

    # Draw path on mini-map
    if path is not None:
        for i in range(len(path)-1):
            px = int(ego_x + path[i][1] * MAP_SCALE)
            py = int(ego_y - path[i][0] * MAP_SCALE)
            px2 = int(ego_x + path[i+1][1] * MAP_SCALE)
            py2 = int(ego_y - path[i+1][0] * MAP_SCALE)
            cv2.line(vis, (px, py), (px2, py2), (0, 255, 0), 1)

    # Draw obstacles on mini-map
    for o in obstacles:
        ox = int(ego_x + o["vehicle_xyz"][1] * MAP_SCALE)
        oy = int(ego_y - o["vehicle_xyz"][0] * MAP_SCALE)
        cv2.circle(vis, (ox, oy), 5, (0, 0, 255), -1)

    # Draw ego vehicle
    cv2.circle(vis, (ego_x, ego_y), 6, (0, 255, 255), -1)

    return vis


def _show(frame, obstacles, path, closest):
    _draw_overlay(frame, obstacles, path, None, 0, 0, closest)
    cv2.imshow("Autonomous Avoidance", frame)
    cv2.waitKey(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viz", action="store_true")
    parser.add_argument("--no-lidar", action="store_true")
    args = parser.parse_args()
    if args.no_viz:
        CFG["show_viz"] = False
    if args.no_lidar:
        CFG["lidar_use"] = False
    main(CFG)
