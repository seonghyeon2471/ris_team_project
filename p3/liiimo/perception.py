"""
Perception Module
=================
1. YOLOv11  — obstacle detection (bounding boxes)
2. Depth Anything V2 — monocular depth estimation
3. Coordinate transform: image → camera → vehicle frame

Reference: arXiv:2507.12449 §II-A
"""

import logging
import numpy as np
import cv2
import torch
from ultralytics import YOLO

log = logging.getLogger("perception")


class PerceptionModule:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.fx = cfg["fx"]
        self.fy = cfg["fy"]
        self.cx = cfg["cx"]
        self.cy = cfg["cy"]

        # ── YOLOv11 ──────────────────────────────────────
        log.info(f"Loading YOLO model: {cfg['yolo_model']}")
        self.yolo = YOLO(cfg["yolo_model"])
        self.yolo_conf = cfg["yolo_conf"]
        self.yolo_classes = cfg["yolo_classes"]

        # ── Depth Anything V2 ─────────────────────────────
        log.info(f"Loading Depth Anything V2 encoder={cfg['depth_encoder']}")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.depth_model = self._load_depth_anything(cfg["depth_encoder"])

        # Depth scale factor (calibrate with known distances)
        # depth_metric = depth_scale * depth_raw
        self.depth_scale = 10.0  # adjust per environment

        log.info(f"Perception ready on device={self.device}")

    # ──────────────────────────────────────────────────────
    def _load_depth_anything(self, encoder: str):
        """
        Load Depth Anything V2.
        Install:  pip install depth-anything-v2
        Or clone: https://github.com/DepthAnything/Depth-Anything-V2
        """
        try:
            from depth_anything_v2.dpt import DepthAnythingV2
            model_configs = {
                "vits": {"encoder": "vits", "features": 64,
                         "out_channels": [48, 96, 192, 384]},
                "vitb": {"encoder": "vitb", "features": 128,
                         "out_channels": [96, 192, 384, 768]},
                "vitl": {"encoder": "vitl", "features": 256,
                         "out_channels": [256, 512, 1024, 1024]},
            }
            model = DepthAnythingV2(**model_configs[encoder])
            ckpt = f"checkpoints/depth_anything_v2_{encoder}.pth"
            model.load_state_dict(
                torch.load(ckpt, map_location="cpu"), strict=True)
            model = model.to(self.device).eval()
            log.info("Depth Anything V2 loaded from checkpoint")
            return model
        except Exception as e:
            log.warning(
                f"Depth Anything V2 not available ({e}). "
                "Falling back to MiDaS (torch.hub).")
            return self._load_midas_fallback()

    def _load_midas_fallback(self):
        """Fallback: MiDaS small for depth estimation."""
        model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small",
                               trust_repo=True)
        model.to(self.device).eval()
        self._midas_transform = torch.hub.load(
            "intel-isl/MiDaS", "transforms").small_transform
        self._is_midas = True
        return model
    # ──────────────────────────────────────────────────────

    def process(self, frame: np.ndarray):
        """
        Run full perception on one RGB frame.

        Returns
        -------
        detections : list of dict with keys
            label, conf, bbox (x1,y1,x2,y2), distance (m),
            cam_xyz (3,), vehicle_xyz (3,)
        depth_map  : H×W float32 metric depth in metres
        annotated  : BGR frame with overlay drawn
        """
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # ── Depth estimation ──────────────────────────────
        depth_raw = self._infer_depth(rgb)          # H×W, relative [0-1]
        depth_map = depth_raw * self.depth_scale    # → metres (approx)

        # ── YOLO detection ────────────────────────────────
        results = self.yolo.predict(
            frame,
            conf=self.yolo_conf,
            classes=self.yolo_classes,
            verbose=False,
        )
        annotated = results[0].plot()

        detections = []
        if results[0].boxes is not None:
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.yolo.names[cls_id]

                # ── Distance from depth map ────────────────
                # Use median inside bounding box (noise robust, §II-A2)
                roi = depth_map[
                    max(0, y1):min(depth_map.shape[0], y2),
                    max(0, x1):min(depth_map.shape[1], x2),
                ]
                if roi.size == 0:
                    continue
                distance = float(np.median(roi))

                # ── Image → Camera coordinates (§II-A3) ────
                xi = (x1 + x2) / 2.0   # bbox centre x
                yi = (y1 + y2) / 2.0   # bbox centre y
                cam_xyz = self._image_to_camera(xi, yi, distance)

                # ── Camera → Vehicle frame ──────────────────
                vehicle_xyz = self._camera_to_vehicle(cam_xyz)

                detections.append({
                    "label": label,
                    "conf": conf,
                    "bbox": (x1, y1, x2, y2),
                    "distance": distance,
                    "cam_xyz": cam_xyz,
                    "vehicle_xyz": vehicle_xyz,
                })

        return detections, depth_map, annotated

    # ──────────────────────────────────────────────────────
    # Coordinate transforms (§II-A3)
    # ──────────────────────────────────────────────────────

    def _image_to_camera(self, xi: float, yi: float, d: float) -> np.ndarray:
        """
        Pixel (xi, yi) + depth d  →  3D camera frame [Xc, Yc, Zc].

            Zc = d
            Xc = (xi - cx) * d / fx
            Yc = (yi - cy) * d / fy
        """
        Zc = d
        Xc = (xi - self.cx) * d / self.fx
        Yc = (yi - self.cy) * d / self.fy
        return np.array([Xc, Yc, Zc], dtype=float)

    def _camera_to_vehicle(self, p_cam: np.ndarray) -> np.ndarray:
        """
        Camera frame → Vehicle frame.

        Assumes camera is mounted at the front, facing forward:
          - Camera X  →  Vehicle Y  (lateral, left positive)
          - Camera Y  →  Vehicle Z  (vertical, down)
          - Camera Z  →  Vehicle X  (forward)

        Adjust R_cam_to_veh and t_cam for your mounting position.
        """
        # Rotation: camera axes to vehicle axes
        R_cam_to_veh = np.array([
            [0,  0,  1],   # Xv = Zc (forward)
            [-1, 0,  0],   # Yv = -Xc (left)
            [0, -1,  0],   # Zv = -Yc (up)
        ], dtype=float)

        # Translation: camera optical centre in vehicle frame [x_fwd, y_lat, z_up]
        t_cam = np.array([0.3, 0.0, 0.5], dtype=float)  # tune for your mount

        return R_cam_to_veh @ p_cam + t_cam

    # ──────────────────────────────────────────────────────
    # Depth inference
    # ──────────────────────────────────────────────────────

    def _infer_depth(self, rgb: np.ndarray) -> np.ndarray:
        """Returns H×W depth map (relative, normalised 0-1)."""
        h, w = rgb.shape[:2]

        if getattr(self, "_is_midas", False):
            # MiDaS fallback path
            inp = self._midas_transform(rgb).to(self.device)
            with torch.no_grad():
                pred = self.depth_model(inp)
                pred = torch.nn.functional.interpolate(
                    pred.unsqueeze(1), size=(h, w),
                    mode="bicubic", align_corners=False).squeeze()
            depth = pred.cpu().numpy()
        else:
            # Depth Anything V2 path
            with torch.no_grad():
                depth = self.depth_model.infer_image(rgb)   # returns H×W ndarray

        # Normalise to [0, 1] then scale is applied outside
        depth_min, depth_max = depth.min(), depth.max()
        if depth_max - depth_min > 1e-6:
            depth = (depth - depth_min) / (depth_max - depth_min)
        return depth.astype(np.float32)
