import cv2
import numpy as np
import math

# =========================
# CAMERA PARAMETER
# =========================
CAMERA_HEIGHT = 0.75   # 75cm = 0.75m
CAMERA_TILT = np.deg2rad(30)  # 30도 아래로 기울어짐

FRAME_W = 640
FRAME_H = 480

# USB 카메라 가정 FOV
HFOV = np.deg2rad(60)   # horizontal FOV 60도 가정
VFOV = np.deg2rad(45)

fx = FRAME_W / (2 * np.tan(HFOV / 2))
fy = FRAME_H / (2 * np.tan(VFOV / 2))

cx = FRAME_W / 2
cy = FRAME_H / 2


# =========================
# HSV (RED/YELLOW/BLUE)
# =========================
COLOR_RANGES = {
    "red": [
        (np.array([169, 168, 96]), np.array([179, 222, 157]))
    ],
    "yellow": [
        (np.array([16, 137, 142]), np.array([30, 214, 195]))
    ],
    "blue": [
        (np.array([106, 168, 54]), np.array([131, 210, 82]))
    ]
}


# =========================
# DETECT COLOR CENTER
# =========================
def get_center(mask):
    M = cv2.moments(mask)
    if M["m00"] == 0:
        return None
    cx_ = int(M["m10"] / M["m00"])
    cy_ = int(M["m01"] / M["m00"])
    return cx_, cy_


# =========================
# PIXEL → WORLD DISTANCE
# =========================
def pixel_to_distance(u, v):
    """
    카메라 픽셀 -> 바닥 평면까지 거리 (m)
    """

    # 카메라 좌표계 ray
    x = (u - cx) / fx
    y = (v - cy) / fy
    z = 1.0

    ray = np.array([x, y, z])
    ray = ray / np.linalg.norm(ray)

    # 카메라 pitch rotation (x축 기준)
    pitch = CAMERA_TILT
    R = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch),  np.cos(pitch)]
    ])

    ray_world = R @ ray

    # 바닥과 교차 (y 대신 z축 기준 사용)
    # 여기서 world: z up 기준
    t = CAMERA_HEIGHT / (-ray_world[1])

    X = ray_world[0] * t
    Z = ray_world[2] * t

    distance = math.sqrt(X**2 + Z**2)

    return distance


# =========================
# MAIN LOOP
# =========================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

target_color = "red"

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_total = None

    for lower, upper in COLOR_RANGES[target_color]:
        mask = cv2.inRange(hsv, lower, upper)
        mask_total = mask if mask_total is None else mask_total | mask

    center = get_center(mask_total)

    if center:
        u, v = center

        dist = pixel_to_distance(u, v)

        # 시각화
        cv2.circle(frame, center, 8, (0, 255, 0), -1)
        cv2.putText(frame, f"{dist:.2f} m",
                    (u + 10, v),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2)

        print("Distance:", dist, "m")

    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask_total)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
