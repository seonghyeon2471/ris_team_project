#!/usr/bin/env python3

import cv2
import numpy as np
import sys
import time

COLOR = sys.argv[1] if len(sys.argv) > 1 else "red"
WIN = f"HSV Tuner [{COLOR}]"

DEFAULTS = {
    "red":    [0,120,80,12,255,255],
    "yellow": [20,40,180,40,255,255],
    "blue":   [95,70,50,130,255,255],
    "green":  [40,80,80,80,255,255],
}

init = DEFAULTS.get(COLOR, [0,50,50,30,255,255])

# --------------------------------------------------
# CAMERA
# --------------------------------------------------
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*'MJPG'))

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

time.sleep(1.0)

# 자동 노출 OFF
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.4)

# 자동 화이트밸런스 OFF
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# --------------------------------------------------
# TRACKBAR
# --------------------------------------------------
cv2.namedWindow(WIN)

cv2.createTrackbar("H_lo", WIN, init[0], 179, lambda x: None)
cv2.createTrackbar("S_lo", WIN, init[1], 255, lambda x: None)
cv2.createTrackbar("V_lo", WIN, init[2], 255, lambda x: None)

cv2.createTrackbar("H_hi", WIN, init[3], 179, lambda x: None)
cv2.createTrackbar("S_hi", WIN, init[4], 255, lambda x: None)
cv2.createTrackbar("V_hi", WIN, init[5], 255, lambda x: None)

print()
print("====================================")
print("HSV TUNER")
print("p : HSV 값 출력")
print("q : 종료")
print("====================================")
print()

prev_time = time.time()

while True:

    # 최신 프레임 사용
    for _ in range(2):
        cap.grab()

    ret, frame = cap.read()

    if not ret:
        continue

    frame = cv2.flip(frame, 1)

    hlo = cv2.getTrackbarPos("H_lo", WIN)
    slo = cv2.getTrackbarPos("S_lo", WIN)
    vlo = cv2.getTrackbarPos("V_lo", WIN)

    hhi = cv2.getTrackbarPos("H_hi", WIN)
    shi = cv2.getTrackbarPos("S_hi", WIN)
    vhi = cv2.getTrackbarPos("V_hi", WIN)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = np.array([hlo, slo, vlo])
    upper = np.array([hhi, shi, vhi])

    mask = cv2.inRange(hsv, lower, upper)

    # Morphology
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (7, 7)
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    result = cv2.bitwise_and(
        frame,
        frame,
        mask=mask
    )

    # Contour
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if contours:

        biggest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(biggest)

        if area > 300:

            x, y, w, h = cv2.boundingRect(biggest)

            M = cv2.moments(biggest)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.circle(
                    frame,
                    (cx, cy),
                    6,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    f"({cx},{cy})",
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0,255,0),
                    1
                )

            cv2.rectangle(
                frame,
                (x, y),
                (x+w, y+h),
                (0,255,0),
                2
            )

            cv2.putText(
                frame,
                f"Area:{int(area)}",
                (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0,255,0),
                1
            )

    # FPS
    now = time.time()
    fps = 1.0 / (now - prev_time)
    prev_time = now

    cv2.putText(
        frame,
        f"FPS:{fps:.1f}",
        (10,30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255,0,0),
        2
    )

    cv2.putText(
        frame,
        f"H:[{hlo},{hhi}] S:[{slo},{shi}] V:[{vlo},{vhi}]",
        (10,60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255,255,255),
        1
    )

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    stack = np.hstack([
        frame,
        mask_bgr,
        result
    ])

    cv2.imshow(WIN, stack)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    elif key == ord('p'):
        print()
        print(f"# {COLOR}")
        print(
            f'(np.array([{hlo}, {slo}, {vlo}]), '
            f'np.array([{hhi}, {shi}, {vhi}])),'
        )
        print()

cap.release()
cv2.destroyAllWindows()
