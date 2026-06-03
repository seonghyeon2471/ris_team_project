#!/usr/bin/env python3
"""
HSV Color Tuner – practice6/7 스타일 확장판
색상별 HSV 범위를 트랙바로 실시간 조정하고 결과를 출력합니다.
color_tracking_robot.py 의 COLOR_PROFILES 값을 보정할 때 사용하세요.

Usage:
  python3 hsv_tuner.py [color_name]
  예) python3 hsv_tuner.py red
"""

import cv2
import numpy as np
import sys

COLOR = sys.argv[1] if len(sys.argv) > 1 else "red"
WIN   = f"HSV Tuner [{COLOR}]"

# 초기값 (red 기준 – 필요 시 변경)
DEFAULTS = {
    "red":    [0, 120, 70, 10, 255, 255],
    "yellow": [20, 100, 100, 40, 255, 255],
    "blue":   [100, 150, 50, 130, 255, 255],
    "green":  [40, 80, 80, 80, 255, 255],
}
init = DEFAULTS.get(COLOR, [0, 50, 50, 30, 255, 255])

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cv2.namedWindow(WIN)
cv2.createTrackbar("H_lo", WIN, init[0], 179, lambda x: None)
cv2.createTrackbar("S_lo", WIN, init[1], 255,  lambda x: None)
cv2.createTrackbar("V_lo", WIN, init[2], 255,  lambda x: None)
cv2.createTrackbar("H_hi", WIN, init[3], 179, lambda x: None)
cv2.createTrackbar("S_hi", WIN, init[4], 255,  lambda x: None)
cv2.createTrackbar("V_hi", WIN, init[5], 255,  lambda x: None)

print(f"[HSV Tuner] Tuning: {COLOR}  |  Press 'p' to print values, 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hlo = cv2.getTrackbarPos("H_lo", WIN)
    slo = cv2.getTrackbarPos("S_lo", WIN)
    vlo = cv2.getTrackbarPos("V_lo", WIN)
    hhi = cv2.getTrackbarPos("H_hi", WIN)
    shi = cv2.getTrackbarPos("S_hi", WIN)
    vhi = cv2.getTrackbarPos("V_hi", WIN)

    lower = np.array([hlo, slo, vlo])
    upper = np.array([hhi, shi, vhi])

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w//2, y + h//2
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"area:{int(cv2.contourArea(cnt))}",
                        (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    stack = np.hstack([frame, result])
    cv2.imshow(WIN, stack)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('p'):
        print(f"\n# {COLOR}")
        print(f"(np.array([{hlo}, {slo}, {vlo}]), np.array([{hhi}, {shi}, {vhi}])),")

cap.release()
cv2.destroyAllWindows()
