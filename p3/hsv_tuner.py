python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PROJECT HSV + BGR TUNER

1 : RED
2 : YELLOW
3 : BLUE

p : 현재 설정 출력
q : 종료

네 color_tracking 코드의 COLOR_CFG를 직접 튜닝하기 위한 툴
"""

import cv2
import numpy as np
import time

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

time.sleep(1)

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# =========================================
# DEFAULT COLOR CONFIG
# =========================================
COLOR_PRESET = {
    "red": {
        "hsv": [0, 60, 120, 12, 255, 255],
        "bgr": [20, 20, 80, 255, 255, 255],
        "draw": (0, 0, 255),
    },

    "yellow": {
        "hsv": [18, 15, 180, 45, 255, 255],
        "bgr": [0, 80, 80, 255, 255, 255],
        "draw": (0, 220, 255),
    },

    "blue": {
        "hsv": [95, 50, 50, 130, 255, 255],
        "bgr": [40, 0, 0, 255, 220, 220],
        "draw": (255, 100, 0),
    }
}

current_color = "red"

# =========================================
# WINDOW
# =========================================
cv2.namedWindow("TUNER", cv2.WINDOW_NORMAL)

# HSV
cv2.createTrackbar("H_LO", "TUNER", 0, 179, lambda x: None)
cv2.createTrackbar("S_LO", "TUNER", 0, 255, lambda x: None)
cv2.createTrackbar("V_LO", "TUNER", 0, 255, lambda x: None)

cv2.createTrackbar("H_HI", "TUNER", 179, 179, lambda x: None)
cv2.createTrackbar("S_HI", "TUNER", 255, 255, lambda x: None)
cv2.createTrackbar("V_HI", "TUNER", 255, 255, lambda x: None)

# BGR
cv2.createTrackbar("B_LO", "TUNER", 0, 255, lambda x: None)
cv2.createTrackbar("G_LO", "TUNER", 0, 255, lambda x: None)
cv2.createTrackbar("R_LO", "TUNER", 0, 255, lambda x: None)

cv2.createTrackbar("B_HI", "TUNER", 255, 255, lambda x: None)
cv2.createTrackbar("G_HI", "TUNER", 255, 255, lambda x: None)
cv2.createTrackbar("R_HI", "TUNER", 255, 255, lambda x: None)

kernel = cv2.getStructuringElement(
    cv2.MORPH_RECT,
    (5, 5)
)

# =========================================
# LOAD PRESET
# =========================================
def load_color(name):

    hsv = COLOR_PRESET[name]["hsv"]
    bgr = COLOR_PRESET[name]["bgr"]

    cv2.setTrackbarPos("H_LO", "TUNER", hsv[0])
    cv2.setTrackbarPos("S_LO", "TUNER", hsv[1])
    cv2.setTrackbarPos("V_LO", "TUNER", hsv[2])

    cv2.setTrackbarPos("H_HI", "TUNER", hsv[3])
    cv2.setTrackbarPos("S_HI", "TUNER", hsv[4])
    cv2.setTrackbarPos("V_HI", "TUNER", hsv[5])

    cv2.setTrackbarPos("B_LO", "TUNER", bgr[0])
    cv2.setTrackbarPos("G_LO", "TUNER", bgr[1])
    cv2.setTrackbarPos("R_LO", "TUNER", bgr[2])

    cv2.setTrackbarPos("B_HI", "TUNER", bgr[3])
    cv2.setTrackbarPos("G_HI", "TUNER", bgr[4])
    cv2.setTrackbarPos("R_HI", "TUNER", bgr[5])

load_color("red")

print("=" * 60)
print("1 : RED")
print("2 : YELLOW")
print("3 : BLUE")
print("p : PRINT CONFIG")
print("q : QUIT")
print("=" * 60)

# =========================================
# MAIN LOOP
# =========================================
while True:

    ret, frame = cap.read()

    if not ret:
        continue

    frame = cv2.flip(frame, 1)

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # -----------------------------
    # TRACKBAR READ
    # -----------------------------
    H_LO = cv2.getTrackbarPos("H_LO", "TUNER")
    S_LO = cv2.getTrackbarPos("S_LO", "TUNER")
    V_LO = cv2.getTrackbarPos("V_LO", "TUNER")

    H_HI = cv2.getTrackbarPos("H_HI", "TUNER")
    S_HI = cv2.getTrackbarPos("S_HI", "TUNER")
    V_HI = cv2.getTrackbarPos("V_HI", "TUNER")

    B_LO = cv2.getTrackbarPos("B_LO", "TUNER")
    G_LO = cv2.getTrackbarPos("G_LO", "TUNER")
    R_LO = cv2.getTrackbarPos("R_LO", "TUNER")

    B_HI = cv2.getTrackbarPos("B_HI", "TUNER")
    G_HI = cv2.getTrackbarPos("G_HI", "TUNER")
    R_HI = cv2.getTrackbarPos("R_HI", "TUNER")

    hsv_mask = cv2.inRange(
        hsv,
        np.array([H_LO, S_LO, V_LO]),
        np.array([H_HI, S_HI, V_HI])
    )

    bgr_mask = cv2.inRange(
        frame,
        np.array([B_LO, G_LO, R_LO]),
        np.array([B_HI, G_HI, R_HI])
    )

    final_mask = cv2.bitwise_and(
        hsv_mask,
        bgr_mask
    )

    final_mask = cv2.morphologyEx(
        final_mask,
        cv2.MORPH_OPEN,
        kernel
    )

    final_mask = cv2.morphologyEx(
        final_mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    result = cv2.bitwise_and(
        frame,
        frame,
        mask=final_mask
    )

    # -----------------------------
    # DETECTION
    # -----------------------------
    contours, _ = cv2.findContours(
        final_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if contours:

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(largest)

        if area > 300:

            rect = cv2.minAreaRect(largest)
            box = cv2.boxPoints(rect)
            box = np.int32(box)

            cx = int(rect[0][0])
            cy = int(rect[0][1])

            cv2.drawContours(
                frame,
                [box],
                0,
                COLOR_PRESET[current_color]["draw"],
                2
            )

            cv2.circle(
                frame,
                (cx, cy),
                5,
                (0, 255, 0),
                -1
            )

            cv2.putText(
                frame,
                f"AREA:{int(area)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,0),
                2
            )

    cv2.putText(
        frame,
        f"COLOR : {current_color}",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255,255,255),
        2
    )

    top = np.hstack([
        frame,
        cv2.cvtColor(hsv_mask, cv2.COLOR_GRAY2BGR)
    ])

    bottom = np.hstack([
        cv2.cvtColor(bgr_mask, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(final_mask, cv2.COLOR_GRAY2BGR)
    ])

    display = np.vstack([top, bottom])

    cv2.imshow(
        "TUNER",
        display
    )

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    elif key == ord('1'):
        current_color = "red"
        load_color(current_color)

    elif key == ord('2'):
        current_color = "yellow"
        load_color(current_color)

    elif key == ord('3'):
        current_color = "blue"
        load_color(current_color)

    elif key == ord('p'):

        print("\n" + "=" * 50)

        print(f'"{current_color}": {{')
        print(
            f'    "hsv1": ([{H_LO}, {S_LO}, {V_LO}], '
            f'[{H_HI}, {S_HI}, {V_HI}]),'
        )
        print(
            f'    "bgr": ([{B_LO}, {G_LO}, {R_LO}], '
            f'[{B_HI}, {G_HI}, {R_HI}]),'
        )
        print("},")

        print("=" * 50)

cap.release()
cv2.destroyAllWindows()
```
