import cv2
import serial
import numpy as np
import time

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.1
)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

time.sleep(1)

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):

    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():
    send_cmd(0.0, 0.0)

# =========================================
# COLOR CONFIG
# =========================================
COLOR_CFG = {

    "red": {
        "hsv1": ([0,80,120], [10,255,255]),
        "hsv2": ([165,80,120], [179,255,255]),
        "draw": (0,0,255)
    },

    "yellow": {
        "hsv1": ([15,100,180], [40,255,255]),
        "hsv2": None,
        "draw": (0,255,255)
    },

    "blue": {
        "hsv1": ([100,90,70], [130,255,255]),
        "hsv2": None,
        "draw": (255,0,0)
    }
}

# =========================================
# PARAMETERS
# =========================================
TARGET_COLOR = "red"

MIN_AREA = 500

KP_ROT = 0.003

FORWARD_SPEED = 0.15

CENTER_TOL = 10

# =========================================
# MASK
# =========================================
def make_mask(frame, hsv, color_name):

    cfg = COLOR_CFG[color_name]

    lo1 = np.array(cfg["hsv1"][0])
    hi1 = np.array(cfg["hsv1"][1])

    mask = cv2.inRange(hsv, lo1, hi1)

    if cfg["hsv2"] is not None:

        lo2 = np.array(cfg["hsv2"][0])
        hi2 = np.array(cfg["hsv2"][1])

        mask2 = cv2.inRange(hsv, lo2, hi2)

        mask = cv2.bitwise_or(mask, mask2)

    kernel = np.ones((5,5), np.uint8)

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

    return mask

# =========================================
# MAIN
# =========================================
try:

    while True:

        ret, frame = cap.read()

        if not ret:
            continue

        frame = cv2.flip(frame, 1)

        HEIGHT, WIDTH = frame.shape[:2]

        frame_cx = WIDTH // 2

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        mask = make_mask(
            frame,
            hsv,
            TARGET_COLOR
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) > 0:

            c = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(c)

            if area > MIN_AREA:

                M = cv2.moments(c)

                if M["m00"] != 0:

                    cx = int(
                        M["m10"] / M["m00"]
                    )

                    cy = int(
                        M["m01"] / M["m00"]
                    )

                    error_x = cx - frame_cx

                    # 센트로이드 표시
                    cv2.circle(
                        frame,
                        (cx, cy),
                        5,
                        (0,255,0),
                        -1
                    )

                    cv2.line(
                        frame,
                        (frame_cx, 0),
                        (frame_cx, HEIGHT),
                        (255,0,0),
                        2
                    )

                    cv2.putText(
                        frame,
                        f"CX:{cx}",
                        (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0,255,0),
                        2
                    )

                    # ===== 제어 =====

                    if abs(error_x) > CENTER_TOL:

                        v = 0.0
                        w = -KP_ROT * error_x

                    else:

                        v = FORWARD_SPEED
                        w = 0.0

                    send_cmd(v, w)

                else:
                    stop_robot()

            else:
                stop_robot()

        else:

            # 색지를 못 찾으면 탐색 회전
            send_cmd(0.0, 0.8)

        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("USER INTERRUPT")

finally:

    stop_robot()

    cap.release()

    cv2.destroyAllWindows()

    print("SYSTEM SHUTDOWN")
