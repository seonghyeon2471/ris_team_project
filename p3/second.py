import cv2
import numpy as np
import serial
import time

# =========================================
# UART
# =========================================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.1
)

time.sleep(2)

# =========================================
# CAMERA (LOW LATENCY)
# =========================================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*'MJPG')
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# =========================================
# RED HSV
# =========================================
LOWER_RED = np.array([169, 168, 96])
UPPER_RED = np.array([179, 222, 157])

kernel = np.ones((5, 5), np.uint8)

# =========================================
# TRACKING PARAMETER
# =========================================
CENTER_TOL = 20

KP = 0.01

FORWARD_V = 0.15

MAX_W = 1.2

# =========================================
# LOOP
# =========================================
while True:

    cap.grab()
    cap.grab()

    ret, frame = cap.read()

    if not ret:
        continue

    frame = cv2.flip(frame, 1)

    height, width = frame.shape[:2]

    frame_center_x = width // 2
    frame_center_y = height // 2

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # =========================================
    # RED MASK
    # =========================================
    mask = cv2.inRange(
        hsv,
        LOWER_RED,
        UPPER_RED
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

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # =========================================
    # CAMERA CENTER
    # =========================================
    cv2.line(
        frame,
        (frame_center_x, 0),
        (frame_center_x, height),
        (255, 0, 0),
        2
    )

    cv2.circle(
        frame,
        (frame_center_x, frame_center_y),
        6,
        (255, 0, 0),
        -1
    )

    target_found = False

    # =========================================
    # TRACK LARGEST RED OBJECT
    # =========================================
    if contours:

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(largest)

        if area > 300:

            M = cv2.moments(largest)

            if M["m00"] > 0:

                target_found = True

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                error_x = cx - frame_center_x

                x, y, w, h = cv2.boundingRect(largest)

                # =====================================
                # TRACKING CONTROL
                # =====================================
                w_cmd = -KP * error_x

                if w_cmd > MAX_W:
                    w_cmd = MAX_W

                if w_cmd < -MAX_W:
                    w_cmd = -MAX_W

                v_cmd = FORWARD_V

                if abs(error_x) < CENTER_TOL:
                    state = "CENTER"
                elif error_x > 0:
                    state = "RIGHT"
                else:
                    state = "LEFT"

                cmd = f"{v_cmd:.3f},{w_cmd:.3f}\n"
                arduino_ser.write(cmd.encode())

                # =====================================
                # DRAW
                # =====================================
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    frame,
                    (cx, cy),
                    8,
                    (0, 0, 255),
                    -1
                )

                cv2.line(
                    frame,
                    (cx - 10, cy),
                    (cx + 10, cy),
                    (0, 255, 255),
                    2
                )

                cv2.line(
                    frame,
                    (cx, cy - 10),
                    (cx, cy + 10),
                    (0, 255, 255),
                    2
                )

                cv2.line(
                    frame,
                    (frame_center_x, frame_center_y),
                    (cx, cy),
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    f"CX:{cx}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"ERR:{error_x}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    state,
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2
                )

                cv2.putText(
                    frame,
                    f"v:{v_cmd:.2f}",
                    (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    f"w:{w_cmd:.2f}",
                    (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )

    # =========================================
    # NO TARGET
    # =========================================
    if not target_found:

        v_cmd = 0.0
        w_cmd = 0.0

        arduino_ser.write(
            f"{v_cmd:.3f},{w_cmd:.3f}\n".encode()
        )

        cv2.putText(
            frame,
            "SEARCH",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2
        )

    # =========================================
    # DISPLAY
    # =========================================
    cv2.imshow("FRAME", frame)
    cv2.imshow("MASK", mask)

    key = cv2.waitKey(1) & 0xFF

    if key == 27:

        arduino_ser.write(
            b"0.0,0.0\n"
        )

        break

cap.release()

arduino_ser.write(
    b"0.0,0.0\n"
)

arduino_ser.close()

cv2.destroyAllWindows()
