import cv2
import serial
import numpy as np

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.30, 0.30)
    w = np.clip(w, -1.00, 1.00)

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():
    send_cmd(0, 0)

# =========================================
# SPEED
# =========================================
FORWARD_V = 0.15
BACKWARD_V = -0.15

TURN_V = 0.08
TURN_W = 0.80

print("=================================")
print("W : Forward")
print("S : Backward")
print("A : Left Turn")
print("D : Right Turn")
print("Space : Stop")
print("ESC : Exit")
print("=================================")

try:

    while True:

        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.flip(frame, 1)

        key = cv2.waitKey(1) & 0xFF

        # 기본 정지
        v = 0
        w = 0
        state = "STOP"

        # =====================================
        # CONTROL
        # =====================================
        if key == ord('w'):
            v = FORWARD_V
            state = "FORWARD"

        elif key == ord('s'):
            v = BACKWARD_V
            state = "BACKWARD"

        elif key == ord('a'):
            v = TURN_V
            w = -TURN_W
            state = "LEFT"

        elif key == ord('d'):
            v = TURN_V
            w = TURN_W
            state = "RIGHT"

        elif key == 32:  # Space
            stop_robot()
            state = "STOP"

        elif key == 27:  # ESC
            break

        send_cmd(v, w)

        # =====================================
        # HUD
        # =====================================
        cv2.putText(
            frame,
            state,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"V={v:.2f} W={w:.2f}",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.imshow("Robot Camera", frame)

except KeyboardInterrupt:
    pass

finally:
    stop_robot()
    cap.release()
    cv2.destroyAllWindows()
