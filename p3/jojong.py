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
    w = np.clip(w, -0.80, 0.80)

    # 네가 쓰던 프로토콜 그대로
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
TURN_W = 1.45

print("=================================")
print("W : Forward")
print("S : Backward")
print("A : Left")
print("D : Right")
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

        v = 0
        w = 0

        # -------------------------
        # WASD CONTROL
        # -------------------------
        if key == ord('w'):
            v = FORWARD_V

        elif key == ord('s'):
            v = BACKWARD_V

        elif key == ord('a'):
            w = TURN_W

        elif key == ord('d'):
            w = -TURN_W

        elif key == 32:      # Space
            stop_robot()
            continue

        elif key == 27:      # ESC
            break

        send_cmd(v*5, w*10)

        cv2.putText(
            frame,
            f"V={v:.2f} W={w:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,255,0),
            2
        )

        cv2.imshow("Robot Camera", frame)

except KeyboardInterrupt:
    pass

finally:
    stop_robot()
    cap.release()
    cv2.destroyAllWindows()
