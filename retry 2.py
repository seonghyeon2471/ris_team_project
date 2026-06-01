import cv2
import serial
import time
import numpy as np

# ==========================
# SERIAL
# ==========================
arduino_ser = serial.Serial("/dev/serial0",115200)
time.sleep(2)

# ==========================
# MOTOR
# ==========================
def send_cmd(v,w):
    arduino_ser.write(f"{v:.2f},{-w:.2f}\n".encode())

def stop():
    send_cmd(0,0)

# ==========================
# CAMERA
# ==========================
cap = cv2.VideoCapture(0)

FORWARD_SPEED = 0.25
TURN_SPEED = 0.7

print("Camera avoidance start")

try:

    while True:

        ret, frame = cap.read()

        if not ret:
            continue

        frame = cv2.resize(frame,(320,240))

        # BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 검은색 범위
        lower = np.array([0,0,0])
        upper = np.array([180,255,60])

        mask = cv2.inRange(hsv, lower, upper)

        # 중앙 ROI
        roi = mask[120:240,100:220]

        obstacle_pixels = np.sum(roi > 0)

        # ROI 표시
        cv2.rectangle(frame,(100,120),(220,240),(0,255,0),2)

        if obstacle_pixels > 5000:

            v = 0.10
            w = TURN_SPEED

            print("TURN")

        else:

            v = FORWARD_SPEED
            w = 0

            print("FORWARD")

        send_cmd(v,w)

        # 컬러 화면 출력
        cv2.imshow("camera", frame)

        # 마스크도 확인 가능
        cv2.imshow("mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:

    stop()

    cap.release()

    cv2.destroyAllWindows()
