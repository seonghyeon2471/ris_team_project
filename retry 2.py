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

FORWARD_SPEED = 0.20
TURN_GAIN = 0.004
STOP_AREA = 25000     # 가까워졌다고 판단할 면적

FRAME_W = 320
CENTER_TOL = 25

print("RED FOLLOW START")

try:

    while True:

        ret, frame = cap.read()

        if not ret:
            continue

        frame = cv2.resize(frame,(320,240))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 (빨강은 두 구간 필요)
        lower1 = np.array([0,120,70])
        upper1 = np.array([10,255,255])

        lower2 = np.array([170,120,70])
        upper2 = np.array([180,255,255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask = mask1 + mask2

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        v = 0
        w = 0

        if len(contours) > 0:

            biggest = max(contours, key=cv2.contourArea)

            area = cv2.contourArea(biggest)

            if area > 300:

                x,y,width,height = cv2.boundingRect(biggest)

                cx = x + width//2
                error = cx - FRAME_W//2

                cv2.rectangle(
                    frame,
                    (x,y),
                    (x+width,y+height),
                    (0,255,0),
                    2
                )

                cv2.circle(
                    frame,
                    (cx,y+height//2),
                    5,
                    (255,0,0),
                    -1
                )

                # ===== STOP =====
                if area > STOP_AREA:

                    v = 0
                    w = 0

                    print("TARGET REACHED -> STOP")

                else:

                    # ===== 좌우 조향 =====
                    if abs(error) > CENTER_TOL:

                        w = TURN_GAIN * error
                        w = np.clip(w,-0.6,0.6)

                    else:
                        w = 0

                    v = FORWARD_SPEED

                    print(
                        f"FOLLOW area={area:.0f} "
                        f"err={error}"
                    )

        else:

            # 빨강 못 찾으면 정지
            v = 0
            w = 0

            print("NO TARGET")

        send_cmd(v,w)

        cv2.imshow("camera",frame)
        cv2.imshow("mask",mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:

    stop()

    cap.release()

    cv2.destroyAllWindows()
