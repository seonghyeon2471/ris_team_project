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

cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*'MJPG')
)

FRAME_W = 320
FRAME_H = 240

CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

# =========================================
# RED HSV RANGE
# =========================================
LOW_RED1  = np.array([0,120,120])
HIGH_RED1 = np.array([8,255,255])

LOW_RED2  = np.array([170,120,120])
HIGH_RED2 = np.array([179,255,255])

MIN_AREA = 700

KP_ROT  = 0.01
MIN_ROT = 0.25

MOVE_DONE = False

# =========================================
# MOTOR
# =========================================
def send_cmd(v,w):

    v=np.clip(v,-0.3,0.3)
    w=np.clip(w,-0.8,0.8)

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():

    send_cmd(0.0,0.0)

def forward_3cm():

    send_cmd(
        0.12,
        0.0
    )

    time.sleep(0.35)

    stop_robot()

def backward_3cm():

    send_cmd(
        -0.12,
        0.0
    )

    time.sleep(0.35)

    stop_robot()

# =========================================
# MAIN LOOP
# =========================================
try:

    while True:

        ret,frame = cap.read()

        if not ret:
            continue

        frame = cv2.flip(
            frame,
            1
        )

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        mask1 = cv2.inRange(
            hsv,
            LOW_RED1,
            HIGH_RED1
        )

        mask2 = cv2.inRange(
            hsv,
            LOW_RED2,
            HIGH_RED2
        )

        mask = cv2.bitwise_or(
            mask1,
            mask2
        )

        kernel = np.ones(
            (5,5),
            np.uint8
        )

        mask = cv2.erode(
            mask,
            kernel,
            iterations=1
        )

        mask = cv2.dilate(
            mask,
            kernel,
            iterations=2
        )

        contours,_ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 카메라 중심
        cv2.circle(
            frame,
            (CENTER_X,CENTER_Y),
            6,
            (255,0,0),
            -1
        )

        if len(contours) > 0:

            c = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(c)

            if area > MIN_AREA:

                x,y,w,h = cv2.boundingRect(c)

                cx = x + w//2
                cy = y + h//2

                # 초록 박스
                cv2.rectangle(
                    frame,
                    (x,y),
                    (x+w,y+h),
                    (0,255,0),
                    2
                )

                # 물체 중심
                cv2.circle(
                    frame,
                    (cx,cy),
                    6,
                    (0,0,255),
                    -1
                )

                cv2.line(
                    frame,
                    (CENTER_X,CENTER_Y),
                    (cx,cy),
                    (255,255,0),
                    2
                )

                error = cx - CENTER_X

                cv2.putText(
                    frame,
                    f"ERR:{error}",
                    (10,25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0,255,0),
                    2
                )

                print(
                    "CENTER ERROR:",
                    error
                )

                if not MOVE_DONE:

                    # 중심 안맞음
                    if abs(error) > 20:

                        rot = KP_ROT * error

                        if rot > 0:

                            rot = max(
                                rot,
                                MIN_ROT
                            )

                        else:

                            rot = min(
                                rot,
                                -MIN_ROT
                            )

                        send_cmd(
                            0.0,
                            rot
                        )

                    # 중심 맞음
                    else:

                        print(
                            "ALIGN COMPLETE"
                        )

                        forward_3cm()

                        MOVE_DONE = True

                        print(
                            "MISSION COMPLETE"
                        )

                else:

                    stop_robot()

            else:

                stop_robot()

        else:

            stop_robot()

        cv2.imshow(
            "FRAME",
            frame
        )

        cv2.imshow(
            "MASK",
            mask
        )

        key = cv2.waitKey(1)

        if key == 27:
            break

finally:

    stop_robot()

    cap.release()

    cv2.destroyAllWindows()

    arduino_ser.close()
