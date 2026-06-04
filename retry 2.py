import cv2
import serial
import numpy as np
import time

# =====================================
# SERIAL
# =====================================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.1
)

# =====================================
# CAMERA
# =====================================
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

WIDTH  = 320
HEIGHT = 240

CAM_X = WIDTH//2
CAM_Y = HEIGHT//2

# =====================================
# RED HSV
# =====================================
LOW_RED1  = np.array([0,120,120])
HIGH_RED1 = np.array([8,255,255])

LOW_RED2  = np.array([170,120,120])
HIGH_RED2 = np.array([179,255,255])

MIN_AREA = 700
CENTER_TOL = 20

MISSION_DONE = False

# =====================================
# MOTOR
# =====================================
def send_cmd(v,w):

    v=np.clip(v,-0.3,0.3)
    w=np.clip(w,-0.8,0.8)

    cmd=f"{v:.3f},{w:.3f}\n"

    arduino_ser.write(
        cmd.encode()
    )

def stop_robot():

    send_cmd(
        0.0,
        0.0
    )

def move_3cm():

    print("MOVE 3CM")

    send_cmd(
        0.12,
        0.0
    )

    time.sleep(0.35)

    stop_robot()

# =====================================
# MAIN
# =====================================
try:

    while True:

        ret,frame=cap.read()

        if not ret:
            continue

        frame=cv2.flip(
            frame,
            1
        )

        hsv=cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        mask1=cv2.inRange(
            hsv,
            LOW_RED1,
            HIGH_RED1
        )

        mask2=cv2.inRange(
            hsv,
            LOW_RED2,
            HIGH_RED2
        )

        mask=cv2.bitwise_or(
            mask1,
            mask2
        )

        kernel=np.ones(
            (5,5),
            np.uint8
        )

        mask=cv2.erode(
            mask,
            kernel,
            iterations=1
        )

        mask=cv2.dilate(
            mask,
            kernel,
            iterations=2
        )

        contours,_=cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 카메라 중심
        cv2.circle(
            frame,
            (CAM_X,CAM_Y),
            6,
            (255,0,0),
            -1
        )

        if not MISSION_DONE:

            # =================================
            # 빨간색 찾음
            # =================================
            if len(contours)>0:

                c=max(
                    contours,
                    key=cv2.contourArea
                )

                area=cv2.contourArea(c)

                if area>MIN_AREA:

                    x,y,w,h=cv2.boundingRect(c)

                    cx=x+w//2
                    cy=y+h//2

                    error = cx - CAM_X

                    cv2.rectangle(
                        frame,
                        (x,y),
                        (x+w,y+h),
                        (0,255,0),
                        2
                    )

                    cv2.circle(
                        frame,
                        (cx,cy),
                        6,
                        (0,0,255),
                        -1
                    )

                    cv2.line(
                        frame,
                        (CAM_X,CAM_Y),
                        (cx,cy),
                        (255,255,0),
                        2
                    )

                    cv2.putText(
                        frame,
                        f"ERR:{error}",
                        (10,25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0,255,0),
                        2
                    )

                    # 중심 안맞음
                    if abs(error) > CENTER_TOL:

                        if error > 0:

                            send_cmd(
                                0.0,
                                0.55
                            )

                        else:

                            send_cmd(
                                0.0,
                                -0.55
                            )

                    # 중심 맞음 -> 가장 중요
                    else:

                        move_3cm()

                        MISSION_DONE=True

            # =================================
            # 빨간색 없음
            # =================================
            else:

                # 천천히 탐색 전진
                send_cmd(
                    0.05,
                    0.0
                )

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

        if cv2.waitKey(1)==27:
            break

finally:

    stop_robot()

    cap.release()

    cv2.destroyAllWindows()

    arduino_ser.close()
