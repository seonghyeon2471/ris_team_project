import cv2
import serial
import numpy as np
import time
import threading

# ====================================
# SERIAL
# ====================================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.05
)

lidar_ser = serial.Serial(
    "/dev/ttyUSB0",
    460800,
    timeout=0.001
)

# ====================================
# CAMERA
# ====================================
cap = cv2.VideoCapture(
    0,
    cv2.CAP_V4L2
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

if not cap.isOpened():

    print("camera fail")

    exit()

# ====================================
# LIDAR START
# ====================================
lidar_ser.write(bytes([0xA5,0x40]))

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(
    bytes([0xA5,0x20])
)

lidar_ser.read(7)

# ====================================
# PARAM
# ====================================
MISSION = [

    "red",

    "yellow"

]

mission_idx = 0

KP_ROT = 0.004

MIN_V = 0.08
MAX_V = 0.18

TARGET_AREA = 22000
MIN_AREA = 800

FORWARD_3CM_SPEED = 0.10
FORWARD_3CM_TIME = 0.32

SEARCH_V = 0.12
SEARCH_W = 0.35

last_seen_x = 160

search_dir = 1

search_start = time.time()

# ====================================
# COLOR
# ====================================
def make_mask(hsv,target):

    if target=="red":

        mask1=cv2.inRange(

            hsv,

            np.array([0,45,50]),

            np.array([15,255,255])

        )

        mask2=cv2.inRange(

            hsv,

            np.array([160,45,50]),

            np.array([179,255,255])

        )

        mask=cv2.bitwise_or(
            mask1,
            mask2
        )

    else:

        mask=cv2.inRange(

            hsv,

            np.array([15,40,60]),

            np.array([40,255,255])

        )

    kernel=np.ones(
        (5,5),
        np.uint8
    )

    mask=cv2.erode(
        mask,
        kernel
    )

    mask=cv2.dilate(
        mask,
        kernel
    )

    return mask

# ====================================
# MOTOR
# ====================================
def send_cmd(v,w):

    arduino_ser.write(

        f"{v:.3f},{-w:.3f}\n".encode()

    )

def stop_robot():

    send_cmd(0,0)

# ====================================
# LIDAR THREAD
# ====================================
front_min = 150

def lidar_loop():

    global front_min

    scan=np.full(
        360,
        150,
        dtype=np.float32
    )

    while True:

        raw=lidar_ser.read(5)

        if len(raw)!=5:

            continue

        angle=int(

            (

                (raw[1]>>1)

                |

                (raw[2]<<7)

            )/64

        )%360

        dist=(

            raw[3]

            |

            (raw[4]<<8)

        )/40

        if 3<dist<150:

            scan[angle]=dist

            idx=np.arange(
                -45,
                46
            )%360

            front_min=np.min(
                scan[idx]
            )

threading.Thread(

target=lidar_loop,

daemon=True

).start()

# ====================================
# MAIN
# ====================================
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

        target=MISSION[
            mission_idx
        ]

        mask=make_mask(
            hsv,
            target
        )

        contours,_=cv2.findContours(

            mask,

            cv2.RETR_EXTERNAL,

            cv2.CHAIN_APPROX_SIMPLE

        )

        v=0
        w=0

        # ========= obstacle =========
        if front_min<15:

            send_cmd(
                0.05,
                0.7
            )

            continue

        # ========= target =========
        if contours:

            c=max(
                contours,
                key=cv2.contourArea
            )

            area=cv2.contourArea(c)

            if area>MIN_AREA:

                x,y,w_box,h=cv2.boundingRect(c)

                cx=x+w_box//2

                last_seen_x=cx

                err=cx-160

                # 목표 도착
                if area>TARGET_AREA:

                    stop_robot()

                    send_cmd(
                        FORWARD_3CM_SPEED,
                        0
                    )

                    time.sleep(
                        FORWARD_3CM_TIME
                    )

                    stop_robot()

                    mission_idx += 1

                    if mission_idx>=len(MISSION):

                        break

                    # 즉시 노랑으로 전환

                    last_seen_x=160

                    search_start=time.time()

                    continue

                v=MIN_V+(

                    MAX_V-MIN_V

                )*(

                    1-area/TARGET_AREA

                )

                w=-KP_ROT*err

        else:

            # 원운동 탐색
            elapsed=time.time()-search_start

            if elapsed>5:

                search_start=time.time()

                search_dir*=-1

            v=SEARCH_V

            w=SEARCH_W*search_dir

            # 원 반경 약 1m
            # r = v / w
            # 0.12 / 0.35 ≈ 0.34m
            # 실제 미끄럼 고려시 ~1m 근처

        send_cmd(
            v,
            w
        )

        cv2.putText(

            frame,

            target,

            (20,40),

            cv2.FONT_HERSHEY_SIMPLEX,

            1,

            (0,255,0),

            2

        )

        cv2.imshow(
            "frame",
            frame
        )

        cv2.imshow(
            "mask",
            mask
        )

        if cv2.waitKey(1)&0xFF==ord('q'):

            break

finally:

    stop_robot()

    lidar_ser.write(
        bytes([0xA5,0x25])
    )

    cap.release()

    lidar_ser.close()

    cv2.destroyAllWindows()
