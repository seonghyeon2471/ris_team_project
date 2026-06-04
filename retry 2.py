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
    timeout=0.01
)

# ====================================
# CAMERA
# ====================================
cap = cv2.VideoCapture(
    0,
    cv2.CAP_V4L2
)

# 버퍼 최소화 (지연 감소)
cap.set(
    cv2.CAP_PROP_BUFFERSIZE,
    1
)

cap.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    320
)

cap.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    240
)

if not cap.isOpened():

    print("camera fail")

    exit()

# ====================================
# LiDAR START
# ====================================
lidar_ser.write(
    bytes([0xA5,0x40])
)

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
MIN_AREA = 600

FORWARD_3CM_SPEED = 0.10
FORWARD_3CM_TIME = 0.32

SEARCH_V = 0.10
SEARCH_W = 0.35

last_seen_x = 160

search_dir = 1

search_start = time.time()

# ====================================
# COLOR
# ====================================
def make_mask(hsv,target):

    if target=="red":

        # 빨간색 범위 확대
        mask1 = cv2.inRange(

            hsv,

            np.array([0,70,40]),

            np.array([12,255,255])

        )

        mask2 = cv2.inRange(

            hsv,

            np.array([165,70,40]),

            np.array([179,255,255])

        )

        mask = cv2.bitwise_or(
            mask1,
            mask2
        )

    else:

        mask = cv2.inRange(

            hsv,

            np.array([15,40,60]),

            np.array([40,255,255])

        )

    kernel = np.ones(
        (3,3),
        np.uint8
    )

    mask = cv2.erode(
        mask,
        kernel
    )

    mask = cv2.dilate(
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
# LiDAR THREAD
# ====================================
front_min = 150

def lidar_loop():

    global front_min

    scan = np.full(
        360,
        150,
        dtype=np.float32
    )

    while True:

        raw = lidar_ser.read(5)

        if len(raw) != 5:

            # CPU 독점 방지
            time.sleep(0.001)

            continue

        angle = int(

            (

                (raw[1]>>1)

                |

                (raw[2]<<7)

            )/64

        ) % 360

        dist = (

            raw[3]

            |

            (raw[4]<<8)

        ) / 40

        if 3 < dist < 150:

            scan[angle] = dist

            idx = np.arange(
                -45,
                46
            ) % 360

            front_min = np.min(
                scan[idx]
            )

lidar_thread = threading.Thread(

    target=lidar_loop,

    daemon=True

)

lidar_thread.start()

time.sleep(0.2)

# ====================================
# MAIN
# ====================================
try:

    while True:

        if mission_idx >= len(MISSION):

            print(
                "MISSION COMPLETE"
            )

            break

        target = MISSION[
            mission_idx
        ]

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

        mask = make_mask(
            hsv,
            target
        )

        contours,_ = cv2.findContours(

            mask,

            cv2.RETR_EXTERNAL,

            cv2.CHAIN_APPROX_SIMPLE

        )

        v = 0
        w = 0

        # ==========================
        # 장애물 회피
        # ==========================
        if front_min < 15:

            print(
                "Obstacle"
            )

            send_cmd(
                0.05,
                0.7
            )

            continue

        # ==========================
        # 목표 발견
        # ==========================
        if contours:

            c = max(

                contours,

                key=cv2.contourArea

            )

            area = cv2.contourArea(c)

            if area > MIN_AREA:

                x,y,w_box,h = cv2.boundingRect(c)

                cx = x + w_box//2

                last_seen_x = cx

                err = cx - 160

                # 목표 도착
                if area > TARGET_AREA:

                    print(
                        target,
                        "reached"
                    )

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

                    last_seen_x = 160

                    search_dir = 1

                    search_start = time.time()

                    # 빨간색 잔상 제거
                    for _ in range(8):

                        cap.read()

                    time.sleep(
                        0.5
                    )

                    continue

                # 가까울수록 감속
                v = MIN_V + (

                    MAX_V - MIN_V

                ) * (

                    1-area/TARGET_AREA

                )

                w = -KP_ROT * err

        # ==========================
        # 목표 없음
        # ==========================
        else:

            err = last_seen_x - 160

            # 마지막 방향 먼저 탐색
            if abs(err) > 25:

                v = 0.05

                w = -KP_ROT * err * 2

            else:

                elapsed = time.time()-search_start

                if elapsed > 5:

                    search_start = time.time()

                    search_dir *= -1

                # 원운동 탐색
                v = SEARCH_V

                w = SEARCH_W * search_dir

        send_cmd(
            v,
            w
        )

        cv2.putText(

            frame,

            f"{target} {front_min:.1f}",

            (20,40),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.8,

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

        if cv2.waitKey(1)&0xFF == ord('q'):

            break

finally:

    print(
        "STOP"
    )

    stop_robot()

    lidar_ser.write(
        bytes([0xA5,0x25])
    )

    cap.release()

    lidar_ser.close()

    cv2.destroyAllWindows()
