import cv2
import serial
import numpy as np
import time
import threading

# ====================================
# SERIAL 연결
# ====================================
arduino_ser = serial.Serial(
    "/dev/serial0",   # 아두이노 연결 포트
    115200,
    timeout=0.05
)

lidar_ser = serial.Serial(
    "/dev/ttyUSB0",   # LiDAR 포트
    460800,
    timeout=0.001
)

# ====================================
# CAMERA 설정
# ====================================
cap = cv2.VideoCapture(
    0,
    cv2.CAP_V4L2
)

# 해상도 낮춰서 속도 확보
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

if not cap.isOpened():

    print("camera fail")

    exit()

# ====================================
# LiDAR 시작 명령
# ====================================
lidar_ser.write(
    bytes([0xA5,0x40])
)

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(
    bytes([0xA5,0x20])
)

# descriptor 제거
lidar_ser.read(7)

# ====================================
# 미션 순서
# red → yellow
# ====================================
MISSION = [

    "red",

    "yellow"

]

mission_idx = 0

# ====================================
# 주행 파라미터
# ====================================

# 회전 gain
KP_ROT = 0.004

# 최소/최대 전진속도
MIN_V = 0.08
MAX_V = 0.18

# 이 면적 이상이면 도착
TARGET_AREA = 22000

# 너무 작은 노이즈 제거
MIN_AREA = 800

# 목표 도달 후 추가 전진
FORWARD_3CM_SPEED = 0.10
FORWARD_3CM_TIME = 0.32

# 탐색 속도
SEARCH_V = 0.12
SEARCH_W = 0.35

# 마지막 본 위치
last_seen_x = 160

# 탐색 회전 방향
search_dir = 1

# 탐색 시작 시간
search_start = time.time()

# ====================================
# COLOR MASK 생성
# ====================================
def make_mask(hsv,target):

    if target=="red":

        # 빨간색은 HSV 양끝 분리됨
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

        # 노란색 범위
        mask=cv2.inRange(

            hsv,

            np.array([15,40,60]),

            np.array([40,255,255])

        )

    # 노이즈 제거
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
# 모터 제어
# ====================================
def send_cmd(v,w):

    arduino_ser.write(

        f"{v:.3f},{-w:.3f}\n".encode()

    )

def stop_robot():

    send_cmd(0,0)

# ====================================
# LiDAR 쓰레드
# 전방 최소거리 계산
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

            # 전방 ±45도 범위
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
# MAIN LOOP
# ====================================
try:

    while True:

        # 미션 끝났으면 종료
        if mission_idx >= len(MISSION):

            break

        target = MISSION[
            mission_idx
        ]

        print(
            "Current target:",
            target
        )

        ret,frame=cap.read()

        if not ret:

            continue

        # 좌우 반전
        frame=cv2.flip(
            frame,
            1
        )

        hsv=cv2.cvtColor(

            frame,

            cv2.COLOR_BGR2HSV

        )

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

        # ====================================
        # 장애물 회피
        # ====================================
        if front_min < 15:

            print(
                "Obstacle"
            )

            send_cmd(
                0.05,
                0.7
            )

            continue

        # ====================================
        # 목표 발견
        # ====================================
        if contours:

            c=max(

                contours,

                key=cv2.contourArea

            )

            area=cv2.contourArea(c)

            if area > MIN_AREA:

                x,y,w_box,h = cv2.boundingRect(c)

                cx = x + w_box//2

                last_seen_x = cx

                err = cx - 160

                # ==========================
                # 목표 도착
                # ==========================
                if area > TARGET_AREA:

                    print(
                        f"{target} reached"
                    )

                    stop_robot()

                    # 조금 더 전진
                    send_cmd(

                        FORWARD_3CM_SPEED,

                        0

                    )

                    time.sleep(

                        FORWARD_3CM_TIME

                    )

                    stop_robot()

                    # 다음 목표
                    mission_idx += 1

                    if mission_idx >= len(MISSION):

                        print(
                            "MISSION COMPLETE"
                        )

                        break

                    # 탐색 상태 초기화
                    last_seen_x = 160

                    search_dir = 1

                    search_start = time.time()

                    # 카메라 잔상 제거
                    for _ in range(8):

                        cap.read()

                    time.sleep(
                        0.5
                    )

                    print(

                        "NEXT TARGET =",

                        MISSION[
                            mission_idx
                        ]

                    )

                    continue

                # 가까울수록 감속
                v = MIN_V + (

                    MAX_V - MIN_V

                )*(

                    1-area/TARGET_AREA

                )

                # 중앙 정렬
                w = -KP_ROT * err

        # ====================================
        # 목표 잃어버림
        # ====================================
        else:

            elapsed = time.time()-search_start

            # 5초마다 방향 변경
            if elapsed > 5:

                search_start=time.time()

                search_dir *= -1

            # 원운동 탐색
            v = SEARCH_V

            w = SEARCH_W * search_dir

        send_cmd(
            v,
            w
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
