import cv2
import serial
import numpy as np
import time
import threading

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.1
)

lidar_ser = serial.Serial(
    "/dev/ttyUSB0",
    460800,
    timeout=0.01
)

# =========================================
# OPENCV OPT
# =========================================
cv2.setUseOptimized(True)
cv2.setNumThreads(4)

# =========================================
# CAMERA
# =========================================
cap = cv2.VideoCapture(
    0,
    cv2.CAP_V4L2
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

cap.set(cv2.CAP_PROP_FPS,30)

cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*'MJPG')
)

time.sleep(1)

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5,0x40]))

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5,0x20]))

lidar_ser.read(7)

# =========================================
# PARAM
# =========================================
CENTER_X_TOL = 12

KP_ROT = 0.004

TRACK_V = 0.12

SEARCH_V = 0.10

SEARCH_W = 0.55

FORWARD_SPEED = 0.10

FORWARD_TIME = 0.32

MIN_AREA = 1000

WAIT_AFTER_FORWARD = 1.0

# =========================================
# COLOR
# 빨간 범위 확대
# =========================================
COLOR_CFG = {

"red":{

"hsv1":([0,40,20],[30,255,255]),

"hsv2":([150,40,20],[179,255,255]),

"draw":(0,0,255)

},

"yellow":{

"hsv1":([15,30,60],[45,255,255]),

"hsv2":None,

"draw":(0,220,255)

},

"blue":{

"hsv1":([90,40,30],[145,255,255]),

"hsv2":None,

"draw":(255,0,0)

}

}

MISSION = [

"red",

"yellow",

"blue"

]

mission_index = 0

last_seen_x = 160

state = "SEARCH"

# =========================================
# LIDAR THREAD
# =========================================
def lidar_loop():

    while True:

        raw = lidar_ser.read(5)

        if len(raw)!=5:

            time.sleep(0.001)

            continue

threading.Thread(

target=lidar_loop,

daemon=True

).start()

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

    send_cmd(0,0)

# =========================================
# MASK
# =========================================
def make_mask(hsv,target):

    cfg = COLOR_CFG[target]

    lo1=np.array(cfg["hsv1"][0])

    hi1=np.array(cfg["hsv1"][1])

    mask=cv2.inRange(

        hsv,

        lo1,

        hi1

    )

    if cfg["hsv2"]:

        lo2=np.array(cfg["hsv2"][0])

        hi2=np.array(cfg["hsv2"][1])

        mask |= cv2.inRange(

            hsv,

            lo2,

            hi2

        )

    return mask

def flush_camera(n=20):

    for _ in range(n):

        cap.grab()

# =========================================
# MAIN
# =========================================
try:

    while True:

        ret,frame = cap.read()

        if not ret:

            continue

        frame=cv2.flip(
            frame,
            1
        )

        HEIGHT,WIDTH,_ = frame.shape

        frame_cx = WIDTH//2

        frame_cy = HEIGHT//2

        hsv=cv2.cvtColor(

            frame,

            cv2.COLOR_BGR2HSV

        )

        if mission_index >= len(MISSION):

            print(
                "MISSION COMPLETE"
            )

            break

        target = MISSION[
            mission_index
        ]

        mask = make_mask(

            hsv,

            target

        )

        contours,_ = cv2.findContours(

            mask,

            cv2.RETR_EXTERNAL,

            cv2.CHAIN_APPROX_SIMPLE

        )

        v=0
        w=0

        # 카메라 중심
        cv2.circle(

            frame,

            (frame_cx,frame_cy),

            5,

            (255,255,255),

            -1

        )

        if contours:

            c=max(

                contours,

                key=cv2.contourArea

            )

            area=cv2.contourArea(c)

            if area>MIN_AREA:

                rect=cv2.minAreaRect(c)

                (cx,cy),_,_=rect

                cx=int(cx)

                cy=int(cy)

                last_seen_x = cx

                error = cx-frame_cx

                box=cv2.boxPoints(
                    rect
                )

                box=np.int32(box)

                # 초록 테두리
                cv2.drawContours(

                    frame,

                    [box],

                    0,

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

                # 연결선
                cv2.line(

                    frame,

                    (frame_cx,frame_cy),

                    (cx,cy),

                    (255,0,0),

                    2

                )

                state="TRACK"

                # 중심 일치
                if abs(error) < CENTER_X_TOL:

                    stop_robot()

                    send_cmd(

                        FORWARD_SPEED,

                        0

                    )

                    time.sleep(

                        FORWARD_TIME

                    )

                    stop_robot()

                    time.sleep(

                        WAIT_AFTER_FORWARD

                    )

                    mission_index += 1

                    flush_camera()

                    state="SEARCH"

                    last_seen_x=160

                    continue

                else:

                    v=TRACK_V

                    w=-KP_ROT*error

        else:

            state="SEARCH"

            if last_seen_x < frame_cx:

                v=SEARCH_V

                w=SEARCH_W

            else:

                v=SEARCH_V

                w=-SEARCH_W

        send_cmd(v,w)

        cv2.putText(

            frame,

            f"{target} {state}",

            (10,30),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.7,

            (0,255,0),

            2

        )

        cv2.putText(

            frame,

            f"OBJ ({cx},{cy})" if contours else "OBJ LOST",

            (10,60),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.5,

            (255,255,255),

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

        if cv2.waitKey(1)&0xFF==27:

            break

finally:

    stop_robot()

    cap.release()

    lidar_ser.write(
        bytes([0xA5,0x25])
    )

    lidar_ser.close()

    cv2.destroyAllWindows()
