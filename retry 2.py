import cv2
import serial
import numpy as np
import time
import threading

# ======================================
# SERIAL
# ======================================
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

# ======================================
# CAMERA
# ======================================
cap = cv2.VideoCapture(
    0,
    cv2.CAP_V4L2
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

if not cap.isOpened():

    print("camera fail")

    exit()

# ======================================
# LIDAR START
# ======================================
lidar_ser.write(bytes([0xA5,0x40]))

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(
    bytes([0xA5,0x20])
)

lidar_ser.read(7)

# ======================================
# PARAM
# ======================================
EMA_ALPHA=0.35

scan=np.full(
    360,
    150.0,
    dtype=np.float32
)

KP_ROT=0.0035

MIN_V=0.08
MAX_V=0.18

TARGET_AREA=24000
MIN_AREA=900

SEARCH_V=0.12
SEARCH_W=0.42

FORWARD_3CM_SPEED=0.10
FORWARD_3CM_TIME=0.32

MISSION=[

    "red",

    "yellow"

]

COLOR_CFG={

"red":{

"hsv1":([0,45,50],[15,255,255]),

"hsv2":([160,45,50],[179,255,255])

},

"yellow":{

"hsv1":([15,30,60],[40,255,255]),

"hsv2":None

}

}

# ======================================
# STATE
# ======================================
mission_index=0

state="SEARCH"

last_seen_x=160

wander_start=time.time()

wander_dir=1

# ======================================
# MOTOR
# ======================================
def send_cmd(v,w):

    arduino_ser.write(

        f"{v:.3f},{-w:.3f}\n".encode()

    )

def stop_robot():

    send_cmd(0,0)

# ======================================
# MISSION CHANGE
# ======================================
def next_target():

    global mission_index
    global state
    global last_seen_x
    global wander_start
    global wander_dir

    mission_index += 1

    if mission_index >= len(MISSION):

        return False

    state="SEARCH"

    last_seen_x=160

    wander_start=time.time()

    wander_dir*=-1

    stop_robot()

    time.sleep(1)

    # 카메라 버퍼 비우기
    for _ in range(15):

        cap.grab()

    print(
        "NEXT TARGET:",
        MISSION[mission_index]
    )

    return True

# ======================================
# MASK
# ======================================
def make_mask(hsv,color):

    cfg=COLOR_CFG[color]

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

        mask=cv2.bitwise_or(

            mask,

            cv2.inRange(

                hsv,

                lo2,

                hi2

            )

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

# ======================================
# MAIN
# ======================================
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
            mission_index
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

                last_seen_x=cx

                err=cx-160

                if area>TARGET_AREA:

                    stop_robot()

                    time.sleep(0.2)

                    send_cmd(

                        FORWARD_3CM_SPEED,

                        0

                    )

                    time.sleep(

                        FORWARD_3CM_TIME

                    )

                    stop_robot()

                    if not next_target():

                        break

                    continue

                v=MIN_V+(

                    MAX_V-MIN_V

                )*(

                    1-area/TARGET_AREA

                )

                w=-KP_ROT*err

                state="TRACK"

        else:

            elapsed=time.time()-wander_start

            if elapsed>4:

                wander_start=time.time()

                wander_dir*=-1

            v=SEARCH_V

            w=SEARCH_W*wander_dir

        send_cmd(
            v,
            w
        )

        cv2.putText(

            frame,

            f"TARGET:{target}",

            (10,30),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.7,

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
