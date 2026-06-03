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
    timeout=0.05
)

lidar_ser = serial.Serial(
    "/dev/ttyUSB0",
    460800,
    timeout=0.001
)

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

if not cap.isOpened():

    print("camera fail")

    exit()

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5,0x40]))

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(
    bytes([0xA5,0x20])
)

lidar_ser.read(7)

# =========================================
# PARAM
# =========================================
EMA_ALPHA = 0.35
MEDIAN_K = 2

scan_buf = np.full(
    360,
    150.0,
    dtype=np.float32
)

scan_shared = np.full(
    360,
    150.0,
    dtype=np.float32
)

scan_lock = threading.Lock()

KP_ROT = 0.0035

MAX_V = 0.20
MIN_V = 0.08

TARGET_AREA = 24000
MIN_AREA = 900

SEARCH_RADIUS_TIME = 5.0

FREE_MOVE_V = 0.13
FREE_MOVE_W = 0.42

FORWARD_3CM_SPEED = 0.10
FORWARD_3CM_TIME = 0.32

THRESH_10 = 12

# =========================================
# COLOR
# =========================================
COLOR_CFG = {

"red": {

"hsv1":([0,45,50],[15,255,255]),

"hsv2":([160,45,50],[179,255,255]),

"draw":(0,0,255)

},

"yellow": {

"hsv1":([15,30,60],[40,255,255]),

"hsv2":None,

"draw":(0,255,255)

}

}

MISSION = [

"red",

"yellow"

]

# =========================================
# STATE
# =========================================
mission_index = 0

state = "SEARCH"

last_seen_x = 160

wander_start = None

wander_turn_dir = 1

approach_start = None

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
# LIDAR THREAD
# =========================================
def lidar_thread():

    global scan_buf

    while True:

        raw=lidar_ser.read(5)

        if len(raw)!=5:

            continue

        s_flag=raw[0]&1

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

            scan_buf[angle]=(

                (1-EMA_ALPHA)

                *

                scan_buf[angle]

                +

                EMA_ALPHA

                *

                dist

            )

threading.Thread(

target=lidar_thread,

daemon=True

).start()

# =========================================
# MASK
# =========================================
def make_mask(frame,hsv,color):

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

    return mask

# =========================================
# MAIN
# =========================================
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

        frame_cx=160

        target=MISSION[
            mission_index
        ]

        mask=make_mask(
            frame,
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

                err=cx-frame_cx

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

                    mission_index+=1

                    if mission_index>=len(MISSION):

                        break

                    state="SEARCH"

                    continue

                v=MIN_V + (

                    MAX_V-MIN_V

                )*(

                    1-area/TARGET_AREA

                )

                w=-KP_ROT*err

                state="TRACK"

        else:

            if state!="WANDERING":

                wander_start=time.time()

                wander_turn_dir*=-1

                state="WANDERING"

            elapsed=time.time()-wander_start

            if elapsed>SEARCH_RADIUS_TIME:

                wander_start=time.time()

                wander_turn_dir*=-1

            v=FREE_MOVE_V

            w=FREE_MOVE_W*wander_turn_dir

        send_cmd(v,w)

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
