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

time.sleep(1)

cap.set(
    cv2.CAP_PROP_AUTO_EXPOSURE,
    1
)

cap.set(
    cv2.CAP_PROP_AUTO_WB,
    0
)

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(
    bytes([0xA5,0x40])
)

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(
    bytes([0xA5,0x20])
)

lidar_ser.read(7)

print("LIDAR START")

# =========================================
# FILTER PARAM
# =========================================
EMA_ALPHA = 0.35
MEDIAN_K  = 2

_scan_buf = np.full(
    360,
    150.0,
    dtype=np.float32
)

_scan_shared = np.full(
    360,
    150.0,
    dtype=np.float32
)

scan_lock = threading.Lock()

# =========================================
# CONTROL PARAM
# =========================================
MAX_V = 0.24
MIN_V = 0.10

KP_ROT = 0.003

MIN_AREA = 900

TARGET_AREA = 13000

PARK_SEC = 3.0

APPROACH_DRIVE_SEC = 1.2

SEARCH_TIMEOUT = 2.2

APPROACH_MAX_TIMEOUT = 2.0

FORWARD_3CM_SPEED = 0.10
FORWARD_3CM_TIME  = 0.32

# =========================================
# COLOR CONFIG
# =========================================
COLOR_CFG = {

    "red":{

        "hsv1":([0,45,50],[15,255,255]),

        "hsv2":([160,45,50],[179,255,255]),

        "bgr":([0,0,0],[255,255,255]),

        "draw":(0,0,255)

    },

    "yellow":{

        "hsv1":([15,30,60],[40,255,255]),

        "hsv2":([10,0,190],[45,45,255]),

        "bgr":([0,0,0],[255,255,255]),

        "draw":(0,200,255)

    },

    "blue":{

        "hsv1":([90,45,40],[140,255,255]),

        "hsv2":None,

        "bgr":([0,0,0],[255,255,255]),

        "draw":(255,80,0)

    }

}

MISSION = [

    "red",

    "yellow",

    "blue"

]

# =========================================
# STATE
# =========================================
mission_index = 0

state = "SEARCH"

last_seen_x = 160

park_start = None

search_start_time = None

approach_start_time = None

# =========================================
# LIDAR FUNCTIONS
# =========================================
def _apply_ema(angle,dist):

    _scan_buf[angle] = (

        (1-EMA_ALPHA)

        * _scan_buf[angle]

        +

        EMA_ALPHA*dist

    )

def _apply_median():

    k = MEDIAN_K

    filtered = np.empty(
        360,
        dtype=np.float32
    )

    for i in range(360):

        idx = [

            (i+d)%360

            for d in range(-k,k+1)

        ]

        filtered[i] = np.sort(

            _scan_buf[idx]

        )[k]

    _scan_buf[:] = filtered

def lidar_loop():

    while True:

        raw = lidar_ser.read(5)

        if len(raw)!=5:

            time.sleep(0.001)

            continue

        angle = int(

            (

                (raw[1]>>1)

                |

                (raw[2]<<7)

            )/64

        )%360

        dist = (

            raw[3]

            |

            (raw[4]<<8)

        )/40

        if 3<dist<150:

            _apply_ema(

                angle,

                dist

            )

        if raw[0]&1:

            _apply_median()

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
def make_mask(frame,hsv,target):

    cfg = COLOR_CFG[target]

    lo1,hi1 = map(
        np.array,
        cfg["hsv1"]
    )

    mask = cv2.inRange(
        hsv,
        lo1,
        hi1
    )

    if cfg["hsv2"]:

        lo2,hi2 = map(
            np.array,
            cfg["hsv2"]
        )

        mask |= cv2.inRange(
            hsv,
            lo2,
            hi2
        )

    return mask

def flush_camera(n=10):

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

        frame = cv2.flip(
            frame,
            1
        )

        hsv = cv2.cvtColor(

            frame,

            cv2.COLOR_BGR2HSV

        )

        if mission_index >= len(MISSION):

            break

        target = MISSION[
            mission_index
        ]

        draw = COLOR_CFG[
            target
        ]["draw"]

        if state=="PARKING":

            stop_robot()

            elapsed = time.time()-park_start

            if elapsed>PARK_SEC:

                mission_index +=1

                flush_camera(15)

                state="FORCED_SEARCH"

                search_start_time=time.time()

                last_seen_x=160

            continue

        mask = make_mask(
            frame,
            hsv,
            target
        )

        contours,_ = cv2.findContours(

            mask,

            cv2.RETR_EXTERNAL,

            cv2.CHAIN_APPROX_SIMPLE

        )

        cam_v=0
        cam_w=0

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

                error=cx-160

                last_seen_x=cx

                if area>TARGET_AREA:

                    state="APPROACH"

                if state=="APPROACH":

                    cam_v=0.12

                    cam_w=-KP_ROT*error*0.5

                    if area>24000:

                        stop_robot()

                        send_cmd(

                            FORWARD_3CM_SPEED,

                            0

                        )

                        time.sleep(

                            FORWARD_3CM_TIME

                        )

                        stop_robot()

                        state="PARKING"

                        park_start=time.time()

                else:

                    cam_w=-KP_ROT*error

                    cam_v=MIN_V + (

                        MAX_V-MIN_V

                    )*(

                        (TARGET_AREA-area)

                        /TARGET_AREA

                    )

        else:

            if state=="APPROACH":

                cam_v=0.12

                cam_w=0

            else:

                cam_v=0.03

                cam_w=0.65 if last_seen_x<160 else -0.65

        send_cmd(
            cam_v,
            cam_w
        )

        cv2.putText(

            frame,

            f"{target} {state}",

            (20,40),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.7,

            draw,

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

    print("SYSTEM OFF")
