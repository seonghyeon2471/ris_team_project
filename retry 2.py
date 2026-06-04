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

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,1)
cap.set(cv2.CAP_PROP_AUTO_WB,0)

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
MAX_V = 0.24
MIN_V = 0.10

KP_ROT = 0.003

MIN_AREA = 900

TARGET_AREA = 19000

STOP_AREA = 26000

PARK_SEC = 3

FORWARD_SPEED = 0.10
FORWARD_TIME = 0.32

# =========================================
# COLOR
# =========================================
COLOR_CFG = {

"red":{

"hsv1":([0,70,40],[20,255,255]),

"hsv2":([160,70,40],[179,255,255]),

"draw":(0,0,255)

},

"yellow":{

"hsv1":([15,30,60],[40,255,255]),

"hsv2":([10,0,190],[45,45,255]),

"draw":(0,220,255)

},

"blue":{

"hsv1":([90,45,40],[140,255,255]),

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

state = "SEARCH"

last_seen_x = 160

park_start = None

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

def flush_camera(n=15):

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

        hsv=cv2.cvtColor(

            frame,

            cv2.COLOR_BGR2HSV

        )

        if mission_index >= len(MISSION):

            break

        target = MISSION[
            mission_index
        ]

        if state=="PARKING":

            stop_robot()

            elapsed=time.time()-park_start

            if elapsed>PARK_SEC:

                mission_index += 1

                if mission_index>=len(MISSION):

                    break

                print(

                    "NEXT TARGET:",

                    MISSION[
                        mission_index
                    ]

                )

                flush_camera()

                state="SEARCH"

                last_seen_x=160

            continue

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

                error=cx-160

                last_seen_x=cx

                if area>TARGET_AREA:

                    state="APPROACH"

                if state=="APPROACH":

                    v=0.10

                    w=-KP_ROT*error*0.5

                    # 중심 정렬
                    if abs(error)<15:

                        stop_robot()

                        send_cmd(

                            FORWARD_SPEED,

                            0

                        )

                        time.sleep(

                            FORWARD_TIME

                        )

                        stop_robot()

                        state="PARKING"

                        park_start=time.time()

                else:

                    v=MIN_V + (

                        MAX_V-MIN_V

                    )*(

                        (TARGET_AREA-area)

                        /TARGET_AREA

                    )

                    w=-KP_ROT*error

        else:

            if state=="APPROACH":

                v=0.10

                w=0

            else:

                if last_seen_x<160:

                    v=0.03

                    w=0.65

                else:

                    v=0.03

                    w=-0.65

        send_cmd(
            v,
            w
        )

        cv2.putText(

            frame,

            f"{target} {state}",

            (20,40),

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
