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
KP_ROT = 0.003

MIN_AREA = 900

CENTER_X_TOL = 12

FORWARD_SPEED = 0.10

FORWARD_TIME = 0.32

WAIT_AFTER_FORWARD = 1.0

TRACK_V = 0.06

SEARCH_V = 0.05

SEARCH_W = 0.55

# =========================================
# COLOR
# =========================================
COLOR_CFG = {

"red":{

"hsv1":([0,50,25],[25,255,255]),

"hsv2":([155,50,25],[179,255,255]),

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

    cfg=COLOR_CFG[target]

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

        frame = cv2.flip(
            frame,
            1
        )

        hsv = cv2.cvtColor(

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

                error=cx-160

                state="TRACK"

                if abs(error) < CENTER_X_TOL:

                    print(
                        target,
                        "CENTERED"
                    )

                    stop_robot()

                    # ===== 3cm 이동 =====
                    send_cmd(

                        FORWARD_SPEED,

                        0

                    )

                    time.sleep(

                        FORWARD_TIME

                    )

                    stop_robot()

                    # ===== 1초 정지 =====
                    time.sleep(

                        WAIT_AFTER_FORWARD

                    )

                    # ===== 다음 타겟 =====
                    mission_index += 1

                    if mission_index >= len(MISSION):

                        break

                    flush_camera()

                    state="SEARCH"

                    last_seen_x=160

                    print(

                        "NEXT TARGET:",

                        MISSION[
                            mission_index
                        ]

                    )

                    continue

                else:

                    v=TRACK_V

                    w=-KP_ROT*error

        else:

            state="SEARCH"

            if last_seen_x < 160:

                v=SEARCH_V

                w=SEARCH_W

            else:

                v=SEARCH_V

                w=-SEARCH_W

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
