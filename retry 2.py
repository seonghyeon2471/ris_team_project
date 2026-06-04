import cv2
import serial
import numpy as np
import time
import threading

# =========================================
# SERIAL
# =========================================
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)

# =========================================
# CAMERA & HARDWARE FIX
# =========================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

time.sleep(1.0)

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20]))
lidar_ser.read(7)

FRONT_CHECK_RANGE = 45
EMA_ALPHA = 0.35
MEDIAN_K = 2

_scan_buf = np.full(360,150.0,dtype=np.float32)
_scan_shared = np.full(360,150.0,dtype=np.float32)
scan_lock = threading.Lock()

MAX_V = 0.24
MIN_V = 0.10

KP_ROT = 0.003

X_TOL = 12
MIN_AREA = 900
TARGET_AREA = 10000

PARK_SEC = 3.0

SEARCH_TIMEOUT = 2.2

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

MISSION = ["red","yellow","blue"]

mission_index = 0
state = "SEARCH"

last_seen_x = 160

park_start = None
search_start_time = None

def _apply_ema(angle,dist_cm):
    if dist_cm <= 0:
        return
    _scan_buf[angle] = (1-EMA_ALPHA)*_scan_buf[angle] + EMA_ALPHA*dist_cm

def _apply_median_filter():
    k = MEDIAN_K
    filtered = np.empty(360,dtype=np.float32)

    for i in range(360):
        idx = [(i+d)%360 for d in range(-k,k+1)]
        filtered[i] = np.sort(_scan_buf[idx])[k]

    _scan_buf[:] = filtered

def _publish_scan():
    with scan_lock:
        _scan_shared[:] = _scan_buf

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)

        if len(raw) != 5:
            continue

        s_flag = raw[0] & 0x01

        if ((raw[0] & 0x02)>>1)!=(1-s_flag):
            continue

        angle = int(((raw[1]>>1)|(raw[2]<<7))/64.0)%360
        dist_cm = (raw[3] | (raw[4]<<8))/40.0

        if 3 < dist_cm < 150:
            _apply_ema(angle,dist_cm)

        if s_flag == 1:
            _apply_median_filter()
            _publish_scan()

threading.Thread(target=lidar_loop,daemon=True).start()

def send_cmd(v,w):
    v = np.clip(v,-0.3,0.3)
    w = np.clip(w,-0.8,0.8)

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():
    send_cmd(0.0,0.0)

def make_mask(frame,hsv,color_name):

    cfg = COLOR_CFG[color_name]

    lo1,hi1 = np.array(cfg["hsv1"][0]),np.array(cfg["hsv1"][1])

    mask = cv2.inRange(hsv,lo1,hi1)

    if cfg["hsv2"] is not None:

        lo2,hi2 = np.array(cfg["hsv2"][0]),np.array(cfg["hsv2"][1])

        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(hsv,lo2,hi2)
        )

    bgr_mask = cv2.inRange(
        frame,
        np.array(cfg["bgr"][0]),
        np.array(cfg["bgr"][1])
    )

    return cv2.bitwise_and(mask,bgr_mask)

def flush_camera_buffer(n=8):
    for _ in range(n):
        cap.grab()

try:

    while True:

        ret, frame = cap.read()

        if not ret:
            continue

        frame = cv2.flip(frame,1)

        HEIGHT, WIDTH = frame.shape[:2]

        frame_cx = WIDTH//2
        frame_cy = HEIGHT//2

        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        if mission_index >= len(MISSION):

            stop_robot()

            cv2.putText(
                frame,
                "ALL COMPLETE",
                (20,HEIGHT//2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,0),
                2
            )

            cv2.imshow("frame",frame)

            if cv2.waitKey(1)&0xFF==27:
                break

            continue

        target = MISSION[mission_index]

        draw = COLOR_CFG[target]["draw"]

        if state == "PARKING":

            stop_robot()

            remain = max(
                0,
                PARK_SEC - (time.time()-park_start)
            )

            cv2.putText(
                frame,
                f"PARK {remain:.1f}",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                draw,
                2
            )

            if time.time()-park_start >= PARK_SEC:

                mission_index += 1

                if mission_index < len(MISSION):

                    state = "FORCED_SEARCH"

                    search_start_time = time.time()

                    flush_camera_buffer(15)

                    last_seen_x = frame_cx

            cv2.imshow("frame",frame)

            if cv2.waitKey(1)&0xFF==27:
                break

            continue

        mask = make_mask(frame,hsv,target)

        contours,_ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        cam_v = 0
        cam_w = 0

        if contours:

            c = max(contours,key=cv2.contourArea)

            area = cv2.contourArea(c)

            if area > MIN_AREA:

                rect = cv2.minAreaRect(c)

                (cx,cy),_,_ = rect

                cx = int(cx)
                cy = int(cy)

                last_seen_x = cx

                error_x = cx - frame_cx

                cv2.drawContours(
                    frame,
                    [np.int32(cv2.boxPoints(rect))],
                    0,
                    draw,
                    2
                )

                cv2.circle(
                    frame,
                    (cx,cy),
                    6,
                    (0,255,0),
                    -1
                )

                if area < TARGET_AREA:

                    cam_v = 0.10

                    cam_w = -KP_ROT*error_x

                else:

                    cam_w = -KP_ROT*error_x*0.6

                    if abs(error_x) > X_TOL:

                        cam_v = 0.05

                    else:

                        stop_robot()

                        time.sleep(0.1)

                        send_cmd(0.10,0.0)

                        time.sleep(0.30)

                        stop_robot()

                        state = "PARKING"

                        park_start = time.time()

                cv2.line(
                    frame,
                    (frame_cx,0),
                    (frame_cx,HEIGHT),
                    (255,255,255),
                    2
                )

                cv2.putText(
                    frame,
                    f"ERR:{error_x}",
                    (20,130),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0,255,0),
                    2
                )

        else:

            if state == "FORCED_SEARCH":

                if time.time()-search_start_time > SEARCH_TIMEOUT:

                    state = "WANDERING"

                cam_v = 0.03
                cam_w = 0.8

            elif state == "WANDERING":

                cam_v = 0.20
                cam_w = 0.0

            else:

                if last_seen_x < frame_cx:

                    cam_v = 0.03
                    cam_w = 0.65

                else:

                    cam_v = 0.03
                    cam_w = -0.65

        send_cmd(cam_v,cam_w)

        cv2.imshow("frame",frame)
        cv2.imshow("mask",mask)

        if cv2.waitKey(1)&0xFF==27:
            break

finally:

    stop_robot()

    cap.release()

    lidar_ser.write(bytes([0xA5,0x25]))

    cv2.destroyAllWindows()
