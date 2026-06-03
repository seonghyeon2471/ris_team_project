import cv2
import serial
import numpy as np
import time

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
    timeout=0.1
)

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

# =========================================
# LIDAR PARAM
# =========================================
MAX_SPEED = 0.30
MIN_SPEED = 0.09

MAX_W = 0.9

THRESH_30 = 32.0
THRESH_20 = 22.0
THRESH_10 = 12.0

FRONT_CHECK_RANGE = 45

EMA_ALPHA = 0.35
MEDIAN_K = 2

scan_data = np.full(
    360,
    150.0,
    dtype=np.float32
)

# =========================================
# TRACK PARAM
# =========================================
KP_ROT = 0.0045

TRACK_SPEED = 0.10

X_TOL = 15

CENTER_HOLD_TIME = 8

FORWARD_AFTER_CENTER = 0.18

FORWARD_3CM_TIME = 0.45

MIN_AREA = 500

SEARCH_W = 0.35

# =========================================
# STATE
# =========================================
target_color = "RED"

found_once = False

search_dir = 1

search_timer = 0

last_seen_x = 160

center_counter = 0

s_flag = 0

# =========================================
# RED
# =========================================
lower_red1=np.array([0,70,70])
upper_red1=np.array([12,255,255])

lower_red2=np.array([160,70,70])
upper_red2=np.array([179,255,255])

lower_red_bgr=np.array([40,20,120])
upper_red_bgr=np.array([210,170,255])

# =========================================
# YELLOW
# =========================================
lower_yellow_hsv=np.array([15,80,80])
upper_yellow_hsv=np.array([40,255,255])

lower_yellow_bgr=np.array([0,120,120])
upper_yellow_bgr=np.array([170,255,255])

# =========================================
# UTIL
# =========================================
def apply_ema(angle,new_dist):

    scan_data[angle] = (

        (1-EMA_ALPHA)

        * scan_data[angle]

        +

        EMA_ALPHA

        * new_dist

    )

def apply_median_filter():

    filtered=np.empty(
        360,
        dtype=np.float32
    )

    for i in range(360):

        idx=[

            (i+d)%360

            for d in range(
                -MEDIAN_K,
                MEDIAN_K+1
            )

        ]

        filtered[i]=np.median(
            scan_data[idx]
        )

    scan_data[:] = filtered

def get_front_min():

    idx=np.arange(

        -FRONT_CHECK_RANGE,

        FRONT_CHECK_RANGE+1

    ) % 360

    return float(
        np.min(scan_data[idx])
    )

def choose_avoid_direction():

    left=np.mean(
        scan_data[1:90]
    )

    right=np.mean(
        scan_data[271:360]
    )

    return 1 if left>=right else -1

def send_cmd(v,w):

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():

    send_cmd(0,0)

# =========================================
# MAIN
# =========================================
try:

    while True:

        # =========================
        # LIDAR UPDATE
        # =========================
        raw = lidar_ser.read(5)

        s_flag = 0

        if len(raw)==5:

            s_flag = raw[0] & 0x01

            if (

                ((raw[0]&0x02)>>1)

                ==

                (1-s_flag)

                and

                (raw[1]&0x01)==1

            ):

                angle=int(

                    (

                        (raw[1]>>1)

                        |

                        (raw[2]<<7)

                    ) / 64.0

                ) % 360

                dist=(

                    raw[3]

                    |

                    (raw[4]<<8)

                ) / 40.0

                if 3 < dist < 150:

                    apply_ema(
                        angle,
                        dist
                    )

        if s_flag == 1:

            apply_median_filter()

        front_min = get_front_min()

        # =========================
        # OBSTACLE AVOID
        # =========================
        if front_min < THRESH_10:

            d=choose_avoid_direction()

            send_cmd(
                MIN_SPEED,
                d*MAX_W
            )

            continue

        elif front_min < THRESH_20:

            d=choose_avoid_direction()

            send_cmd(
                0.12,
                d*0.8
            )

            continue

        elif front_min < THRESH_30:

            d=choose_avoid_direction()

            send_cmd(
                0.15,
                d*0.7
            )

            continue

        # =========================
        # CAMERA
        # =========================
        ret,frame=cap.read()

        if not ret:
            continue

        frame=cv2.flip(
            frame,
            1
        )

        frame_cx = frame.shape[1]//2

        hsv=cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        if target_color=="RED":

            mask1=cv2.inRange(
                hsv,
                lower_red1,
                upper_red1
            )

            mask2=cv2.inRange(
                hsv,
                lower_red2,
                upper_red2
            )

            hsv_mask=cv2.bitwise_or(
                mask1,
                mask2
            )

            bgr_mask=cv2.inRange(
                frame,
                lower_red_bgr,
                upper_red_bgr
            )

        else:

            hsv_mask=cv2.inRange(
                hsv,
                lower_yellow_hsv,
                upper_yellow_hsv
            )

            bgr_mask=cv2.inRange(
                frame,
                lower_yellow_bgr,
                upper_yellow_bgr
            )

        mask=cv2.bitwise_and(
            hsv_mask,
            bgr_mask
        )

        contours,_=cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:

            c=max(
                contours,
                key=cv2.contourArea
            )

            area=cv2.contourArea(c)

            if area > MIN_AREA:

                rect=cv2.minAreaRect(c)

                (cx,cy),_,_=rect

                cx=int(cx)

                last_seen_x = cx

                error_x = cx-frame_cx

                if abs(error_x)<X_TOL:

                    center_counter += 1

                else:

                    center_counter = 0

                if center_counter > CENTER_HOLD_TIME:

                    stop_robot()

                    time.sleep(0.2)

                    send_cmd(
                        FORWARD_AFTER_CENTER,
                        0
                    )

                    time.sleep(
                        FORWARD_3CM_TIME
                    )

                    stop_robot()

                    time.sleep(1)

                    if target_color=="RED":

                        target_color="YELLOW"

                        center_counter=0

                        continue

                    else:

                        break

                else:

                    w=-KP_ROT*error_x

                    send_cmd(
                        TRACK_SPEED,
                        w
                    )

            else:

                center_counter = 0

        else:

            center_counter = 0

            if last_seen_x < frame_cx:

                send_cmd(
                    0,
                    SEARCH_W
                )

            else:

                send_cmd(
                    0,
                    -SEARCH_W
                )

finally:

    stop_robot()

    lidar_ser.write(
        bytes([0xA5,0x25])
    )

    cap.release()

    lidar_ser.close()

    cv2.destroyAllWindows()
