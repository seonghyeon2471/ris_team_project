import cv2
import serial
import numpy as np
import time

# =========================
# SERIAL
# =========================
arduino_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.01
)

lidar_ser = serial.Serial(
    "/dev/ttyUSB0",
    460800,
    timeout=0.001
)

# =========================
# LIDAR START
# =========================
lidar_ser.write(bytes([0xA5,0x40]))

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(
    bytes([0xA5,0x20])
)

lidar_ser.read(7)

# =========================
# CAMERA
# =========================
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

if not cap.isOpened():

    print("camera fail")

    exit()

# =========================
# PARAM
# =========================
scan_data=np.full(
    360,
    150.0,
    dtype=np.float32
)

EMA_ALPHA=0.35
MEDIAN_K=2

THRESH_10=12
THRESH_20=22
THRESH_30=32

MAX_W=0.9

KP_ROT=0.0045

TRACK_SPEED=0.10

MIN_AREA=500

SEARCH_W=0.35

X_TOL=15

CENTER_HOLD_TIME=8

target_color="RED"

center_counter=0

last_seen_x=160

# =========================
# COLOR
# =========================
lower_red1=np.array([0,70,70])
upper_red1=np.array([12,255,255])

lower_red2=np.array([160,70,70])
upper_red2=np.array([179,255,255])

lower_yellow=np.array([15,80,80])
upper_yellow=np.array([40,255,255])

kernel=np.ones((5,5),np.uint8)

# =========================
# UTIL
# =========================
def send_cmd(v,w):

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():

    send_cmd(0,0)

def apply_ema(angle,d):

    scan_data[angle]=(1-EMA_ALPHA)*scan_data[angle]+EMA_ALPHA*d

def front_min():

    idx=np.arange(-45,46)%360

    return np.min(scan_data[idx])

try:

    while True:

        # =====================
        # LIDAR UPDATE
        # =====================
        for _ in range(15):

            raw=lidar_ser.read(5)

            if len(raw)!=5:

                continue

            s_flag=raw[0]&0x01

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

                apply_ema(
                    angle,
                    dist
                )

        fmin=front_min()

        # =====================
        # CAMERA
        # =====================
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

        if target_color=="RED":

            mask=cv2.bitwise_or(

                cv2.inRange(
                    hsv,
                    lower_red1,
                    upper_red1
                ),

                cv2.inRange(
                    hsv,
                    lower_red2,
                    upper_red2
                )

            )

        else:

            mask=cv2.inRange(
                hsv,
                lower_yellow,
                upper_yellow
            )

        mask=cv2.erode(
            mask,
            kernel
        )

        mask=cv2.dilate(
            mask,
            kernel
        )

        contours,_=cv2.findContours(

            mask,

            cv2.RETR_EXTERNAL,

            cv2.CHAIN_APPROX_SIMPLE

        )

        # =====================
        # OBSTACLE FIRST
        # =====================
        if fmin<THRESH_10:

            send_cmd(
                0.08,
                MAX_W
            )

        elif contours:

            c=max(
                contours,
                key=cv2.contourArea
            )

            area=cv2.contourArea(c)

            if area>MIN_AREA:

                x,y,w,h=cv2.boundingRect(c)

                cx=x+w//2

                last_seen_x=cx

                err=cx-160

                send_cmd(

                    TRACK_SPEED,

                    -KP_ROT*err

                )

                cv2.rectangle(

                    frame,

                    (x,y),

                    (x+w,y+h),

                    (0,255,0),

                    2

                )

        else:

            if last_seen_x<160:

                send_cmd(
                    0,
                    SEARCH_W
                )

            else:

                send_cmd(
                    0,
                    -SEARCH_W
                )

        cv2.imshow(
            "camera",
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
