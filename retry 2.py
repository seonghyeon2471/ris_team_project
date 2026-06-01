import cv2
import serial
import time
import numpy as np

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
# CAMERA
# =========================================
cap = cv2.VideoCapture(0)

# =========================================
# LIDAR START
# =========================================
lidar_ser.write(bytes([0xA5,0x40]))

time.sleep(2)

lidar_ser.reset_input_buffer()

lidar_ser.write(bytes([0xA5,0x20]))

lidar_ser.read(7)

print("SYSTEM START")

# =========================================
# PARAMETERS
# =========================================
scan_data = np.full(
    360,
    150.0,
    dtype=np.float32
)

EMA_ALPHA = 0.35

FRONT_CHECK_RANGE = 45

THRESH_30 = 35
THRESH_20 = 25
THRESH_10 = 15

FORWARD_SPEED = 0.18
SEARCH_SPEED = 0.10

TURN_GAIN = 0.004

STOP_AREA = 22000

FRAME_W = 320

MAX_LOST = 80

last_dir = 1
lost_count = 0

# =========================================
# MOTOR
# =========================================
def send_cmd(v,w):

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop():

    send_cmd(0,0)

# =========================================
# LIDAR UTIL
# =========================================
def apply_ema(angle,dist):

    scan_data[angle] = \
        (1-EMA_ALPHA)*scan_data[angle] \
        + EMA_ALPHA*dist

def get_front_min():

    idx = np.arange(
        -FRONT_CHECK_RANGE,
        FRONT_CHECK_RANGE+1
    ) % 360

    return float(
        np.min(scan_data[idx])
    )

def choose_avoid_direction():

    left = np.mean(
        scan_data[1:90]
    )

    right = np.mean(
        scan_data[271:360]
    )

    return 1 if left >= right else -1

# =========================================
# MAIN LOOP
# =========================================
try:

    while True:

        # -------------------------
        # LIDAR UPDATE
        # -------------------------
        raw = lidar_ser.read(5)

        if len(raw)==5:

            s_flag = raw[0] & 0x01

            valid = (
                ((raw[0]&0x02)>>1)
                == (1-s_flag)
            )

            if valid:

                angle = int(
                    (
                        (raw[1]>>1)
                        |
                        (raw[2]<<7)
                    )/64.0
                ) % 360

                dist = (
                    raw[3]
                    |
                    (raw[4]<<8)
                ) / 40.0

                if 3 < dist < 150:

                    apply_ema(
                        angle,
                        dist
                    )

        front_min = get_front_min()

        # =====================================
        # 장애물 회피 (최우선)
        # =====================================
        if front_min < THRESH_10:

            direction = choose_avoid_direction()

            v = 0.07

            w = direction * 0.9

            print(
                "VERY CLOSE"
            )

        elif front_min < THRESH_20:

            direction = choose_avoid_direction()

            v = 0.09

            w = direction * 0.8

            print(
                "AVOID"
            )

        elif front_min < THRESH_30:

            direction = choose_avoid_direction()

            v = 0.12

            w = direction * 0.6

            print(
                "WARNING"
            )

        # =====================================
        # CAMERA FOLLOW
        # =====================================
        else:

            ret, frame = cap.read()

            if not ret:

                continue

            frame = cv2.resize(
                frame,
                (320,240)
            )

            hsv = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2HSV
            )

            lower1 = np.array(
                [0,120,70]
            )

            upper1 = np.array(
                [10,255,255]
            )

            lower2 = np.array(
                [170,120,70]
            )

            upper2 = np.array(
                [180,255,255]
            )

            mask1 = cv2.inRange(
                hsv,
                lower1,
                upper1
            )

            mask2 = cv2.inRange(
                hsv,
                lower2,
                upper2
            )

            mask = mask1 + mask2

            contours,_ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            # ---------------------
            # RED FOUND
            # ---------------------
            if len(contours) > 0:

                biggest = max(
                    contours,
                    key=cv2.contourArea
                )

                area = cv2.contourArea(
                    biggest
                )

                if area > 300:

                    lost_count = 0

                    x,y,w_box,h_box = \
                        cv2.boundingRect(
                            biggest
                        )

                    cx = x + w_box//2

                    error = \
                        cx - FRAME_W//2

                    last_dir = \
                        1 if error>=0 else -1

                    cv2.rectangle(
                        frame,
                        (x,y),
                        (x+w_box,y+h_box),
                        (0,255,0),
                        2
                    )

                    if area > STOP_AREA:

                        v = 0
                        w = 0

                        print(
                            "TARGET ARRIVED"
                        )

                    else:

                        w = np.clip(
                            TURN_GAIN*error,
                            -0.5,
                            0.5
                        )

                        v = FORWARD_SPEED

                        print(
                            "FOLLOW"
                        )

                else:

                    lost_count += 1

            # ---------------------
            # RED LOST
            # ---------------------
            else:

                lost_count += 1

                if lost_count < MAX_LOST:

                    v = SEARCH_SPEED

                    w = last_dir * 0.35

                    print(
                        "SEARCH"
                    )

                else:

                    v = 0
                    w = 0

                    print(
                        "TARGET LOST"
                    )

            cv2.imshow(
                "camera",
                frame
            )

            cv2.imshow(
                "mask",
                mask
            )

            if cv2.waitKey(1) & 0xFF == ord('q'):

                break

        send_cmd(v,w)

except KeyboardInterrupt:

    print("STOP")

finally:

    stop()

    cap.release()

    cv2.destroyAllWindows()

    lidar_ser.write(
        bytes([0xA5,0x25])
    )
