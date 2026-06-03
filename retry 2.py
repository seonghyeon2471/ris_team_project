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
# CONTROL
# =========================================
MAX_V = 0.22

MIN_FORWARD = 0.05
MIN_BACKWARD = -0.10

KP_ROT = 0.0045

X_TOL = 15

MIN_AREA = 500

TARGET_AREA = 23000
ARRIVE_MARGIN = 1200

# =========================================
# BLIND SPOT
# =========================================
BLIND_THRESHOLD = 0.75

BLIND_FORWARD_TIME = 20

last_area = 0

blind_counter = 0

# =========================================
# STATE
# =========================================
target_color = "RED"

found_once = False

search_dir = 1
search_timer = 0

last_seen_x = 160

# =========================================
# RED RANGE
# =========================================
lower_red1=np.array([0,70,70])
upper_red1=np.array([12,255,255])

lower_red2=np.array([160,70,70])
upper_red2=np.array([179,255,255])

lower_red_bgr=np.array([40,20,120])
upper_red_bgr=np.array([210,170,255])

# =========================================
# YELLOW RANGE
# =========================================
lower_yellow_hsv=np.array([15,80,80])
upper_yellow_hsv=np.array([40,255,255])

lower_yellow_bgr=np.array([0,120,120])
upper_yellow_bgr=np.array([170,255,255])

# =========================================
# MOTOR
# =========================================
def send_cmd(v,w):

    v=np.clip(v,-0.30,0.30)
    w=np.clip(w,-0.80,0.80)

    arduino_ser.write(
        f"{v:.3f},{-w:.3f}\n".encode()
    )

def stop_robot():

    send_cmd(0,0)

print("MISSION START")

try:

    while True:

        ret,frame=cap.read()

        if not ret:
            continue

        frame=cv2.flip(frame,1)

        HEIGHT,WIDTH=frame.shape[:2]

        frame_cx=WIDTH//2

        hsv=cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # =====================================
        # MASK
        # =====================================
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

        kernel=np.ones((3,3),np.uint8)

        mask=cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        contours,_=cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        state="SEARCH"

        # =====================================
        # TARGET FOUND
        # =====================================
        if contours:

            blind_counter = 0

            found_once=True

            search_timer=0

            c=max(
                contours,
                key=cv2.contourArea
            )

            area=cv2.contourArea(c)

            last_area = area

            if area > MIN_AREA:

                rect=cv2.minAreaRect(c)

                (cx,cy),(rw,rh),angle=rect

                cx=int(cx)

                last_seen_x=cx

                error_x=cx-frame_cx

                w=-KP_ROT*error_x

                distance_error=(
                    TARGET_AREA-area
                )

                v=distance_error*0.000025

                if v>0:

                    v=max(
                        v,
                        MIN_FORWARD
                    )

                else:

                    v=min(
                        v,
                        MIN_BACKWARD
                    )

                # ==================
                # ARRIVED
                # ==================
                if (

                    abs(error_x)<X_TOL and
                    abs(distance_error)<ARRIVE_MARGIN

                ):

                    stop_robot()

                    if target_color=="RED":

                        time.sleep(1)

                        target_color="YELLOW"

                        found_once=False
                        search_timer=0
                        search_dir=1
                        last_seen_x=frame_cx

                        continue

                    else:

                        print(
                            "MISSION COMPLETE"
                        )

                        break

                else:

                    send_cmd(v,w)

                    state="TRACK"

        # =====================================
        # LOST TARGET
        # =====================================
        else:

            blind_counter += 1

            # -------------------------
            # 사각지대 가능
            # -------------------------
            if (

                last_area >
                TARGET_AREA *
                BLIND_THRESHOLD

            ):

                if blind_counter < BLIND_FORWARD_TIME:

                    send_cmd(
                        0.05,
                        0
                    )

                    state="BLIND FORWARD"

                else:

                    stop_robot()

                    state="ASSUME ARRIVED"

            # -------------------------
            # 처음부터 못 찾음
            # -------------------------
            elif not found_once:

                send_cmd(
                    0.06,
                    0.60*search_dir
                )

                state="INIT SEARCH"

                if search_timer>120:

                    search_dir*=-1

                    search_timer=0

                search_timer += 1

            # -------------------------
            # 찾다가 놓침
            # -------------------------
            else:

                if last_seen_x > frame_cx:

                    send_cmd(
                        0.05,
                        -0.45
                    )

                    state="SEARCH RIGHT"

                else:

                    send_cmd(
                        0.05,
                        0.45
                    )

                    state="SEARCH LEFT"

        cv2.putText(
            frame,
            state,
            (20,40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,255,0),
            2
        )

        cv2.putText(
            frame,
            f"TARGET:{target_color}",
            (20,80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0,255,255),
            2
        )

        cv2.imshow(
            "frame",
            frame
        )

        if cv2.waitKey(1)==27:

            break

except KeyboardInterrupt:

    pass

finally:

    stop_robot()

    cap.release()

    cv2.destroyAllWindows()
