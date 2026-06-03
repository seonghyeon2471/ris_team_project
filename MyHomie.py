# integrated_robot_mission.py
# RED -> YELLOW -> BLUE mission with LiDAR avoidance
# Generated from discussion; tune HSV/thresholds on-site.

import cv2
import serial
import threading
import numpy as np
import time

# =========================
# SERIAL
# =========================
arduino_ser = serial.Serial("/dev/serial0",115200,timeout=0.1)
lidar_ser = serial.Serial("/dev/ttyUSB0",460800,timeout=0.1)

# =========================
# CAMERA
# =========================
WIDTH=320
HEIGHT=240

cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS,30)

latest_frame=None

def camera_worker():
    global latest_frame
    while True:
        ret,frame=cap.read()
        if ret:
            latest_frame=frame.copy()

threading.Thread(target=camera_worker,daemon=True).start()

# =========================
# LIDAR
# =========================
scan_data=np.full(360,150.0,dtype=np.float32)

EMA_ALPHA=0.35
FRONT_CHECK_RANGE=45

THRESH_30=40.0
THRESH_20=28.0
THRESH_10=18.0

def apply_ema(angle,new_dist):
    scan_data[angle]=(1-EMA_ALPHA)*scan_data[angle]+EMA_ALPHA*new_dist

def get_front_min():
    idx=np.arange(-FRONT_CHECK_RANGE,FRONT_CHECK_RANGE+1)%360
    return float(np.min(scan_data[idx]))

def choose_avoid_direction():
    left=np.mean(scan_data[1:90])
    right=np.mean(scan_data[271:360])
    return 1 if left>=right else -1

def lidar_worker():
    lidar_ser.write(bytes([0xA5,0x40]))
    time.sleep(2)
    lidar_ser.reset_input_buffer()
    lidar_ser.write(bytes([0xA5,0x20]))
    lidar_ser.read(7)

    while True:
        raw=lidar_ser.read(5)
        if len(raw)!=5:
            continue

        s_flag=raw[0]&0x01

        if ((raw[0]&0x02)>>1)!=(1-s_flag):
            continue

        angle=int(((raw[1]>>1)|(raw[2]<<7))/64.0)%360
        dist=(raw[3]|(raw[4]<<8))/40.0

        if 3<dist<150:
            apply_ema(angle,dist)

threading.Thread(target=lidar_worker,daemon=True).start()

# =========================
# MOTOR
# =========================
def send_cmd(v,w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0,0)

# =========================
# COLORS
# =========================
COLOR_RANGE={
    "red1":(np.array([160,120,120]),np.array([179,255,255])),
    "red2":(np.array([0,120,120]),np.array([10,255,255])),
    "yellow":(np.array([15,100,120]),np.array([35,255,255])),
    "blue":(np.array([95,60,80]),np.array([125,255,255]))
}

def detect_target(frame,target):

    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    if target=="red":
        mask=cv2.inRange(hsv,*COLOR_RANGE["red1"]) | cv2.inRange(hsv,*COLOR_RANGE["red2"])
    else:
        mask=cv2.inRange(hsv,*COLOR_RANGE[target])

    kernel=np.ones((5,5),np.uint8)
    mask=cv2.erode(mask,kernel,1)
    mask=cv2.dilate(mask,kernel,2)

    cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if not cnts:
        return None,mask

    c=max(cnts,key=cv2.contourArea)
    area=cv2.contourArea(c)

    if area<500:
        return None,mask

    x,y,w,h=cv2.boundingRect(c)
    cx=x+w//2
    cy=y+h//2

    return (cx,cy,area,x,y,w,h),mask

# =========================
# FSM
# =========================
mission_order=["red","yellow","blue"]
mission_idx=0

try:

    while True:

        frame=latest_frame

        if frame is None:
            continue

        frame=cv2.flip(frame,1)

        target=mission_order[mission_idx]

        result,mask=detect_target(frame,target)

        front_min=get_front_min()

        # LiDAR priority
        if front_min<THRESH_10:
            d=choose_avoid_direction()
            send_cmd(0.08,d*0.9)
            continue

        elif front_min<THRESH_20:
            d=choose_avoid_direction()
            send_cmd(0.12,d*0.8)
            continue

        elif front_min<THRESH_30:
            d=choose_avoid_direction()
            send_cmd(0.15,d*0.7)
            continue

        if result:

            cx,cy,area,x,y,w,h=result



            error=cx-(WIDTH//2)

            wcmd=np.clip(error*0.006,-0.7,0.7)

            send_cmd(0.18,wcmd)

            area_ratio=area/(WIDTH*HEIGHT)

            if area_ratio > 0.08 and h > 120:

                send_cmd(0.12,0)
                time.sleep(0.7)

                stop_robot()

                print(target,"REACHED")
                time.sleep(1)

                mission_idx+=1

                if mission_idx>=len(mission_order):
                    print("MISSION COMPLETE")
                    stop_robot()
                    break

            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

        else:
            send_cmd(0.10,0.35)

        cv2.putText(frame,f"TARGET:{target}",(10,25),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)

        cv2.imshow("frame",frame)
        cv2.imshow("mask",mask)

        if cv2.waitKey(1)&0xFF==27:
            break

finally:

    stop_robot()

    cap.release()

    try:
        lidar_ser.write(bytes([0xA5,0x25]))
    except:
        pass

    cv2.destroyAllWindows()
