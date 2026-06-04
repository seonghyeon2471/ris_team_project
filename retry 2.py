import cv2
import numpy as np
import serial
import time

# ==========================
# SERIAL
# ==========================
arduino = serial.Serial('/dev/serial0',115200,timeout=0.1)

# ==========================
# CAMERA
# ==========================
cap = cv2.VideoCapture(0)

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

MOVE_DONE = False

# ==========================
# RED HSV RANGE
# (조절 필요)
# ==========================
lower_red = np.array([150,50,150])
upper_red = np.array([179,255,255])

while True:

    ret, frame = cap.read()

    if not ret:
        continue

    frame = cv2.resize(frame,(FRAME_W,FRAME_H))

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(
        hsv,
        lower_red,
        upper_red
    )

    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

    contours,_ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # 카메라 중심 표시
    cv2.circle(
        frame,
        (CENTER_X,CENTER_Y),
        7,
        (255,0,0),
        -1
    )

    if len(contours) > 0:

        c = max(contours,key=cv2.contourArea)

        area = cv2.contourArea(c)

        if area > 500:

            x,y,w,h = cv2.boundingRect(c)

            # 초록 테두리
            cv2.rectangle(
                frame,
                (x,y),
                (x+w,y+h),
                (0,255,0),
                2
            )

            cx = x + w//2
            cy = y + h//2

            # 물체 중심
            cv2.circle(
                frame,
                (cx,cy),
                6,
                (0,0,255),
                -1
            )

            cv2.line(
                frame,
                (CENTER_X,CENTER_Y),
                (cx,cy),
                (255,255,0),
                2
            )

            error = cx - CENTER_X

            print("error :",error)

            # 중심 맞추기
            if abs(error) > 30:

                if error > 0:

                    arduino.write(b"RIGHT\n")

                else:

                    arduino.write(b"LEFT\n")

            else:

                # 중심 정렬 완료

                if not MOVE_DONE:

                    print("3cm forward")

                    arduino.write(b"FORWARD_3CM\n")

                    time.sleep(1)

                    arduino.write(b"STOP\n")

                    MOVE_DONE = True

    cv2.imshow("Red Tracking",frame)

    cv2.imshow("Mask",mask)

    key = cv2.waitKey(1)

    if key == 27:
        break

arduino.close()

cap.release()

cv2.destroyAllWindows()
