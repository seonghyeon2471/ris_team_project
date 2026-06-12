import cv2
import numpy as np
import time

# =========================
# CAMERA
# =========================
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

time.sleep(1)

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))

# =========================
# TRACKBAR
# =========================
cv2.namedWindow("HSV_Tuner")

for name, maxv in [
    ("H_MIN",179),
    ("H_MAX",179),
    ("S_MIN",255),
    ("S_MAX",255),
    ("V_MIN",255),
    ("V_MAX",255),
]:
    cv2.createTrackbar(name,"HSV_Tuner",0,maxv,lambda x:None)

# 초기값
cv2.setTrackbarPos("H_MAX","HSV_Tuner",179)
cv2.setTrackbarPos("S_MAX","HSV_Tuner",255)
cv2.setTrackbarPos("V_MAX","HSV_Tuner",255)

color_name = "RED"

print("1 : RED")
print("2 : YELLOW")
print("3 : BLUE")
print("s : 현재 HSV 출력")
print("ESC : 종료")

while True:

    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv2.flip(frame,1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("H_MIN","HSV_Tuner")
    h_max = cv2.getTrackbarPos("H_MAX","HSV_Tuner")

    s_min = cv2.getTrackbarPos("S_MIN","HSV_Tuner")
    s_max = cv2.getTrackbarPos("S_MAX","HSV_Tuner")

    v_min = cv2.getTrackbarPos("V_MIN","HSV_Tuner")
    v_max = cv2.getTrackbarPos("V_MAX","HSV_Tuner")

    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])

    mask = cv2.inRange(hsv, lower, upper)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours,_ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    detect_frame = frame.copy()

    largest_area = 0

    for c in contours:

        area = cv2.contourArea(c)

        if area < 300:
            continue

        if area > largest_area:
            largest_area = area

        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int32(box)

        cv2.drawContours(
            detect_frame,
            [box],
            0,
            (0,255,0),
            2
        )

    cv2.putText(
        detect_frame,
        f"COLOR : {color_name}",
        (10,25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0,255,0),
        2
    )

    cv2.putText(
        detect_frame,
        f"AREA : {int(largest_area)}",
        (10,55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0,255,0),
        2
    )

    cv2.imshow("Camera", detect_frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF

    if key == 27:
        break

    elif key == ord('1'):
        color_name = "RED"

        cv2.setTrackbarPos("H_MIN","HSV_Tuner",0)
        cv2.setTrackbarPos("H_MAX","HSV_Tuner",12)
        cv2.setTrackbarPos("S_MIN","HSV_Tuner",60)
        cv2.setTrackbarPos("S_MAX","HSV_Tuner",255)
        cv2.setTrackbarPos("V_MIN","HSV_Tuner",120)
        cv2.setTrackbarPos("V_MAX","HSV_Tuner",255)

    elif key == ord('2'):
        color_name = "YELLOW"

        cv2.setTrackbarPos("H_MIN","HSV_Tuner",18)
        cv2.setTrackbarPos("H_MAX","HSV_Tuner",45)
        cv2.setTrackbarPos("S_MIN","HSV_Tuner",15)
        cv2.setTrackbarPos("S_MAX","HSV_Tuner",255)
        cv2.setTrackbarPos("V_MIN","HSV_Tuner",180)
        cv2.setTrackbarPos("V_MAX","HSV_Tuner",255)

    elif key == ord('3'):
        color_name = "BLUE"

        cv2.setTrackbarPos("H_MIN","HSV_Tuner",95)
        cv2.setTrackbarPos("H_MAX","HSV_Tuner",130)
        cv2.setTrackbarPos("S_MIN","HSV_Tuner",50)
        cv2.setTrackbarPos("S_MAX","HSV_Tuner",255)
        cv2.setTrackbarPos("V_MIN","HSV_Tuner",50)
        cv2.setTrackbarPos("V_MAX","HSV_Tuner",255)

    elif key == ord('s'):

        print()
        print(f"{color_name}")
        print(
            f'"hsv1": ([{h_min}, {s_min}, {v_min}], '
            f'[{h_max}, {s_max}, {v_max}]),'
        )
        print()

cap.release()
cv2.destroyAllWindows()
