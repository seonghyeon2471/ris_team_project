import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # 중앙 40x40 영역
    roi = frame[cy-20:cy+20, cx-20:cx+20]

    # 평균 BGR
    b = int(np.mean(roi[:, :, 0]))
    g = int(np.mean(roi[:, :, 1]))
    r = int(np.mean(roi[:, :, 2]))

    # HSV 변환
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    h_val = int(np.mean(hsv_roi[:, :, 0]))
    s_val = int(np.mean(hsv_roi[:, :, 1]))
    v_val = int(np.mean(hsv_roi[:, :, 2]))

    # ==========================
    # 색상 판별
    # ==========================
    color_name = "UNKNOWN"

    if ((h_val <= 10) or (h_val >= 170)) and s_val > 80:
        color_name = "RED"

    elif 15 <= h_val <= 40 and s_val > 80:
        color_name = "YELLOW"

    elif 90 <= h_val <= 140 and s_val > 80:
        color_name = "BLUE"

    # 중앙 표시
    cv2.rectangle(frame,
                  (cx-20, cy-20),
                  (cx+20, cy+20),
                  (0, 255, 0), 2)

    cv2.putText(frame,
                f"RGB: ({r},{g},{b})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2)

    cv2.putText(frame,
                f"HSV: ({h_val},{s_val},{v_val})",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2)

    cv2.putText(frame,
                f"COLOR: {color_name}",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2)

    print(
        f"\rRGB=({r},{g},{b})  HSV=({h_val},{s_val},{v_val})  COLOR={color_name}",
        end=""
    )

    cv2.imshow("Color Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
