import cv2
import numpy as np

# =========================================
# CAMERA (LOW LATENCY)
# =========================================
cap = cv2.VideoCapture(
    0,
    cv2.CAP_V4L2
)

cap.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*'MJPG')
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# =========================================
# HSV RANGE
# =========================================
COLOR_RANGES = {
    "RED": (
        np.array([169, 168, 96]),
        np.array([179, 222, 157])
    ),

    "YELLOW": (
        np.array([16, 137, 142]),
        np.array([30, 214, 195])
    ),

    "BLUE": (
        np.array([106, 168, 54]),
        np.array([131, 210, 82])
    )
}

kernel = np.ones((5, 5), np.uint8)

# =========================================
# LOOP
# =========================================
while True:

    # 오래된 프레임 버리기
    cap.grab()
    cap.grab()

    ret, frame = cap.read()

    if not ret:
        continue

    frame = cv2.flip(frame, 1)

    height, width = frame.shape[:2]
    frame_center = width // 2

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # 카메라 중심선
    cv2.line(
        frame,
        (frame_center, 0),
        (frame_center, height),
        (255, 0, 0),
        2
    )

    for color_name, (lower, upper) in COLOR_RANGES.items():

        mask = cv2.inRange(
            hsv,
            lower,
            upper
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            continue

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(largest)

        if area < 300:
            continue

        M = cv2.moments(largest)

        if M["m00"] <= 0:
            continue

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        error_x = cx - frame_center

        x, y, w, h = cv2.boundingRect(largest)

        # 바운딩 박스
        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )

        # 센트로이드
        cv2.circle(
            frame,
            (cx, cy),
            6,
            (0, 0, 255),
            -1
        )

        # 센트로이드 십자선
        cv2.line(
            frame,
            (cx - 10, cy),
            (cx + 10, cy),
            (0, 255, 255),
            2
        )

        cv2.line(
            frame,
            (cx, cy - 10),
            (cx, cy + 10),
            (0, 255, 255),
            2
        )

        # 색상 이름
        cv2.putText(
            frame,
            color_name,
            (x, y - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        # 좌표
        cv2.putText(
            frame,
            f"X:{cx} Y:{cy}",
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2
        )

        # 중심 오차
        cv2.putText(
            frame,
            f"ERR:{error_x}",
            (x, y + h + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2
        )

    cv2.imshow("FRAME", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
