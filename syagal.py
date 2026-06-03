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
# CAMERA (HBVCAMERA V55)
# =========================================
WIDTH  = 640
HEIGHT = 480

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

# =========================================
# CONTROL PARAMETER
# =========================================
MAX_V      = 0.22
MIN_V      = 0.08
MAX_W      = 0.55

KP_ROT     = 0.0018
KP_FORWARD = 0.0012

X_TOL      = 20     # 중심 X 허용 오차 (px)
Y_TOL      = 20     # 중심 Y 허용 오차 (px)
MIN_AREA   = 700    # 인식 최소 면적

PARK_SEC   = 3.0    # 정차 시간 (초)

# =========================================
# 색상별 HSV + BGR 범위
# 조명 환경에 따라 조정 필요
# =========================================
COLOR_CFG = {
    "red": {
        "hsv": ([160, 130, 150], [179, 255, 255]),
        "bgr": ([80,  30,  170], [150, 100, 255]),
        "display_bgr": (0, 0, 255),
    },
    "yellow": {
        "hsv": ([18,  80,  80],  [38,  255, 255]),
        "bgr": ([0,   150, 150], [100, 255, 255]),
        "display_bgr": (0, 200, 255),
    },
    "blue": {
        "hsv": ([95,  80,  60],  [135, 255, 255]),
        "bgr": ([100, 50,  0],   [255, 150, 80]),
        "display_bgr": (255, 100, 0),
    },
}

# 주차 순서
MISSION = ["red", "yellow", "blue"]

# =========================================
# 상태
# =========================================
STATE_TRACK    = "TRACK"
STATE_CENTERED = "CENTERED"
STATE_PARKING  = "PARKING"
STATE_SEARCH   = "SEARCH"
STATE_DONE     = "DONE"

mission_index = 0
state         = STATE_SEARCH
last_seen_x   = WIDTH // 2
park_start    = None

# =========================================
# MOTOR
# =========================================
def send_cmd(v, w):
    v = np.clip(v, -0.3,  0.3)
    w = np.clip(w, -0.8,  0.8)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0, 0)

# =========================================
# 마스크 생성 (HSV AND BGR 이중 필터)
# =========================================
def make_mask(frame, hsv, color_name):
    cfg = COLOR_CFG[color_name]

    # 빨강은 HSV 0도/180도 양쪽 처리
    if color_name == "red":
        lo1 = np.array([0,   100, 60])
        hi1 = np.array([10,  255, 255])
        lo2 = np.array([160, 100, 60])
        hi2 = np.array([180, 255, 255])
        hsv_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lo1, hi1),
            cv2.inRange(hsv, lo2, hi2)
        )
    else:
        lo = np.array(cfg["hsv"][0])
        hi = np.array(cfg["hsv"][1])
        hsv_mask = cv2.inRange(hsv, lo, hi)

    bgr_mask = cv2.inRange(
        frame,
        np.array(cfg["bgr"][0]),
        np.array(cfg["bgr"][1])
    )

    mask = cv2.bitwise_and(hsv_mask, bgr_mask)

    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.erode( mask, kernel, iterations=1)
    mask   = cv2.dilate(mask, kernel, iterations=2)

    return mask

# =========================================
# START
# =========================================
print("=" * 45)
print(f"  주차 미션 시작: {MISSION}")
print(f"  첫 번째 목표: [{MISSION[0]}]")
print("  ESC 키로 종료")
print("=" * 45)

try:
    while True:

        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.flip(frame, 1)
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ── 미션 완료 ──────────────────────────
        if state == STATE_DONE or mission_index >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "MISSION COMPLETE!", (80, HEIGHT//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        target = MISSION[mission_index]
        cfg    = COLOR_CFG[target]
        disp   = cfg["display_bgr"]

        mask      = make_mask(frame, hsv, target)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 십자선 + 중심점
        frame_cx = WIDTH  // 2
        frame_cy = HEIGHT // 2
        cv2.circle(frame, (frame_cx, frame_cy), 7, (0, 255, 255), -1)
        cv2.line(frame, (frame_cx, 0),     (frame_cx, HEIGHT), (0,255,255), 1)
        cv2.line(frame, (0, frame_cy),     (WIDTH, frame_cy),  (0,255,255), 1)

        # ── PARKING 상태: 정차 타이머 ──────────
        if state == STATE_PARKING:
            stop_robot()
            elapsed = time.time() - park_start
            remain  = max(0.0, PARK_SEC - elapsed)
            cv2.putText(frame,
                        f"PARKING [{target}]  {remain:.1f}s",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, disp, 2)
            cv2.putText(frame,
                        f"MISSION {mission_index+1}/{len(MISSION)}",
                        (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)

            if elapsed >= PARK_SEC:
                print(f"✅ [{target}] 정차 완료!")
                mission_index += 1
                if mission_index >= len(MISSION):
                    state = STATE_DONE
                    print("🏁 미션 전체 완료!")
                else:
                    next_t = MISSION[mission_index]
                    state  = STATE_SEARCH
                    last_seen_x = WIDTH // 2
                    print(f"➡️  다음 목표: [{next_t}]")

            cv2.imshow("frame", frame)
            cv2.imshow("mask",  mask)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        # ── 객체 감지 ──────────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                rect         = cv2.minAreaRect(c)
                (cx, cy), (rw, rh), angle = rect
                cx = int(cx)
                cy = int(cy)
                last_seen_x  = cx

                box = np.int32(cv2.boxPoints(rect))
                cv2.drawContours(frame, [box], 0, disp, 2)
                cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)

                error_x = cx - frame_cx
                error_y = frame_cy - cy

                # ── CENTERED → 정차 시작 ───────
                if abs(error_x) < X_TOL and abs(error_y) < Y_TOL:
                    stop_robot()
                    state      = STATE_PARKING
                    park_start = time.time()
                    print(f"🅿️  [{target}] 중심 정렬 완료 → 정차 시작")

                # ── TRACK: 중심으로 이동 ────────
                else:
                    w = -KP_ROT * error_x
                    if abs(error_x) < 80:
                        v = np.clip(
                            KP_FORWARD * abs(error_y),
                            MIN_V, MAX_V
                        )
                    else:
                        v = MIN_V

                    send_cmd(v, w)
                    state = STATE_TRACK

            else:
                stop_robot()
                state = "SMALL"

        # ── LOST: 마지막으로 본 방향으로 탐색 ──
        else:
            if last_seen_x > frame_cx:
                send_cmd(0.05, -0.35)
                state = "SEARCH RIGHT"
            else:
                send_cmd(0.05,  0.35)
                state = "SEARCH LEFT"

        # ── HUD 오버레이 ───────────────────────
        cv2.putText(frame, state,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.putText(frame,
                    f"TARGET: {target}  ({mission_index+1}/{len(MISSION)})",
                    (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, disp, 2)

        cv2.imshow("frame", frame)
        cv2.imshow("mask",  mask)

        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    pass

finally:
    stop_robot()
    cap.release()
    cv2.destroyAllWindows()
    print("SHUTDOWN")
