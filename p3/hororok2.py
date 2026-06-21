#!/usr/bin/env python3
"""
5_Wall_following.py 의 벽 추종 로직을 ROS 없이 구현.
LiDAR 통신·스캔 구조는 기존 코드(serial RPLidar)를 그대로 사용.
모터 명령도 기존 send_cmd() 방식 유지.

[5번 로직 매핑]
  scan_dist  = 0.5 m  →  50.0 cm  (라이다 배열이 cm 단위)
  offset     = 0.2 m  →  20.0 cm
  angle_distance(deg) →  scan[deg]  (0~359 인덱스 = 각도)
  degrees 배열 불필요  →  인덱스가 곧 각도
  전방 ±5°           →  scan[355:360] + scan[0:6]
  측면 ±90°          →  scan[270:360] + scan[0:91]

[상태 (5번 그대로)]
  forward    : 벽 미감지 → 직진
  close      : 벽이 너무 가까움 → 반대 방향 조향
  far        : 벽이 너무 멀음 → 벽 방향 조향
  maintaining: 적정 거리 → θ 각도 보정
  obstacle   : 전방 장애물 → 우회

[파라미터 — 5번 원본과 동일]
  default_speed = 0.15
  default_angle = 0.2
  scan_dist     = 50.0 cm
  offset        = 20.0 cm
  DEFAULT_THETA = 0.28 rad
"""

import serial
import numpy as np
import time
import threading
from math import acos, pi, floor

# ── 방향 상수 ─────────────────────────────────────────────────────────
LEFT  = "LEFT"
RIGHT = "RIGHT"

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR 내부 버퍼 ───────────────────────────────────────────────────
EMA_ALPHA = 0.35
MEDIAN_K  = 2

_scan     = np.full(360, 150.0, dtype=np.float32)
_scan_pub = np.full(360, 150.0, dtype=np.float32)
scan_lock = threading.Lock()

def _ema(a, d):
    if d > 0:
        _scan[a] = (1 - EMA_ALPHA) * _scan[a] + EMA_ALPHA * d

def _median():
    k = MEDIAN_K
    buf = np.empty(360, dtype=np.float32)
    for i in range(360):
        idx = [(i + d) % 360 for d in range(-k, k + 1)]
        buf[i] = np.sort(_scan[idx])[k]
    _scan[:] = buf

def lidar_loop():
    while True:
        raw = lidar_ser.read(5)
        if len(raw) != 5: continue
        sf = raw[0] & 0x01
        if ((raw[0] & 0x02) >> 1) != (1 - sf) or (raw[1] & 0x01) != 1 or (raw[0] >> 2) < 3:
            continue
        angle   = int(((raw[1] >> 1) | (raw[2] << 7)) / 64.0) % 360
        dist_cm = (raw[3] | (raw[4] << 8)) / 40.0
        if 3 < dist_cm < 150: _ema(angle, dist_cm)
        if sf == 1:
            _median()
            with scan_lock: _scan_pub[:] = _scan

threading.Thread(target=lidar_loop, daemon=True).start()

def get_scan():
    with scan_lock: return _scan_pub.copy()

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot():
    send_cmd(0.0, 0.0)

# ══════════════════════════════════════════════════════════════════════
#  WallFollower — 5번 로직을 scan 배열 기반으로 재구현
# ══════════════════════════════════════════════════════════════════════
class WallFollower:
    """
    5번(Wall_following.py)의 클래스 구조·파라미터·알고리즘을 그대로 유지.
    ROS msg 대신 numpy scan 배열(360, cm 단위)을 직접 받는다.

    scan 배열 인덱스 = 각도(°):
      0   = 전방
      90  = 왼쪽
      180 = 후방
      270 = 오른쪽
    """

    # ── 파라미터 (5번 원본과 동일) ────────────────────────────────────
    default_speed = 0.15
    default_angle = 0.2
    scan_dist     = 50.0   # cm (원본 0.5 m)
    offset        = 20.0   # cm (원본 0.2 m)
    DEFAULT_THETA = 0.28   # rad

    def __init__(self, direction: str = LEFT):
        self.DIRECTION = direction
        self.condition = None
        self.speed     = 0.0
        self.angle     = 0.0

        # scan 루프마다 채워지는 임시 버퍼
        self.distance           = []   # 측면 ±90° 이내, scan_dist 이하
        self.obstacle_data_range = []  # 전방 ±5° 장애물 거리
        self.obstacle_data_idx   = []  # 전방 ±5° 장애물 각도 인덱스

    # ── LiDAR_scan (5번 동일 로직, msg 대신 scan 배열) ───────────────
    def LiDAR_scan(self, scan: np.ndarray):
        """
        ±90° 측면 거리 수집 및 전방 ±5° 장애물 감지.
        원본의 self.degrees 룩업을 배열 인덱스로 대체:
          전방 좌측: 0~90°
          전방 우측: 270~359° (= -90~0°)
        """
        # ── 측면 ±90° — scan_dist 이하 값 수집 ──────────────────────
        # 0~90°  (전방~왼쪽)
        for deg in range(0, 91):
            d = float(scan[deg])
            if 0 < d <= self.scan_dist:
                self.distance.append(d)
        # 270~359° (오른쪽~전방, 즉 -90~0°)
        for deg in range(270, 360):
            d = float(scan[deg])
            if 0 < d <= self.scan_dist:
                self.distance.append(d)

        # ── 전방 ±5° — 장애물 감지 ───────────────────────────────────
        # 0~5°
        for deg in range(0, 6):
            d = float(scan[deg])
            if 0 < d < self.scan_dist:
                self.obstacle_data_idx.append(deg)
                self.obstacle_data_range.append(d)
        # 355~359° (= -5~0°)
        for deg in range(355, 360):
            d = float(scan[deg])
            if 0 < d < self.scan_dist:
                self.obstacle_data_idx.append(deg)
                self.obstacle_data_range.append(d)

    # ── judge_distance (5번 동일) ─────────────────────────────────────
    def judge_distance(self):
        if len(self.obstacle_data_range) == 0:
            self.condition = "forward"
            self.speed     = self.default_speed
            if len(self.distance) > 0:
                if min(self.distance) < self.scan_dist - self.offset:
                    self.condition = "close"
                elif (self.scan_dist - self.offset
                      <= max(self.distance)
                      <= self.scan_dist + self.offset):
                    self.condition = "maintaining"
            else:
                self.condition = "far"
        else:
            self.condition = "obstacle"

    # ── angle_distance (5번 동일, scan 배열로 대체) ───────────────────
    def angle_distance(self, scan: np.ndarray, degree: int):
        """
        지정 각도(°) ±1° 범위에서 0.6 m(60 cm) 이내 첫 번째 거리값 반환.
        원본: self.msg.ranges 순회 + self.degrees 비교
        대체: scan[(degree) % 360] 직접 참조
        """
        for delta in range(0, 2):   # degree, degree+1 (원본 range 그대로)
            idx = (degree + delta) % 360
            d = float(scan[idx])
            if 0 < d < 60.0:        # 원본 0.6 m = 60 cm
                return d
        return None

    # ── maintain_direction (5번 동일) ────────────────────────────────
    def maintain_direction(self, scan: np.ndarray):
        """
        70°, 80° 두 거리로 acos(d80/d70) = 벽과의 각도 θ 계산.
        θ < 20° → 앞머리가 벽에 파고드는 중 → 반대로 조향
        θ > 20° → 앞머리가 벽에서 멀어지는 중 → 벽 쪽으로 조향
        """
        if self.DIRECTION == LEFT:
            angle1 = self.angle_distance(scan,  70)   # d70
            angle2 = self.angle_distance(scan,  80)   # d80
        else:  # RIGHT
            angle1 = self.angle_distance(scan, -70 % 360)  # 290°
            angle2 = self.angle_distance(scan, -80 % 360)  # 280°

        if angle1 is None or angle2 is None:
            return  # 데이터 없으면 현재 속도/각도 유지

        try:
            ratio = angle2 / angle1
            ratio = max(-1.0, min(1.0, ratio))  # acos 도메인 보호
            theta  = acos(ratio)
            thetad = theta * 180 / pi
            print(f"maintain θ={thetad:.1f}°")

            if 0 < thetad < 20:
                print("세타가 작습니다")
                if self.DIRECTION == LEFT:
                    self.angle = theta - self.DEFAULT_THETA
                else:
                    self.angle = -(theta - self.DEFAULT_THETA)
                self.speed = self.default_speed

            elif thetad >= 20:
                print("세타가 큽니다.")
                if self.DIRECTION == LEFT:
                    self.angle = -(theta - self.DEFAULT_THETA)
                else:
                    self.angle = theta - self.DEFAULT_THETA
                self.speed = self.default_speed

        except ValueError:
            pass  # acos 실패 시 현재 값 유지

    # ── obstacle_motion (5번 동일) ────────────────────────────────────
    def obstacle_motion(self):
        """전방 장애물 우회 조향. 5번 원본 로직 그대로."""
        print("obstacle_motion")
        angle_incre   = len(self.obstacle_data_idx) / 10 * pi / 180
        obstacle_end_point = self.obstacle_data_idx[-1]
        blank_space   = len(self.obstacle_data_idx) - obstacle_end_point
        turn_angle    = angle_incre * blank_space / 2

        if self.DIRECTION == LEFT:
            self.angle = turn_angle
        else:
            self.angle = -turn_angle
        self.speed = -turn_angle / 10

    # ── move_control (5번 동일) ───────────────────────────────────────
    def move_control(self, scan: np.ndarray):
        if self.condition == "forward":
            self.speed = self.default_speed
            self.angle = 0.0

        elif self.condition == "close":
            print("벽과의 거리가 너무 가깝습니다.")
            if self.DIRECTION == LEFT:
                self.angle = -self.default_angle / (min(self.distance) * 10)
            else:
                self.angle =  self.default_angle / (min(self.distance) * 10)

        elif self.condition == "far":
            print("벽과의 거리가 너무 멉니다.")
            if self.DIRECTION == LEFT:
                self.angle =  self.default_angle * 2
            else:
                self.angle = -self.default_angle * 2

        elif self.condition == "maintaining":
            self.maintain_direction(scan)

        elif self.condition == "obstacle":
            self.obstacle_motion()

        # 버퍼 초기화 (5번 원본과 동일)
        self.obstacle_data_idx   = []
        self.obstacle_data_range = []
        self.distance            = []

    # ── main (5번 main() 동일 흐름) ──────────────────────────────────
    def step(self, scan: np.ndarray):
        """
        매 루프마다 호출. scan 배열(360, cm)을 받아 (v, w)를 반환.
        send_cmd 호출은 외부에서 해도 되고, 여기서 직접 해도 된다.
        """
        self.LiDAR_scan(scan)
        self.judge_distance()
        self.move_control(scan)
        return self.speed, self.angle


# ══════════════════════════════════════════════════════════════════════
#  메인 루프
# ══════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    DIRECTION = LEFT   # LEFT 또는 RIGHT 선택

    follower = WallFollower(direction=DIRECTION)
    print(f"Wall Following 시작 | 방향: {DIRECTION}")

    try:
        while True:
            scan = get_scan()
            v, w = follower.step(scan)
            send_cmd(v, w)
            time.sleep(1 / 30)   # 30 Hz (원본 rospy.Rate(30) 상당)

    except KeyboardInterrupt:
        print("STOP")
    finally:
        stop_robot()
        lidar_ser.write(bytes([0xA5, 0x25]))
