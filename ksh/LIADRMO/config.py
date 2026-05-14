# =========================
# SERIAL
# =========================

MOTOR_PORT = "/dev/serial0"
MOTOR_BAUD = 115200

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

# =========================
# ROBOT
# =========================

WHEEL_BASE = 0.17

# Robot footprint (meters)
# 앞뒤 20cm, 양옆 20cm 정사각형
ROBOT_LENGTH = 0.20       # 전체 앞뒤 길이
ROBOT_WIDTH  = 0.20       # 전체 좌우 폭

# 라이다는 로봇 앞면 정중앙에서 2.5cm 뒤에 위치
# 로봇 절반 길이 = 0.10m, 라이다 위치 = 앞면 - 0.025m = 중심에서 0.075m 앞
LIDAR_OFFSET_X = 0.075    # 로봇 중심 기준 라이다까지 앞방향 오프셋 (m)

# footprint 반치수
ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2   # 0.10 m
ROBOT_HALF_WIDTH  = ROBOT_WIDTH  / 2   # 0.10 m

# 여유 마진 (m) - 오차 버퍼
FOOTPRINT_MARGIN = 0.03

# =========================
# SPEED
# =========================

FORWARD_SPEED = 0.22
MAX_W = 2.0

# =========================
# SAFETY
# =========================

SAFE_DISTANCE = 0.22
EMERGENCY_DISTANCE = 0.12

# =========================
# LIDAR
# =========================

ANGLE_MIN = -100
ANGLE_MAX = 100

MIN_LIDAR_DIST = 80
MAX_LIDAR_DIST = 2500

# =========================
# GAP
# =========================

GAP_DISTANCE_THRESHOLD = 450
MIN_GAP_SIZE = 8

# =========================
# MAP
# =========================

MAP_RESOLUTION = 0.01
MAP_SIZE = 400

VISIT_DECAY = 0.995

# =========================
# GOAL
# =========================

GOAL_DIRECTION_WEIGHT = 1.8
VISIT_WEIGHT = 1.5
CENTER_WEIGHT = 0.6
