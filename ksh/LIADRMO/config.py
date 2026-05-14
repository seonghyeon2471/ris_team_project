# =========================
# SERIAL
# =========================

MOTOR_PORT = "/dev/serial0"
MOTOR_BAUD = 115200

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

# =========================
# ROBOT SIZE
# =========================

# 로봇 크기 (m)
ROBOT_LENGTH = 0.20
ROBOT_WIDTH  = 0.20

# 라이다 위치
# 로봇 중심 기준 +면 앞쪽
LIDAR_OFFSET_X = 0.075

# 안전 여유
SAFETY_MARGIN = 0.04

# 회전 시 필요한 최소 여유거리
TURN_CLEARANCE = 180  # mm

# =========================
# ROBOT
# =========================

WHEEL_BASE = 0.17

# =========================
# SPEED
# =========================

FORWARD_SPEED = 0.15

# 최대 회전 속도
MAX_W = 0.8

# 회전 중 최소 전진
MIN_FORWARD_SPEED = 0.05

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

# gap으로 인정할 최소 거리
GAP_DISTANCE_THRESHOLD = 500

# 최소 gap 크기
MIN_GAP_SIZE = 10

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
