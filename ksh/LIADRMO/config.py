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

# =========================
# SPEED
# =========================

FORWARD_SPEED = 0.18   # ↓ 안정성 위해 낮춤
MAX_W = 2.5            # ↑ 회전 강화

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

GAP_DISTANCE_THRESHOLD = 400  # 조금 줄임 (더 민감하게)
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
