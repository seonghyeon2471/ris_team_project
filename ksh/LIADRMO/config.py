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
FORWARD_SPEED = 0.20
MAX_W = 2.5

# =========================
# SAFETY
# =========================
EMERGENCY_DISTANCE = 120  # mm

# =========================
# LIDAR FILTER
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
MAP_SIZE = 400
MAP_RESOLUTION = 0.01
VISIT_DECAY = 0.98

# =========================
# GRP WEIGHTS
# =========================
GOAL_BIAS = 2.0
CENTER_BIAS = 0.8
RISK_BIAS = 1.5
