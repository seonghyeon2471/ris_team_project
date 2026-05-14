
# =========================
# SERIAL
# =========================
MOTOR_PORT = "/dev/serial0"
MOTOR_BAUD = 115200

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

# =========================
# ROBOT MODEL
# =========================
WHEEL_BASE = 0.17

# =========================
# SPEED (핵심 튜닝 영역)
# =========================
FORWARD_SPEED = 0.18   # 🔥 살짝 낮춰 안정성 확보
MAX_W = 2.3            # 너무 크면 jitter 발생 → 살짝 감소

# =========================
# SAFETY (훨씬 중요)
# =========================
EMERGENCY_DISTANCE = 150   # 🔥 120 → 150 (조기 회피)

SAFE_DISTANCE = 250        # (추가) 완충 구간

# =========================
# LIDAR FILTER (성능 핵심)
# =========================
ANGLE_MIN = -90            # 🔥 -100 → -90 (노이즈 제거)
ANGLE_MAX = 90             # 🔥 전방 집중

MIN_LIDAR_DIST = 100       # 80 → 100 (근거리 noise 제거)
MAX_LIDAR_DIST = 2500

# =========================
# GAP DETECTION (핵심 안정성)
# =========================
GAP_DISTANCE_THRESHOLD = 420   # 450 → 420 (gap 더 잘 잡힘)
MIN_GAP_SIZE = 6               # 8 → 6 (작은 gap도 활용)

# =========================
# MAP (현재 구조에서는 보조용)
# =========================
MAP_SIZE = 400
MAP_RESOLUTION = 0.01
VISIT_DECAY = 0.985   # 🔥 0.98 → 0.985 (기억 더 오래 유지)

# =========================
# GRP / BEHAVIOR WEIGHTS
# =========================
GOAL_BIAS = 2.2       # 🔥 전방 선호 강화
CENTER_BIAS = 0.9     # 약간 증가 (안정성)
RISK_BIAS = 1.6       # 장애물 회피 강화

# =========================
# CONTROL STABILITY (추가 중요)
# =========================
STEERING_GAIN = 1.5    # planner에서 쓰면 안정성 증가
DEADZONE = 0.08        # 작은 흔들림 제거
