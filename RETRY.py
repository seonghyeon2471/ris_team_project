# =========================================
# RECOVERY PARAMETERS
# =========================================

RECOVERY_BACK_TIME = 0.28
RECOVERY_ROTATE_TIME = 0.55
RECOVERY_FORWARD_TIME = 0.35

RECOVERY_BACK_SPEED = -0.07
RECOVERY_ROTATE_W = 1.2
RECOVERY_FORWARD_SPEED = 0.08

GAP_THRESHOLD = 0.22

# =========================================
# GAP ANALYSIS
# =========================================

def compute_gap_score(start_angle, end_angle):

    score = 0
    continuous = 0
    max_continuous = 0

    for a in range(start_angle, end_angle + 1):

        idx = normalize_angle(a)

        d = scan_data[idx]

        # 충분히 통과 가능한 공간
        if d > GAP_THRESHOLD:

            continuous += 1
            score += d

        else:

            max_continuous = max(
                max_continuous,
                continuous
            )

            continuous = 0

    max_continuous = max(
        max_continuous,
        continuous
    )

    # 연속 공간 길이 가중치
    score += max_continuous * 0.35

    return score

# =========================================
# RECOVERY
# =========================================

def emergency_escape():

    global is_backtracking

    print("SMART RECOVERY")

    is_backtracking = True

    # =====================================
    # STEP 1 : BACKWARD
    # =====================================

    send_cmd(
        RECOVERY_BACK_SPEED,
        0.0,
        record=False
    )

    time.sleep(RECOVERY_BACK_TIME)

    stop_robot()

    time.sleep(0.08)

    # =====================================
    # STEP 2 : SPACE ANALYSIS
    # =====================================

    left_score = compute_gap_score(
        25,
        90
    )

    right_score = compute_gap_score(
        -90,
        -25
    )

    print(
        f"LEFT:{left_score:.2f} | "
        f"RIGHT:{right_score:.2f}"
    )

    # =====================================
    # STEP 3 : SELECT ESCAPE DIRECTION
    # =====================================

    if left_score > right_score:

        rotate_w = RECOVERY_ROTATE_W

        print("ESCAPE LEFT")

    else:

        rotate_w = -RECOVERY_ROTATE_W

        print("ESCAPE RIGHT")

    # =====================================
    # STEP 4 : FORCED ROTATION
    # =====================================

    send_cmd(
        0.02,
        rotate_w,
        record=False
    )

    time.sleep(RECOVERY_ROTATE_TIME)

    stop_robot()

    time.sleep(0.05)

    # =====================================
    # STEP 5 : SHORT FORWARD ESCAPE
    # =====================================

    send_cmd(
        RECOVERY_FORWARD_SPEED,
        0.0,
        record=False
    )

    time.sleep(RECOVERY_FORWARD_TIME)

    stop_robot()

    # =====================================
    # STEP 6 : RESET
    # =====================================

    time.sleep(0.15)

    is_backtracking = False

    print("RECOVERY END")
