# =========================================
# CAMERA PARAMETERS (★초고속 스캐닝 튜닝)
# =========================================
MAX_V       = 0.24      
MIN_V       = 0.10      
KP_ROT      = 0.003     
X_TOL       = 35        
MIN_AREA    = 500
TARGET_AREA = 15000     
PARK_SEC    = 3.0
APPROACH_DRIVE_SEC = 1.0

# [변경] 제자리 탐색은 짧고 굵게! 2.2초 동안 빠르게 돌고 없으면 바로 시야개척 출발
SEARCH_TIMEOUT = 2.2    

# ... (중간 생략: 기존 코드와 동일) ...

# =========================================
# 카메라 버퍼 flush (★성능 향상을 위해 n값 증가)
# =========================================
def flush_camera_buffer(n=8):  # 3에서 8로 상향하여 잔상 완전 제거
    for _ in range(n):
        cap.grab()

# ... (메인 루프 진입) ...

        # ── PARKING 상태 ──────────────────────────
        if state == "PARKING":
            stop_robot()

            elapsed = time.time() - park_start
            remain   = max(0.0, PARK_SEC - elapsed)

            if elapsed >= PARK_SEC:
                print(f"✅ [{target}] 정차 완료!")
                mission_index += 1
                if mission_index >= len(MISSION):
                    print("🏁 미션 전체 완료!")
                else:
                    # [핵심] 주차 끝나자마자 카메라 버퍼를 싹 비워야 
                    # 다음 루프에서 센서 잔상 때문에 멍청하게 멈춰있는 현상이 사라집니다.
                    flush_camera_buffer(n=8) 
                    
                    state = "FORCED_SEARCH" 
                    search_start_time = time.time()  
                    last_seen_x = 160  # 센터로 초기화
                    print(f"➡️  다음 목표: [{MISSION[mission_index]}] 초고속 탐색 시작")

            cv2.imshow("frame", frame)
            cv2.imshow("mask",  mask)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            continue

        cam_v     = 0.0
        cam_w     = 0.0

        # ── 객체 감지 및 주행 제어 ──────────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                if state in ["FORCED_SEARCH", "WANDERING"]:
                    print(f"🎯 [{target}] 객체 발견! 즉시 추적 모드 전환.")
                    state = "TRACK"

                rect = cv2.minAreaRect(c)
                (cx, cy), (rw, rh), angle = rect
                cx = int(cx)
                last_seen_x = cx
                
                error_x = cx - frame_cx

                if area > TARGET_AREA and state == "TRACK":
                    state = "APPROACH"
                    approach_start_time = time.time()

                if state == "APPROACH":
                    cam_w = -KP_ROT * error_x * 0.5 
                    cam_v = 0.12  
                    if area > 28000: 
                        stop_robot()
                        state      = "PARKING"
                        park_start = time.time()
                        continue
                else:
                    cam_w     = -KP_ROT * error_x
                    rem_area  = max(0.0, TARGET_AREA - area)
                    cam_v     = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                    cam_v     = np.clip(cam_v, MIN_V, MAX_V)

            else:
                stop_robot()

        else:
            # ── [속도 개선 핵심] 타겟 미감지 시 타임아웃 제어 ──
            if state == "APPROACH":
                cam_v, cam_w = 0.12, 0.0 
                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    stop_robot()
                    state      = "PARKING"
                    park_start = time.time()
                    continue

            elif state == "FORCED_SEARCH":
                if time.time() - search_start_time > SEARCH_TIMEOUT:
                    # 2.2초 동안 제자리 돌았는데 안 보이면 딜레이 없이 즉시 탈출 주행
                    state = "WANDERING"
                    print(f"⚠️ 컷! [{target}] 안 보임 -> 맵 크게 돌러 출발")
                    cam_v, cam_w = 0.20, 0.0  # 시야 개척 속도 업 (0.15 -> 0.20)
                else:
                    # 회전 속도를 0.55에서 0.75로 올려서 타겟을 아주 빠르게 서칭합니다.
                    cam_v, cam_w = 0.0, 0.75  
            
            elif state == "WANDERING":
                # 방을 크게 쓰며 빠르게 순항 주행
                cam_v, cam_w = 0.20, 0.0  
                
            else:
                # 일반 서치 상황에서도 턴 속도를 스피디하게 교체
                if last_seen_x > frame_cx:
                    cam_v, cam_w = 0.02, -0.55  
                else:
                    cam_v, cam_w = 0.02,  0.55

        # ── LiDAR 제어 및 모터 전송 (기존과 동일) ──
        if state in ["APPROACH", "PARKING"]:
            final_v, final_w, avoid_on = cam_v, cam_w, False
        else:
            final_v, final_w, avoid_on = decide_cmd(cam_v, cam_w, front_min)
            
        send_cmd(final_v, final_w)
