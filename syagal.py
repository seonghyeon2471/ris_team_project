# (기존 상단 변수 선언부에 추가했다고 가정)
        # APPROACH_DRIVE_SEC = 1.0  # ★ 객체가 화면에서 사라진 후 내부 중심까지 들어갈 추가 전진 시간 (초)

        # ── 객체 감지 및 주행 제어 ──────────────────────────
        if contours:
            c    = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                if state in ["FORCED_SEARCH", "WANDERING"]:
                    print(f"🎯 [{target}] 객체 발견! 추적 모드로 전환합니다.")
                    state = "TRACK"

                rect = cv2.minAreaRect(c)
                (cx, cy), (rw, rh), angle = rect
                cx = int(cx)
                cy = int(cy)
                last_seen_x = cx

                box = np.int32(cv2.boxPoints(rect))
                cv2.drawContours(frame, [box], 0, draw, 2)
                
                error_x = cx - frame_cx

                # [변경] 1단계: 객체가 코앞까지 도달 (Area가 TARGET_AREA를 넘김)
                if area > TARGET_AREA and state == "TRACK":
                    state = "APPROACH"
                    approach_start_time = time.time()
                    print(f"📥 [{target}] 객체 내부 진입 시작 (Area:{int(area)})")

                # 2단계: APPROACH 상태 주행 (객체를 보면서 파고드는 중)
                if state == "APPROACH":
                    cam_w = -KP_ROT * error_x * 0.5 
                    cam_v = 0.12  # 진입 속도
                    cam_state = "APPROACH"

                    # 벽이 없으므로, Area가 화면의 대부분을 가득 채웠을 때(예: 28000 이상) 
                    # 이미 내부에 들어왔다고 판단하고 즉시 주차 상태로 전환
                    if area > 28000: 
                        stop_robot()
                        state      = "PARKING"
                        park_start = time.time()
                        print(f"🅿️  [{target}] 내부 면적 포화 정지 (Area:{int(area)})")
                        continue
                else:
                    # 일반 TRACK 모드 (기존 유지)
                    cam_w     = -KP_ROT * error_x
                    rem_area  = max(0.0, TARGET_AREA - area)
                    cam_v     = MIN_V + (MAX_V - MIN_V) * (rem_area / TARGET_AREA)
                    cam_v     = np.clip(cam_v, MIN_V, MAX_V)
                    cam_state = "TRACK"

                cv2.putText(frame, f"A:{int(area)}",
                            (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            else:
                cam_state = "SMALL"
                stop_robot()

        else:
            # ── [핵심] 벽이 없을 때: 객체 내부로 완전히 들어와서 카메라 시야에서 사라진 경우 ──
            if state == "APPROACH":
                cv2.putText(frame, "APPROACH (INSIDE BLIND)", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                
                # 객체 아래로 완전히 파고들었으므로 조향(w)은 고정하고 곧장 직진합니다.
                cam_v, cam_w = 0.12, 0.0 
                cam_state    = "APPROACH"

                # 시야에서 사라진 순간부터 설정된 시간(APPROACH_DRIVE_SEC, 예: 1.0초) 동안 
                # 추가로 더 전진한 뒤 정확히 내부 가운데서 정지합니다.
                if time.time() - approach_start_time > APPROACH_DRIVE_SEC:
                    stop_robot()
                    state      = "PARKING"
                    park_start = time.time()
                    print(f"🅿️  [{target}] 내부 진입 후 시간 초과 정지 완료!")
                    continue

            # ── 기존 타겟 미감지 상태 (FORCED_SEARCH, WANDERING 등) ──
            elif state == "FORCED_SEARCH":
                if time.time() - search_start_time > SEARCH_TIMEOUT:
                    state = "WANDERING"
                    cam_v, cam_w = 0.15, 0.0
                    cam_state    = "WANDERING"
                else:
                    cam_v, cam_w = 0.0, 0.55  
                    cam_state    = "FORCED_SEARCH"
            
            elif state == "WANDERING":
                cam_v, cam_w = 0.15, 0.0
                cam_state    = "WANDERING"
                
            else:
                cam_state = "SEARCH LEFT" if last_seen_x <= frame_cx else "SEARCH RIGHT"
                if last_seen_x > frame_cx:
                    cam_v, cam_w = 0.04, -0.38  
                else:
                    cam_v, cam_w = 0.04,  0.38

        # ── LiDAR 회피 제어 분기 ──
        # 내부 진입(APPROACH) 중이거나 주차(PARKING) 중일 때는 라이다가 주변 물체를 장애물로 오인하지 않게 bypass
        if state in ["APPROACH", "PARKING"]:
            final_v, final_w, avoid_on = cam_v, cam_w, False
        else:
            final_v, final_w, avoid_on = decide_cmd(cam_v, cam_w, front_min)
            
        send_cmd(final_v, final_w)
