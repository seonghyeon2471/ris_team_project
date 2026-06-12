# ── STATE ─────────────────────────────────────────────────────────────
mode          = "LIDAR"
mission_idx   = 0
detect_count  = 0

# PARK 상태
park_state    = "SEARCH"
last_seen_x   = 160
last_bottom_y = 0
was_in_bottom = False
park_t        = None

print(f"START | 하단 10% 기준: y > {BOTTOM_10PCT}px")

# ── MAIN LOOP ─────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame  = cv2.flip(frame, 1)
        H, W   = frame.shape[:2]
        cx_mid = W // 2
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scan   = get_scan()
        fm     = front_min(scan)
        adir   = avoid_dir(scan)

        # ── 미션 완료 ─────────────────────────────────────────────────
        if mission_idx >= len(MISSION):
            stop_robot()
            cv2.putText(frame, "ALL DONE", (60, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            cv2.imshow("f", frame); cv2.waitKey(1); continue

        target = MISSION[mission_idx]
        draw   = COLOR_CFG[target]["draw"]

        # ── 공통: 마스크 & 컨투어 ─────────────────────────────────────
        mask = make_mask(frame, hsv, target)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big   = max(cnts, key=cv2.contourArea) if cnts else None
        found = big is not None and cv2.contourArea(big) > MIN_AREA

        # 색지 정보 추출
        if found:
            bx, by_top, bw, bh = cv2.boundingRect(big)
            ox     = bx + bw // 2                   # x축 가로 중심
            by_bot = min(by_top + bh, 239)           # 색지 하단 y
            err_x  = ox - cx_mid

            # 마지막 위치 갱신
            last_seen_x   = ox
            last_bottom_y = by_bot
            was_in_bottom = (by_bot >= BOTTOM_10PCT)

            # 시각화
            cv2.rectangle(frame, (bx, by_top), (bx + bw, by_top + bh), draw, 2)
            cv2.line(frame, (ox, by_top), (ox, by_top + bh), (0, 255, 255), 2)
            cv2.line(frame, (cx_mid, 0), (cx_mid, H), (100, 100, 100), 1)
            cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)
            cv2.putText(frame, f"x_err={err_x}  y={by_bot}",
                        (bx, by_top - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, draw, 1)

        # ══ LIDAR 주행 모드 ═══════════════════════════════════════════
        if mode == "LIDAR":
            if found:
                detect_count += 1
                cv2.putText(frame, f"DETECT [{detect_count}/{DETECT_CONFIRM}]",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw, 1)
            else:
                detect_count = 0

            if detect_count >= DETECT_CONFIRM:
                # PARK 모드 전환
                detect_count  = 0
                mode          = "PARK"
                park_state    = "TRACK"
                last_bottom_y = 0
                was_in_bottom = False
                park_t        = None
                stop_robot()
                print(f"[{target}] 인식 확정 → PARK")
                cv2.imshow("f", frame); cv2.waitKey(1); continue

            # 라이다 장애물 회피
            if fm < THRESH_STOP:
                v, w = 0.09, adir * 0.9
            elif fm < THRESH_TURN:
                v, w = 0.13, adir * 0.7
            elif fm < THRESH_SLOW:
                v, w = 0.18, adir * 0.4
            else:
                v, w = 0.28, 0.0

            send_cmd(v, w)
            cv2.putText(frame, f"LIDAR  front={fm:.0f}cm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
            cv2.imshow("f", frame)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        # ══ PARK 모드 ══════════════════════════════════════════════════

        # ─ PARKING 정차 ───────────────────────────────────────────────
        if park_state == "PARKING":
            stop_robot()
            elapsed = time.time() - park_t
            remain  = max(0.0, PARK_SEC - elapsed)
            cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)
            cv2.putText(frame, f"PARKING [{target}]  {remain:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw, 2)
            cv2.imshow("f", frame)
            
            if elapsed >= PARK_SEC:
                # [수정 포인트 1] 주차 완료 후 다음 미션으로 인덱스 전환
                mission_idx   += 1
                
                # 모든 미션을 다 깼는지 체크 (Index Error 방지)
                if mission_idx >= len(MISSION):
                    print("모든 미션이 끝났습니다.")
                    continue
                
                # [수정 포인트 2] 다음 객체 탐색을 위해 LIDAR로 가지 않고 즉시 SEARCH(제자리 회전) 모드로 진입
                mode           = "PARK"
                park_state     = "SEARCH"
                
                # 이전 타겟의 흔적을 지우기 위해 화면 중심으로 강제 리셋 (특정 방향 우회전 탐색 유도)
                last_seen_x    = cx_mid + 1 
                
                last_bottom_y  = 0
                was_in_bottom  = False
                detect_count   = 0
                print(f"정차 완료 → 다음 타겟 [{MISSION[mission_idx]}] 탐색을 위해 제자리 회전 시작")
                
            cv2.waitKey(1); continue

        # ─ TRACK: 색지 보임 ───────────────────────────────────────────
        if found:
            # [수정 포인트 3] SEARCH(회전) 중에 객체를 발견하면 즉시 회전을 멈추고 TRACK 상태로 전환됨
            if park_state == "SEARCH":
                print(f"[{target}] 탐색 중 발견! 회전을 멈추고 추적 모드(TRACK)로 전환합니다.")
            
            park_state = "TRACK"

            # 전방 장애물 없으면 라이다 회피 끔 → 카메라만으로 조향
            if fm >= THRESH_SLOW:
                w = -KP_ROT * err_x
                v = APPROACH_V
            else:
                # 장애물 있음: 라이다-카메라 블렌딩
                w_cam   = -KP_ROT * err_x
                w_lidar = adir * 0.7
                if fm < THRESH_STOP:
                    v, w = 0.09, w_lidar
                elif fm < THRESH_TURN:
                    v = 0.13
                    w = 0.7 * w_lidar + 0.3 * w_cam
                else:
                    v = 0.18
                    w = 0.3 * w_lidar + 0.7 * w_cam

            send_cmd(v, w)
            cv2.putText(frame, f"TRACK [{target}]  front={fm:.0f}cm  {'LIDAR_OFF' if fm >= THRESH_SLOW else 'LIDAR_ON'}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.48, draw, 1)

        # ─ 색지 사라짐 (또는 탐색 중) ──────────────────────────────────
        else:
            if was_in_bottom:
                # 하단 10%에서 사라짐 → 주차
                park_state = "PARKING"
                park_t     = time.time()
                stop_robot()
                print(f"[{target}] 하단 소실 (last_y={last_bottom_y}px) → {PARK_SEC}s 정차")
            else:
                # [수정 포인트 4] 주차 직후거나 주행 중 놓쳤을 때 제자리 회전 탐색
                park_state = "SEARCH"
                v = 0.0
                # last_seen_x 가 화면 중심보다 크면 우회전, 작으면 좌회전
                w = -1.30 if last_seen_x > cx_mid else 1.30
                send_cmd(v, w)
                cv2.putText(frame, f"SEARCH [{target}]  last_x={last_seen_x}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 0), 1)
            cv2.line(frame, (0, BOTTOM_10PCT), (W, BOTTOM_10PCT), (0, 0, 255), 1)

        cv2.imshow("f", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

except KeyboardInterrupt:
