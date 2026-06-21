"""
방향 부호 진단 스크립트
────────────────────────
로봇을 들어 올려 바퀴가 바닥에 닿지 않게 한 상태에서 실행하세요.
(모터가 실제로 돌아가니 안전하게 들고 확인하시면 됩니다)

확인할 3가지:
  [TEST 1] avoid_dir()의 adir==1 이 실제로 어느 쪽(좌/우)인지
  [TEST 2] wall_follow() 내부에서 follow_side="L"/"R" 일 때 sign이
           로봇을 실제로 어느 방향으로 돌리는지 (벽 추종 정상 동작 시)
  [TEST 3] target_bias_sign 을 그대로 wall_follow()에 넣었을 때
           +1 / -1 이 로봇을 실제로 어느 방향으로 돌리는지
           (TEST 2와 같은 회전 방향이 나와야 정상)

각 테스트는 몇 초간 모터를 돌리고 콘솔에 "지금 어느 쪽으로 도는지 보고
숫자를 입력하세요" 라고 물어봅니다. 사람이 직접 보고 답하면 됩니다.
"""

import serial
import numpy as np
import time
import threading

# ── SERIAL ────────────────────────────────────────────────────────────
arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)
lidar_ser   = serial.Serial("/dev/ttyUSB0",  460800, timeout=0.1)

# ── LIDAR BOOT ────────────────────────────────────────────────────────
lidar_ser.write(bytes([0xA5, 0x40])); time.sleep(2)
lidar_ser.reset_input_buffer()
lidar_ser.write(bytes([0xA5, 0x20])); lidar_ser.read(7)
print("LIDAR OK")

# ── LIDAR (원본과 동일 로직) ─────────────────────────────────────────
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

def avoid_dir(scan):
    return 1 if np.mean(scan[1:90]) >= np.mean(scan[271:360]) else -1

# ── MOTOR ─────────────────────────────────────────────────────────────
def send_cmd(v, w):
    v = np.clip(v, -0.4, 0.4)
    w = np.clip(w, -1.6, 1.6)
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())

def stop_robot(): send_cmd(0.0, 0.0)


def spin_and_ask(label, w_value, seconds=1.5):
    print(f"\n>>> {label}")
    print(f"    send_cmd(0.0, {w_value:+.2f}) 로 {seconds}초간 제자리 회전합니다. 잘 보세요...")
    time.sleep(1.0)
    t0 = time.time()
    while time.time() - t0 < seconds:
        send_cmd(0.0, w_value)
        time.sleep(0.05)
    stop_robot()
    ans = input("    로봇이 실제로 어느 쪽으로 돌았나요? [L=왼쪽 / R=오른쪽]: ").strip().upper()
    return ans


print("=" * 60)
print(" TEST 1: avoid_dir() 의 좌/우 의미 확인")
print(" (벽/장애물 옆에 로봇을 두고, 한쪽 벽 가까이 대보세요)")
print("=" * 60)
input("준비되면 Enter (지금 라이다 스캔 기준 adir 값을 출력합니다)...")
scan = get_scan()
adir = avoid_dir(scan)
left_mean  = float(np.mean(scan[271:360]))   # L 쪽(265~296도와 겹치는 대역)
right_mean = float(np.mean(scan[1:90]))      # R 쪽(65~96도와 겹치는 대역)
print(f"    scan[1:90] 평균(오른쪽 대역 추정) = {right_mean:.1f} cm")
print(f"    scan[271:360] 평균(왼쪽 대역 추정) = {left_mean:.1f} cm")
print(f"    adir = {adir}  (1이면 scan[1:90] 쪽이 더 멀음=트임)")
print(f"    -> 사람이 보기에 로봇 기준 어느 쪽이 더 트여 있었나요?")
real_open = input("    [L=왼쪽이 트임 / R=오른쪽이 트임]: ").strip().upper()
if real_open == "R" and adir == 1:
    print("    결론: adir==1 은 '오른쪽 트임' 이 맞습니다.")
elif real_open == "L" and adir == -1:
    print("    결론: adir==1 은 '오른쪽 트임' 이 맞습니다 (이번엔 -1=왼쪽 확인).")
elif real_open == "R" and adir == -1:
    print("    결론: adir==1 은 '왼쪽 트임' 입니다. avoid_dir 해석이 반대였습니다.")
elif real_open == "L" and adir == 1:
    print("    결론: adir==1 은 '왼쪽 트임' 입니다. avoid_dir 해석이 반대였습니다.")
else:
    print("    응답을 다시 확인해주세요.")

print("\n" + "=" * 60)
print(" TEST 2: follow_side 의 sign 부호가 실제 회전 방향과 맞는지")
print("=" * 60)
print(" follow_side='L' 일 때 코드 내부 sign=+1, 'R' 일 때 sign=-1 입니다.")
print(" 아래에서 w=+0.5 (sign=+1 가정) 로 돌려봅니다.")
ans2a = spin_and_ask("sign = +1 (follow_side='L' 가정)", +0.5)
print(" 이번엔 w=-0.5 (sign=-1, follow_side='R' 가정) 로 돌려봅니다.")
ans2b = spin_and_ask("sign = -1 (follow_side='R' 가정)", -0.5)
print(f"    sign=+1 -> 실제 {ans2a}쪽 회전 / sign=-1 -> 실제 {ans2b}쪽 회전")

print("\n" + "=" * 60)
print(" TEST 3: target_bias_sign 부호가 실제 회전 방향과 맞는지")
print(" (cx_to_lidar_angle 안 거치고, w 에 직접 부호만 넣어 확인)")
print("=" * 60)
print(" target_bias_sign=+1 (색지가 화면 오른쪽) 가정, w=+0.5 로 돌려봅니다.")
ans3a = spin_and_ask("target_bias_sign = +1 (색지 오른쪽 가정)", +0.5)
print(" target_bias_sign=-1 (색지가 화면 왼쪽) 가정, w=-0.5 로 돌려봅니다.")
ans3b = spin_and_ask("target_bias_sign = -1 (색지 왼쪽 가정)", -0.5)
print(f"    target_bias_sign=+1 -> 실제 {ans3a}쪽 회전 / target_bias_sign=-1 -> 실제 {ans3b}쪽 회전")

print("\n" + "=" * 60)
print(" 최종 요약")
print("=" * 60)
print(f" TEST1) adir==1 의 의미        : real_open={real_open}, adir={adir}")
print(f" TEST2) sign=+1 -> {ans2a}쪽 / sign=-1 -> {ans2b}쪽")
print(f" TEST3) bias=+1 -> {ans3a}쪽 / bias=-1 -> {ans3b}쪽")
print()
print(" -> TEST2 와 TEST3 의 회전 방향이 서로 일치하면 (예: 둘 다 sign/bias 가")
print("    +1일 때 같은 쪽으로 돈다면) target_bias_sign 을 wall_follow() w 에")
print("    그대로 더해도 안전합니다. 일치하지 않으면 부호를 반전(-1 곱)해서")
print("    써야 합니다.")
print(" -> TEST1 에서 adir==1 이 'L(왼쪽 트임)'으로 나왔다면 decide_follow_side()의")
print("    'L' if adir==1 else 'R' 분기는 그대로 두면 되고, 'R(오른쪽 트임)'으로")
print("    나왔다면 'R' if adir==1 else 'L' 로 뒤집어야 합니다.")

stop_robot()
arduino_ser.close()
lidar_ser.write(bytes([0xA5, 0x25]))
lidar_ser.close()
