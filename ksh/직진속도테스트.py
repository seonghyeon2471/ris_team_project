import serial
import time

arduino_ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

def send_cmd(v, w):
    arduino_ser.write(f"{v:.3f},{-w:.3f}\n".encode())
    print(f"→ 명령 전송: v = {v:.3f}  (w = {w:.2f})")

print("=== 직진 최대속도 테스트 시작 ===")
print("5초씩 점점 빠르게 직진합니다.")

# 단계별 속도 테스트
speeds = [0.40, 0.55, 0.65, 0.80, 1.00, 1.20]

for target_v in speeds:
    print(f"\n🚀 {target_v} 속도로 5초 직진...")
    send_cmd(target_v, 0.0)
    time.sleep(5.0)
    
    # 중간에 멈추는 신호
    send_cmd(0.0, 0.0)
    time.sleep(1.0)

print("\n테스트 종료! 로봇을 멈췄습니다.")

# 정리
arduino_ser.close()
