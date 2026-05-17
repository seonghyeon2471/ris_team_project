import asyncio
from rplidarc1 import RPLidarC1
import serial
import time

# ====================== 설정 ======================
LIDAR_PORT = '/dev/ttyUSB0'
ARDUINO_PORT = '/dev/serial0'      # ← Raspberry Pi GPIO UART 포트 (확인 완료!)

BAUD_LIDAR = 460800
BAUD_ARDUINO = 115200

# 속도 설정 (현장에서 테스트하면서 조정하세요)
V_FORWARD = 0.35      # 직진 속도 (m/s) - 너무 빠르면 0.30으로 낮추기
W_TURN    = 1.8       # 회전 각속도 (rad/s) - 좌/우 회전 강도
V_SLOW    = 0.25      # 장애물 가까울 때 느린 속도

TIME_LIMIT = 58.0     # 60초 제한 (안전 마진)
# ================================================

ser = None

def send_command(v: float, w: float):
    """Arduino로 v, w 명령 전송"""
    global ser
    cmd = f"{v:.2f},{w:.2f}\n"
    try:
        ser.write(cmd.encode('utf-8'))
        ser.flush()
        # print(f"→ sent: {cmd.strip()}")   # 디버그 필요하면 주석 해제
    except Exception as e:
        print("Serial 전송 에러:", e)

async def main():
    global ser
    # Arduino 연결
    ser = serial.Serial(ARDUINO_PORT, BAUD_ARDUINO, timeout=0.1)
    print(f"🔌 Arduino 연결 성공: {ARDUINO_PORT}")

    # LiDAR 연결
    lidar = RPLidarC1(LIDAR_PORT, baudrate=BAUD_LIDAR)
    await lidar.connect()
    health = await lidar.healthcheck()
    print("🩺 LiDAR Health:", health)
    print("🚀 장애물 회피 주행 시작! (Ctrl+C 또는 58초 후 자동 정지)")

    start_time = time.time()

    try:
        async for scan in lidar.simple_scan():
            elapsed = time.time() - start_time
            if elapsed > TIME_LIMIT:
                print("⏰ 시간 제한 도달 → 정지")
                break

            # 유효한 데이터만 필터링
            valid = [p for p in scan if 200 < p["d_mm"] < 2000 and p["q"] > 10]

            # 3구역으로 나누기
            left  = [p["d_mm"] for p in valid if  40 < p["a_deg"] < 150]
            front = [p["d_mm"] for p in valid if -35 < p["a_deg"] <  35]
            right = [p["d_mm"] for p in valid if-150 < p["a_deg"] < -40]

            l_min = min(left)  if left  else 9999
            f_min = min(front) if front else 9999
            r_min = min(right) if right else 9999

            # ==================== 회피 로직 ====================
            if f_min < 400:                     # 정면 40cm 이내 → 강한 회전
                if l_min > r_min:               # 왼쪽이 더 넓으면
                    send_command(0.0, W_TURN)   # 왼쪽 회전
                else:
                    send_command(0.0, -W_TURN)  # 오른쪽 회전
                await asyncio.sleep(0.45)

            elif f_min < 700:                   # 40~70cm → 살짝 회전하면서 전진
                if l_min > r_min:
                    send_command(V_SLOW, W_TURN * 0.7)
                else:
                    send_command(V_SLOW, -W_TURN * 0.7)
                await asyncio.sleep(0.25)

            else:                               # 앞이 깨끗 → 직진
                send_command(V_FORWARD, 0.0)

            await asyncio.sleep(0.08)           # 제어 주기

    except asyncio.CancelledError:
        print("🛑 사용자에 의해 중단됨")
    except Exception as e:
        print("❌ 에러 발생:", e)
    finally:
        # 안전 정지
        send_command(0.0, 0.0)
        time.sleep(0.2)
        if ser:
            ser.close()
        await lidar.shutdown()
        print("🏁 주행 종료")

if __name__ == "__main__":
    asyncio.run(main())
