"""
Arduino R4 Minima — Serial 통신 모듈 (개선 버전)
=================================================
- R4 Minima 시리얼 연결 안정성 강화
- dsrdtr=False 적용으로 불필요한 리셋 방지
- READY 메시지 대기 로직 개선
- 연결 후 자동 테스트 명령 전송
"""

import logging
import math
import time
import serial

log = logging.getLogger("arduino")

# ── 차량 물리 파라미터 ────────────────────────────────
WHEEL_RADIUS  = 0.034
WHEEL_CIRC    = 2 * math.pi * WHEEL_RADIUS
TRACK_WIDTH   = 0.18           # 실제 로봇 바퀴 간 거리 실측 필요
MAX_RPM       = 150


def mps_to_rpm(v_ms: float) -> int:
    return int((v_ms / WHEEL_CIRC) * 60.0)


def rpm_to_mps(rpm: int) -> float:
    return (rpm / 60.0) * WHEEL_CIRC


class ArduinoComm:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port      = port
        self.baudrate  = baudrate
        self._ser      = None
        self._last_l   = 9999
        self._last_r   = 9999
        self.enc_left  = 0
        self.enc_right = 0

    # ──────────────────────────────────────────────────
    def connect(self):
        """R4 Minima와의 시리얼 연결 (안정성 강화 버전)"""
        try:
            self._ser = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                dsrdtr=False,      # R4 Minima 리셋 방지
                rtscts=False
            )
            time.sleep(2.5)        # 리셋 + 안정화 대기

            # 버퍼 초기화
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

            # READY 대기 (최대 6초)
            deadline = time.time() + 6.0
            got_ready = False

            while time.time() < deadline:
                if self._ser.in_waiting:
                    line = self._ser.readline().decode("ascii", errors="ignore").strip()
                    if "<READY>" in line:
                        log.info(f"Arduino 연결 완료: {self.port}")
                        got_ready = True
                        break
                time.sleep(0.05)

            if not got_ready:
                log.warning("Arduino READY 응답 없음 — 계속 진행")

            # 연결 확인용 테스트 명령 전송
            self.send_rpm(0, 0)
            time.sleep(0.1)

        except serial.SerialException as e:
            log.error(f"Arduino 포트 열기 실패 {self.port}: {e}")
            self._ser = None
        except Exception as e:
            log.error(f"Arduino connect 예외 발생: {e}")
            self._ser = None

    # ──────────────────────────────────────────────────
    def send_rpm(self, rpm_left: int, rpm_right: int):
        """좌/우 RPM 직접 전송"""
        if self._ser is None or not self._ser.is_open:
            return

        rpm_l = int(max(-MAX_RPM, min(MAX_RPM, rpm_left)))
        rpm_r = int(max(-MAX_RPM, min(MAX_RPM, rpm_right)))

        if abs(rpm_l - self._last_l) < 1 and abs(rpm_r - self._last_r) < 1:
            return

        msg = f"{rpm_l},{rpm_r}\n"
        try:
            self._ser.write(msg.encode("ascii"))
            self._last_l = rpm_l
            self._last_r = rpm_r
            log.debug(f"→ Arduino RPM: L={rpm_l}  R={rpm_r}")
        except serial.SerialException as e:
            log.error(f"Arduino 전송 오류: {e}")

    # ──────────────────────────────────────────────────
    def send_command(self, speed: float, steer_deg: float):
        """main.py 호환용 (speed + steer → 좌우 RPM 변환)"""
        steer_rad = math.radians(steer_deg)
        omega = speed * math.tan(steer_rad) / max(TRACK_WIDTH, 1e-3)

        v_left  = speed - omega * TRACK_WIDTH / 2.0
        v_right = speed + omega * TRACK_WIDTH / 2.0

        # 최대 속도 정규화
        max_v = max(abs(v_left), abs(v_right), 1e-6)
        if max_v > abs(speed) and abs(speed) > 0:
            scale = abs(speed) / max_v
            v_left  *= scale
            v_right *= scale

        rpm_l = mps_to_rpm(v_left)
        rpm_r = mps_to_rpm(v_right)

        self.send_rpm(rpm_l, rpm_r)

    # ──────────────────────────────────────────────────
    def send_distance(self, dist_m: float):
        """거리 모드 명령 전송"""
        if self._ser is None or not self._ser.is_open:
            return
        msg = f"{dist_m:.3f}\n"
        try:
            self._ser.write(msg.encode("ascii"))
            log.debug(f"→ Arduino DIST: {dist_m:.3f} m")
        except serial.SerialException as e:
            log.error(f"Arduino 전송 오류: {e}")

    # ──────────────────────────────────────────────────
    def read_feedback(self) -> dict | None:
        """엔코더 피드백 수신"""
        if self._ser is None or not self._ser.is_open:
            return None
        try:
            while self._ser.in_waiting:
                line = self._ser.readline().decode("ascii", errors="ignore").strip()
                parsed = self._parse(line)
                if parsed and "LENC" in parsed:
                    self.enc_left  = int(parsed["LENC"])
                    self.enc_right = int(parsed["RENC"])
                    return parsed
        except Exception:
            pass
        return None

    def _parse(self, line: str) -> dict | None:
        try:
            line = line.strip("<>")
            return dict(kv.split(":") for kv in line.split(","))
        except Exception:
            return None

    # ──────────────────────────────────────────────────
    def emergency_stop(self):
        """긴급 정지"""
        for _ in range(3):
            self._last_l = 9999
            self.send_rpm(0, 0)
        log.warning("긴급 정지 전송 완료")

    def close(self):
        self.emergency_stop()
        if self._ser and self._ser.is_open:
            self._ser.close()
            log.info("Arduino 포트 닫힘")
