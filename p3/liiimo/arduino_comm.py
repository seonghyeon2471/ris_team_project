"""
Arduino R4 Minima — Serial 통신 모듈
=====================================
송신 프로토콜 (RPM 모드):   "LEFT_RPM,RIGHT_RPM\n"   예) "80,80\n"
송신 프로토콜 (거리 모드):  "0.5\n"
수신 피드백:               "<LENC:1234,RENC:5678>\n"

물리 파라미터:
  PPR_R = 492.5,  PPR_L = 493.0  (엔코더 펄스/회전)
  바퀴 반지름 = 0.034 m
  최대 RPM = 150
"""

import logging
import math
import time
import serial

log = logging.getLogger("arduino")

# ── 차량 물리 파라미터 ────────────────────────────────
WHEEL_RADIUS  = 0.034          # m
WHEEL_CIRC    = 2 * math.pi * WHEEL_RADIUS  # ≈ 0.2136 m/rev
TRACK_WIDTH   = 0.17           # m  좌우 바퀴 간격 — 실측 후 수정!
MAX_RPM       = 150            # Arduino 최대 RPM 입력값

# m/s → RPM 변환:  RPM = (v / WHEEL_CIRC) * 60
def mps_to_rpm(v_ms: float) -> int:
    return int((v_ms / WHEEL_CIRC) * 60.0)

# RPM → m/s 변환
def rpm_to_mps(rpm: int) -> float:
    return (rpm / 60.0) * WHEEL_CIRC


class ArduinoComm:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port      = port
        self.baudrate  = baudrate
        self._ser      = None
        self._last_l   = 9999   # 강제 첫 전송
        self._last_r   = 9999
        self.enc_left  = 0
        self.enc_right = 0

    # ──────────────────────────────────────────────────
    def connect(self):
        try:
            self._ser = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                timeout=0.1,
            )
            time.sleep(2.0)   # R4 Minima 리셋 대기
            # READY 수신 대기
            deadline = time.time() + 5.0
            while time.time() < deadline:
                line = self._ser.readline().decode("ascii", errors="ignore").strip()
                if "<READY>" in line:
                    log.info(f"Arduino 연결 완료: {self.port}")
                    return
            log.warning("Arduino READY 응답 없음 — 계속 진행")
        except serial.SerialException as e:
            log.error(f"Arduino 포트 열기 실패 {self.port}: {e}")
            self._ser = None

    # ──────────────────────────────────────────────────
    def send_rpm(self, rpm_left: int, rpm_right: int):
        """
        좌/우 RPM을 직접 전송.  범위: -MAX_RPM ~ +MAX_RPM
        """
        if self._ser is None or not self._ser.is_open:
            return

        rpm_l = int(max(-MAX_RPM, min(MAX_RPM, rpm_left)))
        rpm_r = int(max(-MAX_RPM, min(MAX_RPM, rpm_right)))

        # 변화 없으면 생략
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
        """
        main.py 인터페이스 호환용.
        speed(m/s) + steer_deg(도) → 좌우 RPM으로 변환 후 전송.

        차동구동 믹싱:
          v_base  = speed
          omega   = v_base * tan(steer_rad) / TRACK_WIDTH
          v_left  = v_base - omega * TRACK_WIDTH / 2
          v_right = v_base + omega * TRACK_WIDTH / 2
        """
        steer_rad = math.radians(steer_deg)
        omega     = speed * math.tan(steer_rad) / max(TRACK_WIDTH, 1e-3)

        v_left  = speed - omega * TRACK_WIDTH / 2.0
        v_right = speed + omega * TRACK_WIDTH / 2.0

        # 최대 속도 정규화
        max_v = max(abs(v_left), abs(v_right), 1e-6)
        if max_v > speed and speed > 0:
            scale   = speed / max_v
            v_left  *= scale
            v_right *= scale

        rpm_l = mps_to_rpm(v_left)
        rpm_r = mps_to_rpm(v_right)

        self.send_rpm(rpm_l, rpm_r)

    # ──────────────────────────────────────────────────
    def send_distance(self, dist_m: float):
        """거리 모드: 직진 거리(m) 전송."""
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
        """
        Arduino 엔코더 피드백 수신.
        수신 형식: <LENC:1234,RENC:5678>
        """
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
        """즉시 정지 — 3회 반복 전송."""
        for _ in range(3):
            self._last_l = 9999  # 강제 재전송
            self.send_rpm(0, 0)
        log.warning("긴급 정지 전송 완료")

    def close(self):
        self.emergency_stop()
        if self._ser and self._ser.is_open:
            self._ser.close()
            log.info("Arduino 포트 닫힘")
