"""
Arduino R4 Minima — Serial 통신 모듈 (차동구동 버전)
=====================================================
송신: <SPEED:±0.00,STEER:±00.0>\n
수신: <LENC:1234,RENC:1234>\n
"""

import logging
import time
import serial

log = logging.getLogger("arduino")


class ArduinoComm:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port      = port
        self.baudrate  = baudrate
        self._ser      = None
        self._last_speed = 999.0   # 강제 첫 전송을 위해 큰 값으로 초기화
        self._last_steer = 999.0
        self.enc_left  = 0
        self.enc_right = 0

    def connect(self):
        try:
            self._ser = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                timeout=0.1,
            )
            time.sleep(2.0)   # R4 Minima: 시리얼 열릴 때 리셋됨
            # READY 메시지 수신 대기
            deadline = time.time() + 5.0
            while time.time() < deadline:
                line = self._ser.readline().decode("ascii", errors="ignore").strip()
                if "<READY>" in line:
                    log.info(f"Arduino 연결 완료: {self.port} @ {self.baudrate}")
                    return
            log.warning("Arduino READY 응답 없음 — 계속 진행")
        except serial.SerialException as e:
            log.error(f"Arduino 포트 열기 실패 {self.port}: {e}")
            self._ser = None

    # ──────────────────────────────────────────────────
    def send_command(self, speed: float, steer_deg: float):
        """
        speed    : m/s  (양수=전진)
        steer_deg: 도   (양수=좌회전)
        """
        if self._ser is None or not self._ser.is_open:
            return

        # 변화량이 임계치 이하면 전송 생략 (통신 부하 감소)
        if (abs(speed - self._last_speed) < 0.01 and
                abs(steer_deg - self._last_steer) < 0.5):
            return

        msg = f"<SPEED:{speed:+.2f},STEER:{steer_deg:+05.1f}>\n"
        try:
            self._ser.write(msg.encode("ascii"))
            self._last_speed = speed
            self._last_steer = steer_deg
            log.debug(f"→ Arduino: {msg.strip()}")
        except serial.SerialException as e:
            log.error(f"Arduino 전송 오류: {e}")

    # ──────────────────────────────────────────────────
    def read_feedback(self) -> dict | None:
        """
        Arduino로부터 엔코더 피드백 수신.
        수신 형식: <LENC:1234,RENC:5678>
        """
        if self._ser is None or not self._ser.is_open:
            return None
        try:
            while self._ser.in_waiting:
                line = self._ser.readline().decode("ascii", errors="ignore").strip()
                parsed = self._parse(line)
                if parsed:
                    self.enc_left  = int(parsed.get("LENC", self.enc_left))
                    self.enc_right = int(parsed.get("RENC", self.enc_right))
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
        """즉시 정지 (3회 전송으로 신뢰성 확보)"""
        for _ in range(3):
            self._last_speed = 999.0   # 강제 재전송
            self.send_command(0.0, 0.0)
        log.warning("긴급 정지 전송")

    def close(self):
        self.emergency_stop()
        if self._ser and self._ser.is_open:
            self._ser.close()
            log.info("Arduino 포트 닫힘")
