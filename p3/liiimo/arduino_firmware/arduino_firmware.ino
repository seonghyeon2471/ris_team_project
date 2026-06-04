/*
 * Arduino UNO R4 Minima — Differential Drive Controller
 * ======================================================
 * 차동구동 방식: 좌/우 L298N 2개로 방향 전환
 *
 * Serial 수신 (from Raspberry Pi 4B):
 *   <SPEED:±0.00,STEER:±00.0>\n
 *   SPEED: m/s (양수=전진, 음수=후진)
 *   STEER: 도  (양수=좌회전, 음수=우회전)
 *
 * Serial 송신 (to Raspberry Pi):
 *   <LENC:1234,RENC:1234>\n
 *
 * 배선:
 * ┌─────────────────────────────────────────────────┐
 * │  왼쪽 L298N                                      │
 * │    IN1 → D2   IN2 → D3   ENA → D5(PWM)          │
 * │  오른쪽 L298N                                    │
 * │    IN3 → D4   IN4 → D7   ENB → D6(PWM)          │
 * │  왼쪽 엔코더 A → D8  (인터럽트 폴링)            │
 * │  오른쪽 엔코더 A → D9  (인터럽트 폴링)          │
 * └─────────────────────────────────────────────────┘
 *
 * ※ 레벨 컨버터(LV=3.3V, HV=5V) 통해 Pi GPIO 연결
 * ※ L298N VCC(12V) → 배터리 12V 직결
 * ※ L298N 5V핀 → Arduino 5V 공급 가능 (점퍼 확인)
 */

// ── 핀 정의 ──────────────────────────────────────────
// 왼쪽 모터 (L298N #1)
const int L_IN1 = 2;
const int L_IN2 = 3;
const int L_ENA = 5;   // PWM

// 오른쪽 모터 (L298N #2)
const int R_IN3 = 4;
const int R_IN4 = 7;
const int R_ENB = 6;   // PWM

// 엔코더 (없으면 무시)
const int ENC_L = 8;
const int ENC_R = 9;

// ── 파라미터 ──────────────────────────────────────────
const float MAX_SPEED_MS  = 1.0;    // m/s 풀스케일
const int   PWM_MAX       = 200;    // 하드웨어 보호용 상한
const float MAX_STEER_DEG = 30.0;   // 최대 조향각
// 차동 조향 믹싱 계수: 속도 차이 = steer_ratio * steer_deg
const float STEER_RATIO   = 0.03;   // 실측 후 조정

const unsigned long TIMEOUT_MS = 500;  // 통신 끊기면 정지

// ── 상태 변수 ─────────────────────────────────────────
float cmdSpeed = 0.0;
float cmdSteer = 0.0;
unsigned long lastCmdMs = 0;

volatile long encL = 0;
volatile long encR = 0;

// ── 인터럽트 핸들러 ───────────────────────────────────
void isrEncL() { encL++; }
void isrEncR() { encR++; }

// ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_ENA, OUTPUT);
  pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT); pinMode(R_ENB, OUTPUT);

  // 엔코더 핀 (없어도 동작함)
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isrEncR, RISING);

  stopMotors();
  Serial.println("<READY>");
}

void loop() {
  // ── 시리얼 수신 ────────────────────────────────────
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.startsWith("<") && msg.endsWith(">")) {
      parseCommand(msg);
      lastCmdMs = millis();
    }
  }

  // ── 타임아웃 안전 정지 ─────────────────────────────
  if (millis() - lastCmdMs > TIMEOUT_MS) {
    cmdSpeed = 0.0;
    cmdSteer = 0.0;
  }

  // ── 차동 구동 믹싱 적용 ────────────────────────────
  applyDifferentialDrive(cmdSpeed, cmdSteer);

  // ── 피드백 전송 (100ms 주기) ───────────────────────
  static unsigned long lastFbMs = 0;
  if (millis() - lastFbMs >= 100) {
    Serial.print("<LENC:");
    Serial.print(encL);
    Serial.print(",RENC:");
    Serial.print(encR);
    Serial.println(">");
    lastFbMs = millis();
  }
}

// ─────────────────────────────────────────────────────
// 커맨드 파싱: <SPEED:±0.00,STEER:±00.0>
// ─────────────────────────────────────────────────────
void parseCommand(String msg) {
  msg = msg.substring(1, msg.length() - 1);  // < > 제거

  int commaIdx = msg.indexOf(',');
  if (commaIdx < 0) return;

  String speedPart = msg.substring(0, commaIdx);
  String steerPart = msg.substring(commaIdx + 1);

  int si = speedPart.indexOf(':');
  int ti = steerPart.indexOf(':');
  if (si < 0 || ti < 0) return;

  cmdSpeed = constrain(speedPart.substring(si + 1).toFloat(),
                       -MAX_SPEED_MS, MAX_SPEED_MS);
  cmdSteer = constrain(steerPart.substring(ti + 1).toFloat(),
                       -MAX_STEER_DEG, MAX_STEER_DEG);
}

// ─────────────────────────────────────────────────────
// 차동 구동 믹싱
//   steer > 0 → 좌회전: 오른쪽 빠르게, 왼쪽 느리게
//   steer < 0 → 우회전: 왼쪽 빠르게, 오른쪽 느리게
// ─────────────────────────────────────────────────────
void applyDifferentialDrive(float speed, float steerDeg) {
  // 정규화 (−1.0 ~ +1.0)
  float v = speed / MAX_SPEED_MS;
  float s = steerDeg / MAX_STEER_DEG;  // −1=우 ~ +1=좌

  // 좌우 속도 계산
  float vL = v - s * abs(v);   // 좌회전이면 왼쪽 감속
  float vR = v + s * abs(v);   // 좌회전이면 오른쪽 가속

  // 클램프 (−1 ~ +1)
  vL = constrain(vL, -1.0, 1.0);
  vR = constrain(vR, -1.0, 1.0);

  // PWM 변환 및 출력
  driveMotorL(vL);
  driveMotorR(vR);
}

// ─────────────────────────────────────────────────────
void driveMotorL(float v) {
  int pwm = (int)(abs(v) * PWM_MAX);
  pwm = constrain(pwm, 0, PWM_MAX);

  if (v > 0.02) {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  } else if (v < -0.02) {
    digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH);
  } else {
    digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, LOW);
    pwm = 0;
  }
  analogWrite(L_ENA, pwm);
}

void driveMotorR(float v) {
  int pwm = (int)(abs(v) * PWM_MAX);
  pwm = constrain(pwm, 0, PWM_MAX);

  if (v > 0.02) {
    digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, LOW);
  } else if (v < -0.02) {
    digitalWrite(R_IN3, LOW);  digitalWrite(R_IN4, HIGH);
  } else {
    digitalWrite(R_IN3, LOW);  digitalWrite(R_IN4, LOW);
    pwm = 0;
  }
  analogWrite(R_ENB, pwm);
}

void stopMotors() {
  driveMotorL(0); driveMotorR(0);
}
