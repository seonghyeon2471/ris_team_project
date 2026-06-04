/*
 * Arduino UNO R4 Minima — Differential Drive Controller
 * ======================================================
 * 실제 핀 배선 기반으로 작성
 *
 * ── 핀 배선 ──────────────────────────────────────────
 *   오른쪽 모터:  PWM=D9,  DIR1=D10, DIR2=D11
 *   왼쪽  모터:  PWM=D6,  DIR1=D7,  DIR2=D8
 *   오른쪽 엔코더: D2 (인터럽트 0)
 *   왼쪽  엔코더: D3 (인터럽트 1)
 *
 * ── Raspberry Pi → Arduino 수신 프로토콜 ──────────────
 *   RPM 모드:      "LEFT_RPM,RIGHT_RPM\n"   예) "80,80\n"
 *   거리 모드:     "0.5\n"                   예) "0.5\n" (단위: m)
 *   긴급 정지:     "0,0\n"
 *
 * ── Arduino → Raspberry Pi 송신 ───────────────────────
 *   "<LENC:1234,RENC:5678>\n"   (100ms 주기)
 *
 * ── 물리 파라미터 ─────────────────────────────────────
 *   PPR_R = 492.5  (오른쪽 엔코더 펄스/회전)
 *   PPR_L = 493.0  (왼쪽  엔코더 펄스/회전)
 *   바퀴 반지름 = 34mm
 *
 * ── 버그 수정 ─────────────────────────────────────────
 *   원본 driveRPM()에서 PWPin_r → PWMPin_r 수정
 */

// =========================
// MOTOR PINS
// =========================
const byte PWMPin_r  = 9;
const byte DirPin1_r = 10;
const byte DirPin2_r = 11;

const byte PWMPin_l  = 6;
const byte DirPin1_l = 7;
const byte DirPin2_l = 8;

// =========================
// ENCODER PINS
// =========================
const byte ENC_R = 2;   // 인터럽트 0
const byte ENC_L = 3;   // 인터럽트 1

volatile long encoder_r = 0;
volatile long encoder_l = 0;

// =========================
// PHYSICAL PARAMETERS
// =========================
const float PPR_R      = 492.5;          // 오른쪽 엔코더 펄스/회전
const float PPR_L      = 493.0;          // 왼쪽  엔코더 펄스/회전
const float WHEEL_R    = 0.034;          // 바퀴 반지름 (m)
const float WHEEL_CIRC = 2.0 * PI * WHEEL_R;  // 바퀴 둘레 (m) ≈ 0.2136m

// =========================
// CONTROL PARAMETERS
// =========================
const int   MAX_RPM      = 150;    // driveRPM() 입력 최대값
const int   PWM_MAX      = 255;    // PWM 상한 (필요시 200으로 낮춤)
const int   PWM_MIN_MOVE = 50;     // 정지 판정 하한
const float STRAIGHT_KP  = 200.0; // 직진 보정 gain (거리 모드)

// =========================
// TIMEOUT
// =========================
const unsigned long TIMEOUT_MS = 500;  // 통신 끊기면 자동 정지

// =========================
// MODE
// =========================
enum Mode { IDLE, RPM_MODE, DISTANCE_MODE };
Mode mode = IDLE;

// =========================
// DISTANCE CONTROL STATE
// =========================
float target_distance = 0.0;
bool  moving          = false;

// =========================
// RPM CONTROL STATE
// =========================
int cmd_rpm_l = 0;
int cmd_rpm_r = 0;

unsigned long lastCmdMs = 0;

// =========================
// ENCODER ISR
// =========================
void encoderISR_r() { encoder_r++; }
void encoderISR_l() { encoder_l++; }

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  pinMode(PWMPin_r,  OUTPUT);
  pinMode(DirPin1_r, OUTPUT);
  pinMode(DirPin2_r, OUTPUT);

  pinMode(PWMPin_l,  OUTPUT);
  pinMode(DirPin1_l, OUTPUT);
  pinMode(DirPin2_l, OUTPUT);

  pinMode(ENC_R, INPUT_PULLUP);
  pinMode(ENC_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_R), encoderISR_r, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_L), encoderISR_l, RISING);

  stopMotors();
  Serial.println("<READY>");
}

// =========================
// MAIN LOOP
// =========================
void loop() {

  // ── 시리얼 수신 ──────────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) goto feedback;

    // RPM 모드: "LEFT,RIGHT"  예) "80,80"  또는  "0,0"
    if (cmd.indexOf(',') >= 0) {
      int idx       = cmd.indexOf(',');
      cmd_rpm_l     = constrain(cmd.substring(0, idx).toInt(),     -MAX_RPM, MAX_RPM);
      cmd_rpm_r     = constrain(cmd.substring(idx + 1).toInt(),    -MAX_RPM, MAX_RPM);
      mode          = RPM_MODE;
      lastCmdMs     = millis();
      driveRPM(cmd_rpm_l, cmd_rpm_r);
    }
    // 거리 모드: "0.5"
    else {
      float dist = cmd.toFloat();
      if (dist > 0.0) {
        target_distance = dist;
        encoder_r = 0;
        encoder_l = 0;
        moving    = true;
        mode      = DISTANCE_MODE;
        lastCmdMs = millis();
      }
    }
  }

  // ── 타임아웃 안전 정지 ───────────────────────────
  if (millis() - lastCmdMs > TIMEOUT_MS) {
    if (mode == RPM_MODE) {
      stopMotors();
      mode = IDLE;
    }
  }

  // ── 거리 제어 루프 ───────────────────────────────
  if (mode == DISTANCE_MODE && moving) {
    distanceControl();
  }

  // ── 엔코더 피드백 송신 (100ms) ───────────────────
  feedback:
  static unsigned long lastFbMs = 0;
  if (millis() - lastFbMs >= 100) {
    Serial.print("<LENC:");
    Serial.print(encoder_l);
    Serial.print(",RENC:");
    Serial.print(encoder_r);
    Serial.println(">");
    lastFbMs = millis();
  }
}

// =========================
// RPM CONTROL
// RPM 입력 범위: -MAX_RPM ~ +MAX_RPM
// 음수 = 후진
// =========================
void driveRPM(int left_rpm, int right_rpm) {
  int pwm_l = map(abs(left_rpm),  0, MAX_RPM, 0, PWM_MAX);
  int pwm_r = map(abs(right_rpm), 0, MAX_RPM, 0, PWM_MAX);

  pwm_l = constrain(pwm_l, 0, PWM_MAX);
  pwm_r = constrain(pwm_r, 0, PWM_MAX);

  setMotor(PWMPin_l, DirPin1_l, DirPin2_l, pwm_l, left_rpm  >= 0);
  setMotor(PWMPin_r, DirPin1_r, DirPin2_r, pwm_r, right_rpm >= 0);
}

// =========================
// DISTANCE CONTROL
// 엔코더 기반 직진 + 보정
// =========================
void distanceControl() {
  float dist_r = (encoder_r / PPR_R) * WHEEL_CIRC;
  float dist_l = (encoder_l / PPR_L) * WHEEL_CIRC;
  float current = (dist_r + dist_l) / 2.0;
  float error   = target_distance - current;

  if (error <= 0.002) {   // 2mm 이내면 도착
    stopMotors();
    moving = false;
    mode   = IDLE;
    Serial.println("<DONE>");
    return;
  }

  // 속도 비례 감속 (원본 로직 유지)
  int pwm = 120 + (int)(135.0 * (error / target_distance));

  // 좌우 거리 차이 보정
  float diff       = dist_r - dist_l;
  int   correction = (int)(diff * STRAIGHT_KP);

  int pwm_r = constrain(pwm - correction, PWM_MIN_MOVE, PWM_MAX);
  int pwm_l = constrain(pwm + correction, PWM_MIN_MOVE, PWM_MAX);

  setMotor(PWMPin_r, DirPin1_r, DirPin2_r, pwm_r, true);
  setMotor(PWMPin_l, DirPin1_l, DirPin2_l, pwm_l, true);
}

// =========================
// MOTOR DRIVER
// forward=true → DIR1=HIGH, DIR2=LOW
// forward=false → DIR1=LOW,  DIR2=HIGH
// =========================
void setMotor(byte pwmPin, byte dir1, byte dir2, int pwm, bool forward) {
  digitalWrite(dir1, forward ? HIGH : LOW);
  digitalWrite(dir2, forward ? LOW  : HIGH);
  analogWrite(pwmPin, pwm);
}

// =========================
void stopMotors() {
  analogWrite(PWMPin_r, 0);
  analogWrite(PWMPin_l, 0);
  // 브레이크: 둘 다 LOW
  digitalWrite(DirPin1_r, LOW); digitalWrite(DirPin2_r, LOW);
  digitalWrite(DirPin1_l, LOW); digitalWrite(DirPin2_l, LOW);
}
