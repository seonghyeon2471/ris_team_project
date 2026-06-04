// =============================================
// 엔코더 거리 측정 테스트 코드
// =============================================
// 사용법:
// 1. 로봇 바퀴를 손으로 돌려보면서 거리 확인
// 2. 시리얼 모니터에서 'r' 입력 → 거리 리셋
// 3. 'f' 입력 → 앞으로 3초간 천천히 이동 후 결과 출력
// =============================================

const byte PWMR  = 9;
const byte DIRR1 = 10;
const byte DIRR2 = 11;

const byte PWML  = 6;
const byte DIRL1 = 7;
const byte DIRL2 = 8;

const byte ENC_R = 2;
const byte ENC_L = 3;

volatile long encoder_r = 0;
volatile long encoder_l = 0;

float pulses_per_rev_r = 492.5;
float pulses_per_rev_l = 493;
float wheel_R = 0.034;        // 바퀴 반지름 (m)

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PWMR, OUTPUT); pinMode(DIRR1, OUTPUT); pinMode(DIRR2, OUTPUT);
  pinMode(PWML, OUTPUT); pinMode(DIRL1, OUTPUT); pinMode(DIRL2, OUTPUT);

  pinMode(ENC_R, INPUT_PULLUP);
  pinMode(ENC_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encoderISR_r, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_L), encoderISR_l, RISING);

  stopMotors();
  Serial.println("=== 엔코더 테스트 코드 시작 ===");
  Serial.println("명령어: r = 리셋, f = 앞으로 3초 주행 테스트");
  printStatus();
}

void encoderISR_r() { encoder_r++; }
void encoderISR_l() { encoder_l++; }

void stopMotors() {
  analogWrite(PWMR, 0); digitalWrite(DIRR1, LOW); digitalWrite(DIRR2, LOW);
  analogWrite(PWML, 0); digitalWrite(DIRL1, LOW); digitalWrite(DIRL2, LOW);
}

void driveForward(int pwm) {
  // 오른쪽 모터 전진
  analogWrite(PWMR, pwm);
  digitalWrite(DIRR1, LOW);
  digitalWrite(DIRR2, HIGH);

  // 왼쪽 모터 전진
  analogWrite(PWML, pwm);
  digitalWrite(DIRL1, LOW);
  digitalWrite(DIRL2, HIGH);
}

float getTraveledDistanceCm() {
  noInterrupts();
  long er = encoder_r;
  long el = encoder_l;
  interrupts();

  float dist_r = (er / pulses_per_rev_r) * 2.0 * PI * wheel_R * 100.0;
  float dist_l = (el / pulses_per_rev_l) * 2.0 * PI * wheel_R * 100.0;

  return (dist_r + dist_l) / 2.0;
}

void resetEncoders() {
  noInterrupts();
  encoder_r = 0;
  encoder_l = 0;
  interrupts();
  Serial.println(">>> 엔코더 리셋 완료 <<<");
}

void printStatus() {
  float dist = getTraveledDistanceCm();
  Serial.print("Dist: ");
  Serial.print(dist, 2);
  Serial.print(" cm  |  R: ");
  Serial.print(encoder_r);
  Serial.print("  L: ");
  Serial.println(encoder_l);
}

void loop() {
  // 시리얼 명령 처리
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'r' || cmd == 'R') {
      resetEncoders();
    }
    else if (cmd == 'f' || cmd == 'F') {
      Serial.println(">>> 앞으로 3초 주행 테스트 시작 <<<");
      resetEncoders();
      driveForward(80);           // 낮은 속도로 전진 (80/255)
      delay(3000);
      stopMotors();

      float dist = getTraveledDistanceCm();
      Serial.print(">>> 주행 완료! 측정 거리: ");
      Serial.print(dist, 2);
      Serial.println(" cm");
      Serial.println("실제 자로 재서 비교해보세요!");
    }
  }

  // 200ms마다 현재 거리 출력
  if (millis() - lastPrint > 200) {
    printStatus();
    lastPrint = millis();
  }
}
