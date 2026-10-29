/*
  실험계획 2단계: 서보 3개(D2,D3,D9) + DC모터 2개(L298N, 플라이휠) 통합 펌웨어.

  IMPLEMENT/ 구조에서의 역할(model_control.ino): 시리얼 명령 파싱, 서보 목표각·
  한계 관리, 슬루레이트 제한(15ms당 1도), 플라이휠(DC모터) 속도 램프, 응답 출력.
  기존 firmware/turret_control/turret_control.ino와 프로토콜·동작이 동일하다.

  실제 배선(2026-09-23 현장 확인, 본체 기준 물리적 위치):
    서보1(D2) = 공을 밀어주는 부분 (피더/푸셔, 발사 매커니즘 - 조준 축 아님)
    서보2(D3) = 본체 맨 아래 (포탑 베이스 회전 = pan)
    서보3(D9) = 총구 근처 우측 (포신 고저각 = tilt)
    DC모터 2개(L298N: ENA=5/IN1=7/IN2=8, ENB=6/IN3=11/IN4=12) = 플라이휠 발사 모터.

  시리얼 프로토콜 (9600 baud, s번호는 기능이 아니라 핀 순서(D2,D3,D9) 고정):
    s1 90   : 서보1(D2, 피더)을 90도로 목표 설정
    s2 45   : 서보2(D3, pan)을 45도로 목표 설정
    s3 120  : 서보3(D9, tilt)을 120도로 목표 설정
    c       : 서보 3개 모두 90도(중앙)
    f/b/l/r/x : DC모터(플라이휠) 전진/후진/좌회전/우회전/정지
    v150      : DC모터(플라이휠) 목표 속도(0~255)

  IMPLEMENT/main.py는 매 프레임 "s2 <pan>\n", "s3 <tilt>\n" 두 줄을 보내고,
  IMPLEMENT/serial_link.py의 send_fire()는 사격 시 "v<speed>", "f", "s1 <push>",
  "s1 <rest>", "x"를 순서대로 보내 플라이휠 스핀업 + 피더 푸시를 수행한다.

  서보는 write()로 즉시 이동하지 않고 목표각만 저장해두었다가 매 틱(TICK_MS)마다
  STEP_DEG_PER_TICK만큼만 움직이는 슬루레이트 제한으로 진동/충격을 억제한다.
*/

#include <Servo.h>

// ---------- 서보 (인덱스 0=D2 피더, 1=D3 pan, 2=D9 tilt) ----------
const int SERVO_PINS[3] = {2, 3, 9};
const float SERVO_MIN[3] = {0.0, 10.0, 20.0};    // 피더는 기구 확정 전까지 넓게 열어둠
const float SERVO_MAX[3] = {180.0, 170.0, 160.0};
const float SERVO_CENTER[3] = {90.0, 90.0, 90.0};

Servo servos[3];
float servoCurrent[3];
float servoTarget[3];

const float STEP_DEG_PER_TICK = 1.0;   // 한 틱당 최대 이동각 (슬루레이트 제한으로 진동 억제)
const unsigned long TICK_MS = 15;
unsigned long lastTickMs = 0;

// ---------- DC모터 (L298N, 플라이휠) ----------
const int ENA = 5;   // 모터 A 속도 (PWM)
const int IN1 = 7;
const int IN2 = 8;
const int ENB = 6;   // 모터 B 속도 (PWM)
const int IN3 = 11;
const int IN4 = 12;

const int MAX_SPEED  = 255;  // 전원 전압이 높으면 낮추세요 (예: 14V 전원이면 120)
const int RAMP_DELAY = 12;   // 속도 1 올릴 때마다 간격(ms) -> 0->255 약 3초

int targetSpeed = 200;
int curSpeed = 0;
int dirA = 0, dirB = 0;      // 1 정방향, -1 역방향, 0 정지
unsigned long lastRamp = 0;

void setDir(int in1, int in2, int dir) {
  digitalWrite(in1, dir == 1 ? HIGH : LOW);
  digitalWrite(in2, dir == -1 ? HIGH : LOW);
}

void applySpeed() {
  analogWrite(ENA, dirA == 0 ? 0 : curSpeed);
  analogWrite(ENB, dirB == 0 ? 0 : curSpeed);
}

void setMotion(int a, int b) {
  curSpeed = 0;
  applySpeed();
  dirA = a;
  dirB = b;
  setDir(IN1, IN2, dirA);
  setDir(IN3, IN4, dirB);
}

void stopMotors() {
  curSpeed = 0;
  dirA = dirB = 0;
  applySpeed();
  setDir(IN1, IN2, 0);
  setDir(IN3, IN4, 0);
}

void updateRamp() {
  if (dirA == 0 && dirB == 0) return;
  if (curSpeed > targetSpeed) {
    curSpeed = targetSpeed;
    applySpeed();
  } else if (curSpeed < targetSpeed && millis() - lastRamp >= RAMP_DELAY) {
    lastRamp = millis();
    curSpeed++;
    applySpeed();
  }
}

// ---------- 서보 슬루레이트 제어 ----------
void setServoTarget(int idx, float angle) {
  angle = constrain(angle, SERVO_MIN[idx], SERVO_MAX[idx]);
  servoTarget[idx] = angle;
}

void updateServos() {
  unsigned long now = millis();
  if (now - lastTickMs < TICK_MS) return;
  lastTickMs = now;

  for (int i = 0; i < 3; i++) {
    float diff = servoTarget[i] - servoCurrent[i];
    if (diff > STEP_DEG_PER_TICK) diff = STEP_DEG_PER_TICK;
    if (diff < -STEP_DEG_PER_TICK) diff = -STEP_DEG_PER_TICK;
    servoCurrent[i] += diff;
    servos[i].write((int)servoCurrent[i]);
  }
}

// ---------- 명령 처리 ----------
void handleCommand(String input) {
  if      (input == "f") { setMotion(1, 1);   Serial.println(F("Forward")); }
  else if (input == "b") { setMotion(-1, -1); Serial.println(F("Backward")); }
  else if (input == "l") { setMotion(-1, 1);  Serial.println(F("Turn left")); }
  else if (input == "r") { setMotion(1, -1);  Serial.println(F("Turn right")); }
  else if (input == "x") { stopMotors();      Serial.println(F("Stop")); }

  else if (input.startsWith("v")) {
    String num = input.substring(1);
    num.trim();
    int v = num.toInt();
    if (num.length() > 0 && isDigit(num[0]) && v >= 0 && v <= MAX_SPEED) {
      targetSpeed = v;
      Serial.print(F("DC target speed: "));
      Serial.println(targetSpeed);
    } else {
      Serial.println(F("Speed range error (ex: v150)"));
    }
  }

  else if (input.startsWith("s") && input.length() >= 2) {
    int idx = input.charAt(1) - '1';
    String num = input.substring(2);
    num.trim();
    int angle = num.toInt();
    if (idx < 0 || idx > 2) {
      Serial.println(F("Servo number is 1~3 (ex: s1 90)"));
    } else if (num.length() == 0 || !isDigit(num[0]) || angle < 0 || angle > 180) {
      Serial.println(F("Angle is 0~180 (ex: s1 90)"));
    } else {
      setServoTarget(idx, (float)angle);
      Serial.print(F("Servo"));
      Serial.print(idx + 1);
      Serial.print(F(" (D"));
      Serial.print(SERVO_PINS[idx]);
      Serial.print(F(") -> "));
      Serial.println(angle);
    }
  }

  else if (input == "c") {
    for (int i = 0; i < 3; i++) setServoTarget(i, SERVO_CENTER[i]);
    Serial.println(F("All servos -> center"));
  }

  else {
    Serial.print(F("Unknown command: "));
    Serial.println(input);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);   // 줄바꿈이 없어도 50ms 후 입력 처리 (Tinkercad 대응)

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  for (int i = 0; i < 3; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servoCurrent[i] = SERVO_CENTER[i];
    servoTarget[i] = SERVO_CENTER[i];
    servos[i].write((int)servoCurrent[i]);
  }

  Serial.println(F("=== model_control: Servo(feeder/pan/tilt) + Flywheel ==="));
  Serial.println(F("Flywheel: f/b/l/r/x, v150(speed)"));
  Serial.println(F("Servo: s1 90(feeder,D2), s2 45(pan,D3), s3 120(tilt,D9), c(center)"));
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) handleCommand(input);
  }
  updateRamp();
  updateServos();
}
