/*
  실험계획 2단계: 서보 2개(pan D3, tilt D9) + 전동건 DC모터 1개(N채널 MOSFET, D5) 통합 펌웨어.

  사격 방식: 전동건(AEG). DC모터가 도는 동안 기어박스가 연사하므로 피더 서보와 플라이휠은 없다.
  모터는 "t<ms>" 명령으로 정해진 시간만 돌고 펌웨어가 스스로 끈다. PC 쪽 프로그램이 멈추거나
  시리얼이 끊겨도 모터가 계속 돌지 않게 하기 위함 (최대 MAX_FIRE_MS로 제한).

  배선:
    서보(D3) = pan (포탑 베이스 회전)
    서보(D9) = tilt (포신 고저각)
    MOSFET 게이트 = D5 (PWM, Timer0. Servo 라이브러리와 충돌 없음)

    모터 전원(+) ── 모터 ── MOSFET 드레인
    MOSFET 소스 ── GND (모터 전원 GND와 아두이노 GND 공통 연결 필수)
    D5 ── 220Ω ── 게이트,  게이트-소스 사이 10kΩ 풀다운
    플라이백 다이오드: 모터 양단, 캐소드(띠)를 모터 전원(+) 쪽으로
    (전동건 모터는 기동 전류가 수십 A까지 튈 수 있으니 MOSFET/배선/전원 용량 확인)

  시리얼 프로토콜 (9600 baud, s번호는 기존과 호환되게 s2/s3 유지):
    s2 45    : pan(D3) 목표각
    s3 120   : tilt(D9) 목표각
    c        : pan/tilt 모두 90도
    t1500    : 1500ms 동안 사격 (모터 ON 후 자동 OFF)
    f        : 수동 테스트용 사격 ON (MAX_FIRE_MS 후 자동 OFF)
    x        : 즉시 사격 정지
    v200     : 모터 PWM 세기(0~255). 모터 정격보다 전원 전압이 높을 때 낮춤
*/

#include <Servo.h>

// ---------- 서보 (인덱스 0 = pan D3 = "s2", 1 = tilt D9 = "s3") ----------
const int N_SERVO = 2;
const int SERVO_PINS[N_SERVO] = {3, 9};
const float SERVO_MIN[N_SERVO] = {10.0, 20.0};
const float SERVO_MAX[N_SERVO] = {170.0, 160.0};
const float SERVO_CENTER[N_SERVO] = {90.0, 90.0};

Servo servos[N_SERVO];
float servoCurrent[N_SERVO];
float servoTarget[N_SERVO];

const float STEP_DEG_PER_TICK = 1.0;   // 슬루레이트 제한 (15ms당 1도)
const unsigned long TICK_MS = 15;
unsigned long lastTickMs = 0;

// ---------- 전동건 DC모터 (MOSFET) ----------
const int MOTOR_PWM = 5;
const unsigned long MAX_FIRE_MS = 5000;   // 한 번에 최대 사격 시간 (안전장치)

int motorSpeed = 255;          // 전동건은 기본 최대 출력
bool firing = false;
unsigned long fireStartMs = 0;
unsigned long fireDurationMs = 0;

void startFire(unsigned long ms) {
  if (ms > MAX_FIRE_MS) ms = MAX_FIRE_MS;
  fireStartMs = millis();
  fireDurationMs = ms;
  firing = true;
  analogWrite(MOTOR_PWM, motorSpeed);
}

void stopFire() {
  firing = false;
  analogWrite(MOTOR_PWM, 0);
}

void updateFire() {
  if (firing && millis() - fireStartMs >= fireDurationMs) {
    stopFire();
    Serial.println(F("Fire done"));
  }
}

// ---------- 서보 슬루레이트 제어 ----------
void setServoTarget(int idx, float angle) {
  servoTarget[idx] = constrain(angle, SERVO_MIN[idx], SERVO_MAX[idx]);
}

void updateServos() {
  unsigned long now = millis();
  if (now - lastTickMs < TICK_MS) return;
  lastTickMs = now;

  for (int i = 0; i < N_SERVO; i++) {
    float diff = servoTarget[i] - servoCurrent[i];
    if (diff > STEP_DEG_PER_TICK) diff = STEP_DEG_PER_TICK;
    if (diff < -STEP_DEG_PER_TICK) diff = -STEP_DEG_PER_TICK;
    servoCurrent[i] += diff;
    servos[i].write((int)servoCurrent[i]);
  }
}

// 숫자 부분 파싱: 비어 있거나 숫자로 시작하지 않으면 -1
long parseNumber(String s) {
  s.trim();
  if (s.length() == 0 || !isDigit(s[0])) return -1;
  return s.toInt();
}

// ---------- 명령 처리 ----------
void handleCommand(String input) {
  if (input == "x") {
    stopFire();
    Serial.println(F("Fire stop"));
  }
  else if (input == "f") {
    startFire(MAX_FIRE_MS);
    Serial.println(F("Fire on (manual, auto stop)"));
  }
  else if (input.startsWith("t")) {
    long ms = parseNumber(input.substring(1));
    if (ms <= 0) {
      Serial.println(F("Fire time error (ex: t1500)"));
    } else {
      startFire((unsigned long)ms);
      Serial.print(F("Fire for ms: "));
      Serial.println(fireDurationMs);
    }
  }
  else if (input.startsWith("v")) {
    long v = parseNumber(input.substring(1));
    if (v < 0 || v > 255) {
      Serial.println(F("Speed range error (ex: v200)"));
    } else {
      motorSpeed = (int)v;
      if (firing) analogWrite(MOTOR_PWM, motorSpeed);
      Serial.print(F("Motor speed: "));
      Serial.println(motorSpeed);
    }
  }
  else if (input.startsWith("s") && input.length() >= 2) {
    char n = input.charAt(1);
    long angle = parseNumber(input.substring(2));
    int idx = (n == '2') ? 0 : (n == '3') ? 1 : -1;
    if (n == '1') {
      Serial.println(F("s1 (feeder) removed: AEG mode"));
    } else if (idx < 0) {
      Serial.println(F("Servo is s2(pan) or s3(tilt)"));
    } else if (angle < 0 || angle > 180) {
      Serial.println(F("Angle is 0~180 (ex: s2 90)"));
    } else {
      setServoTarget(idx, (float)angle);
    }
  }
  else if (input == "c") {
    for (int i = 0; i < N_SERVO; i++) setServoTarget(i, SERVO_CENTER[i]);
    Serial.println(F("Pan/tilt -> center"));
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(input);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  pinMode(MOTOR_PWM, OUTPUT);
  stopFire();

  for (int i = 0; i < N_SERVO; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servoCurrent[i] = SERVO_CENTER[i];
    servoTarget[i] = SERVO_CENTER[i];
    servos[i].write((int)servoCurrent[i]);
  }

  Serial.println(F("=== model_control: pan(D3)/tilt(D9) + AEG motor(MOSFET D5) ==="));
  Serial.println(F("s2/s3 <deg>, c, t<ms>(burst), f(manual), x(stop), v<0-255>"));
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) handleCommand(input);
  }
  updateFire();
  updateServos();
}
