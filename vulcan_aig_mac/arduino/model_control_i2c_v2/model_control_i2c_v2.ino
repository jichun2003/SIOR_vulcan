/*
  실험계획 2단계: 서보 2개(pan D3, tilt D9) + 전동건 DC모터 1개(N채널 MOSFET, D5) 통합 펌웨어.
  [2026-11-26] I2C 수신 버전 v2 (model_control_i2c_v2)

  사격 방식: 전동건(AEG). DC모터가 도는 동안 기어박스가 연사하므로 피더 서보와 플라이휠은 없다.
  모터는 "t<ms>" 명령으로 정해진 시간만 돌고 펌웨어가 스스로 끈다. PC 쪽 프로그램이 멈추거나
  통신이 끊겨도 모터가 계속 돌지 않게 하기 위함 (최대 MAX_FIRE_MS로 제한).

  배선:
    서보(D3) = pan (포탑 베이스 회전)
    서보(D9) = tilt (포신 고저각)
    MOSFET 게이트 = D5 (PWM, Timer0. Servo 라이브러리와 충돌 없음)

    모터 전원(+) ── 모터 ── MOSFET 드레인
    MOSFET 소스 ── GND (모터 전원 GND와 아두이노 GND 공통 연결 필수)
    D5 ── 220Ω ── 게이트,  게이트-소스 사이 10kΩ 풀다운
    플라이백 다이오드: 모터 양단, 캐소드(띠)를 모터 전원(+) 쪽으로
    (전동건 모터는 기동 전류가 수십 A까지 튈 수 있으니 MOSFET/배선/전원 용량 확인)

  AI-G 보드 연결 (I2C, 우노 = 슬레이브 주소 0x08):
    AI-G 40핀 3번 (SDA, /dev/i2c-1) -> 우노 A4
    AI-G 40핀 5번 (SCL, /dev/i2c-1) -> 우노 A5
    AI-G 40핀 9번 (GND)             -> 우노 GND
    보드는 3.3V I2C라서 우노 내부 풀업(5V)은 끈다. 풀업은 보드 쪽 3.3V 풀업을 쓴다.
    보드는 명령 한 줄("s2 90\n" 등)을 I2C 쓰기 한 번(32바이트 이하)으로 보낸다.

  명령 프로토콜 (USB 시리얼 9600 baud와 I2C 공통, 줄바꿈으로 구분):
    s2 45    : pan(D3) 목표각
    s3 120   : tilt(D9) 목표각
    c        : pan/tilt 모두 90도
    t1500    : 1500ms 동안 사격 (모터 ON 후 자동 OFF)
    f        : 수동 테스트용 사격 ON (MAX_FIRE_MS 후 자동 OFF)
    x        : 즉시 사격 정지
    v200     : 모터 PWM 세기(0~255). 모터 정격보다 전원 전압이 높을 때 낮춤
    d        : 서보 힘 풀기 (detach, 서보가 손으로 돌아감). s2/s3/c를 받으면 다시 잡음

  변경 내역 (model_control.ino 대비):
    - D2 SoftwareSerial 수신 삭제 (AI-G 디버그 헤더 TX 고장으로 사용 불가)
    - I2C 슬레이브 수신 추가 (Wire, 주소 0x08). 명령 처리(handleCommand)는 그대로 공유
    - DEBUG_I2C = 1 이면 I2C로 받은 줄을 USB 시리얼로 "[i2c] ..." 형태로 출력
  v2 변경 (2026-11-26):
    - d 명령 추가: 서보 detach (보드 run_i2c.sh가 종료할 때 x, d를 보냄)
    - USB 시리얼 9600 -> 115200 baud: 디버그 출력(30ms마다 각도)이 밀려 루프가 느려지지 않게.
      노트북 모니터는 picocom -b 115200 으로 연다
*/

#include <Servo.h>
#include <Wire.h>

#define DEBUG_I2C 1   // 1: I2C로 받은 명령을 USB로 출력 (확인 끝나면 0)

// ---------- I2C 수신 (AI-G 보드 -> 우노) ----------
const uint8_t I2C_ADDR = 0x08;

// 수신 인터럽트에서 받은 바이트를 담아 두는 링버퍼 (loop에서 꺼내 처리)
volatile uint8_t i2cRx[64];
volatile uint8_t i2cHead = 0, i2cTail = 0;

void onI2CReceive(int n) {
  while (Wire.available()) {
    uint8_t c = Wire.read();
    uint8_t next = (i2cHead + 1) & 63;
    if (next != i2cTail) {   // 버퍼가 가득 차면 버림
      i2cRx[i2cHead] = c;
      i2cHead = next;
    }
  }
}

char i2cLine[32];
int i2cLen = 0;

// ---------- 서보 (인덱스 0 = pan D3 = "s2", 1 = tilt D9 = "s3") ----------
const int N_SERVO = 2;
const int SERVO_PINS[N_SERVO] = {3, 9};
const float SERVO_MIN[N_SERVO] = {10.0, 20.0};
const float SERVO_MAX[N_SERVO] = {170.0, 160.0};
const float SERVO_CENTER[N_SERVO] = {90.0, 90.0};

bool servosAttached = false;

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
void attachServos() {
  if (servosAttached) return;
  for (int i = 0; i < N_SERVO; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write((int)servoCurrent[i]);   // 풀기 전 마지막 위치에서 다시 잡음
  }
  servosAttached = true;
}

void detachServos() {
  for (int i = 0; i < N_SERVO; i++) servos[i].detach();
  servosAttached = false;
}

void setServoTarget(int idx, float angle) {
  attachServos();   // 힘을 푼 상태에서 각도 명령이 오면 다시 잡는다
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
  else if (input == "d") {
    detachServos();
    Serial.println(F("Servos released (detach)"));
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

// I2C 링버퍼에서 바이트를 꺼내 줄 단위로 모아 처리 (블로킹 없음)
void processI2C() {
  while (i2cTail != i2cHead) {
    char ch = (char)i2cRx[i2cTail];
    i2cTail = (i2cTail + 1) & 63;
    if (ch == '\n' || ch == '\r') {
      if (i2cLen > 0) {
        i2cLine[i2cLen] = 0;
        String input = String(i2cLine);
        input.trim();
        i2cLen = 0;
        if (input.length() > 0) {
#if DEBUG_I2C
          Serial.print(F("[i2c] "));
          Serial.println(input);
#endif
          handleCommand(input);
        }
      }
    } else if (i2cLen < (int)sizeof(i2cLine) - 1) {
      i2cLine[i2cLen++] = ch;
    } else {
      i2cLen = 0;  // 너무 긴 줄은 버림
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(MOTOR_PWM, OUTPUT);
  stopFire();

  for (int i = 0; i < N_SERVO; i++) {
    servoCurrent[i] = SERVO_CENTER[i];
    servoTarget[i] = SERVO_CENTER[i];
  }
  attachServos();

  Wire.begin(I2C_ADDR);
  // Wire.begin()이 켜는 내부 풀업(5V)을 끔 -> 보드의 3.3V I2C 핀으로 5V가 들어가지 않게 함
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  Wire.onReceive(onI2CReceive);

  Serial.println(F("=== model_control_i2c_v2: pan(D3)/tilt(D9) + AEG motor(MOSFET D5) ==="));
  Serial.println(F("s2/s3 <deg>, c, d(release), t<ms>(burst), f(manual), x(stop), v<0-255>"));
  Serial.println(F("[link] AI-G cmd input = I2C addr 0x08 (SDA=A4, SCL=A5)"));
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) handleCommand(input);
  }
  processI2C();   // AI-G 보드(I2C)에서 들어온 명령 처리
  updateFire();
  updateServos();
}
