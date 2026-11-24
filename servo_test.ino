/*
  servo_test.ino — 서보 2개 중 고장난 쪽 찾기용 테스트 스케치
  (model_control.ino와 같은 배선: pan = D3, tilt = D9, 전동건 MOSFET = D5)

  원칙: 한 번에 서보 하나만 attach해서 움직인다. 나머지는 detach(신호 없음) 상태.
       전동건 MOSFET(D5)는 테스트 내내 LOW로 고정해서 절대 발사되지 않게 한다.

  시리얼 모니터: 9600 baud, 줄 끝 "새 줄(LF)"

  명령:
    1        : pan(D3)만 활성화 (tilt는 detach)
    2        : tilt(D9)만 활성화 (pan은 detach)
    0        : 둘 다 detach (토크 해제, 손으로 돌려서 기어 걸림 확인용)
    a<deg>   : 활성 서보를 해당 각도로 (예: a90, a45)
    w        : 스윕 테스트 (최소 -> 최대 -> 최소, 천천히)
    k        : 스텝 테스트 (90 -> 60 -> 120 -> 90, 각 1초 유지)
    h        : 홀드 테스트 (90도에서 10초 유지, 손으로 살짝 밀어서 버티는지 확인)
    u        : 각도 제한 토글 (포탑 한계 <-> 0~180 전체). 포탑에서 분리한 서보만 전체 범위로!
    ?        : 도움말
*/

#include <Servo.h>

const int PIN_PAN = 3;
const int PIN_TILT = 9;
const int PIN_MOTOR = 5;   // 전동건 MOSFET 게이트 — 항상 LOW

// model_control.ino의 포탑 기구 한계와 동일
const int LIM_MIN[2] = {10, 20};
const int LIM_MAX[2] = {170, 160};
const char* NAMES[2] = {"pan(D3)", "tilt(D9)"};
const int PINS[2] = {PIN_PAN, PIN_TILT};

Servo servo;
int active = -1;          // -1 = 없음, 0 = pan, 1 = tilt
int curAngle = 90;
bool fullRange = false;

int minA() { return fullRange ? 0 : LIM_MIN[active]; }
int maxA() { return fullRange ? 180 : LIM_MAX[active]; }

void detachAll() {
  if (servo.attached()) servo.detach();
  // 신호선이 떠 있으면 서보가 떨 수 있어서 LOW로 고정
  pinMode(PIN_PAN, OUTPUT);  digitalWrite(PIN_PAN, LOW);
  pinMode(PIN_TILT, OUTPUT); digitalWrite(PIN_TILT, LOW);
  active = -1;
}

void selectServo(int idx) {
  detachAll();
  active = idx;
  curAngle = 90;
  servo.attach(PINS[idx]);
  servo.write(curAngle);
  Serial.print(F(">> 활성: ")); Serial.print(NAMES[idx]);
  Serial.println(F(" / 90도로 이동. 다른 서보는 신호 없음"));
}

bool needActive() {
  if (active < 0) { Serial.println(F("먼저 1(pan) 또는 2(tilt)를 선택하세요")); return false; }
  return true;
}

// 1도씩 천천히 이동 (급격한 이동으로 인한 전류 스파이크 방지)
void moveSlow(int target, int msPerDeg = 15) {
  target = constrain(target, minA(), maxA());
  int step = (target > curAngle) ? 1 : -1;
  while (curAngle != target) {
    curAngle += step;
    servo.write(curAngle);
    delay(msPerDeg);
  }
}

void report(const __FlashStringHelper* tag) {
  Serial.print(tag); Serial.print(F(" ")); Serial.print(NAMES[active]);
  Serial.print(F(" @ ")); Serial.print(curAngle); Serial.println(F("도"));
}

void sweepTest() {
  Serial.print(F("[스윕] ")); Serial.print(minA()); Serial.print(F(" -> "));
  Serial.print(maxA()); Serial.println(F(" -> 90  (끊김/걸림/멈춤 구간 확인)"));
  moveSlow(minA(), 20); report(F("  도달"));
  delay(500);
  moveSlow(maxA(), 20); report(F("  도달"));
  delay(500);
  moveSlow(90, 20);     report(F("  복귀"));
}

void stepTest() {
  Serial.println(F("[스텝] 90 -> 60 -> 120 -> 90  (각 위치에서 정확히 멈추는지, 떨림 확인)"));
  const int seq[] = {90, 60, 120, 90};
  for (int i = 0; i < 4; i++) {
    curAngle = constrain(seq[i], minA(), maxA());
    servo.write(curAngle);          // 일부러 즉시 명령 (응답 속도 비교용)
    report(F("  명령"));
    delay(1000);
  }
}

void holdTest() {
  Serial.println(F("[홀드] 90도 10초 유지. 손으로 살짝 밀어보세요 (정상: 버티면서 원위치)"));
  moveSlow(90);
  for (int s = 10; s > 0; s--) { Serial.print(s); Serial.print(' '); delay(1000); }
  Serial.println(F("\n  홀드 끝"));
}

void help() {
  Serial.println(F("--- 서보 테스트 ---"));
  Serial.println(F("1=pan(D3)  2=tilt(D9)  0=전부 해제"));
  Serial.println(F("a<deg>=각도  w=스윕  k=스텝  h=홀드  u=범위토글  ?=도움말"));
  Serial.print(F("현재 범위: ")); Serial.println(fullRange ? F("0~180 (분리 상태 전용)") : F("포탑 한계"));
}

void setup() {
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);     // 발사 금지
  Serial.begin(9600);
  Serial.setTimeout(50);
  detachAll();
  help();
}

void loop() {
  digitalWrite(PIN_MOTOR, LOW);     // 혹시 모를 상황 대비, 계속 LOW 유지
  if (!Serial.available()) return;

  String in = Serial.readStringUntil('\n');
  in.trim();
  if (in.length() == 0) return;
  char c = in.charAt(0);

  switch (c) {
    case '1': selectServo(0); break;
    case '2': selectServo(1); break;
    case '0': detachAll(); Serial.println(F(">> 전부 해제 (손으로 돌려서 걸림 확인 가능)")); break;
    case 'a': {
      if (!needActive()) break;
      String num = in.substring(1); num.trim();
      if (num.length() == 0 || !isDigit(num[0])) { Serial.println(F("예: a90")); break; }
      int t = num.toInt();
      if (t < minA() || t > maxA()) {
        Serial.print(F("범위 밖 -> ")); Serial.print(minA()); Serial.print('~'); Serial.print(maxA());
        Serial.println(F("로 제한"));
      }
      moveSlow(t); report(F(">>"));
      break;
    }
    case 'w': if (needActive()) sweepTest(); break;
    case 'k': if (needActive()) stepTest();  break;
    case 'h': if (needActive()) holdTest();  break;
    case 'u':
      fullRange = !fullRange;
      Serial.println(fullRange ? F(">> 0~180 전체 범위 (포탑에서 분리한 서보만!)") : F(">> 포탑 한계 범위"));
      break;
    case '?': help(); break;
    default: Serial.print(F("알 수 없는 명령: ")); Serial.println(in);
  }
}
