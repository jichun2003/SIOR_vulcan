/*
  이동 표적 구동 펌웨어 — Arduino Uno + L298N(채널 A) + TT 기어드 DC모터 1개.
  보드 2개에 "동일하게" 업로드한다. PC가 USB 시리얼로 두 보드를 동기 구동.

  명령 체계 (시리얼, 115200):
    한 줄에 정수 하나(-255 ~ 255)를 보낸다.
      +값 = 감기(줄을 내 쪽으로 당김),  -값 = 풀기,  0 = 정지
    크기 = 모터 PWM 세기(0~255).
    두 보드 모두 "+ = 내 쪽으로 당김"으로 통일한다.
    방향이 반대로 돌면 그 보드의 모터선(OUT1/OUT2) 2가닥을 바꿔 꽂는다.

  안전장치(watchdog): 3초 동안 명령이 안 오면 자동 정지(줄 엉킴/통신 끊김 대비).

  배선 (보드당, L298N 채널 A):
    L298N ENA  -> D9 (PWM)   ※ ENA 점퍼 캡 반드시 제거 (안 빼면 속도조절 안 됨)
    L298N IN1  -> D7
    L298N IN2  -> D8
    L298N OUT1/OUT2 -> TT 모터 2선
    L298N +12V -> 모터 배터리 +(5~6V),  GND -> 배터리 - 와 Uno GND 공통(필수)
    5V-EN 점퍼는 그대로 두고, 모터를 Uno의 5V에서 끌어쓰지 말 것.
*/

const int ENA = 9;   // PWM (속도)
const int IN1 = 7;   // 방향
const int IN2 = 8;   // 방향
const unsigned long CMD_TIMEOUT_MS = 3000;   // 명령 끊기면 정지

int currentPwm = 0;              // 부호 포함 현재 명령 (-255~255)
unsigned long lastCmdMs = 0;

void applyMotor(int v) {
  if (v > 255) v = 255;
  if (v < -255) v = -255;
  currentPwm = v;
  if (v > 0) {                   // 감기(당김)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, v);
  } else if (v < 0) {            // 풀기
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -v);
  } else {                       // 정지
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  applyMotor(0);                 // 부팅 직후 확실히 정지
  Serial.begin(115200);
  Serial.setTimeout(20);
  lastCmdMs = millis();
  Serial.println(F("target_motor ready (+=wind -=unwind 0=stop, 3s watchdog)"));
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0) {
      int v = s.toInt();         // 숫자가 아니면 0 -> 정지
      applyMotor(v);
      lastCmdMs = millis();
      Serial.println(currentPwm);   // 확인용 에코
    }
  }
  // watchdog: 명령이 끊기면 안전 정지
  if (currentPwm != 0 && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    applyMotor(0);
    Serial.println(F("watchdog stop"));
  }
}
