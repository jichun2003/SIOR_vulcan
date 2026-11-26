// AI-G -> Uno I2C 연결 확인용 스케치
// 우노를 I2C 슬레이브(주소 0x08)로 두고, 받은 바이트를 USB 시리얼(9600)로 그대로 출력함.
// 배선: 보드 SDA -> A4, 보드 SCL -> A5, GND 공통
#include <Wire.h>

const uint8_t I2C_ADDR = 0x08;

// 수신 인터럽트에서 받은 바이트를 담아 두는 버퍼 (loop에서 꺼내 출력)
volatile uint8_t rxBuf[64];
volatile uint8_t rxHead = 0, rxTail = 0;

void onI2CReceive(int n) {
  while (Wire.available()) {
    uint8_t c = Wire.read();
    uint8_t next = (rxHead + 1) & 63;
    if (next != rxTail) {      // 버퍼가 가득 차면 버림
      rxBuf[rxHead] = c;
      rxHead = next;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_ADDR);
  // Wire.begin()이 켜는 내부 풀업(5V)을 끔 -> 보드의 3.3V I2C 핀으로 5V가 들어가지 않게 함
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  Wire.onReceive(onI2CReceive);
  Serial.println(F("[i2c test] addr 0x08, SDA=A4, SCL=A5"));
}

void loop() {
  while (rxTail != rxHead) {
    uint8_t c = rxBuf[rxTail];
    rxTail = (rxTail + 1) & 63;
    Serial.print(F("[i2c "));
    Serial.print(c, HEX);
    Serial.print(F("] "));
    if (c >= 32 && c < 127) Serial.println((char)c);
    else Serial.println();
  }
}
