# -*- coding: utf-8 -*-
"""아두이노 pan/tilt + 전동건 DC모터 포탑 컨트롤러와의 시리얼 통신.

프로토콜은 firmware/model_control/model_control.ino와 동일 (9600 baud):
  s2 <deg> = pan  (D3, 포탑 베이스 회전)
  s3 <deg> = tilt (D9, 포신 고저각)
  v<0-255> = 전동건 모터 PWM 세기
  t<ms>    = ms 동안 사격 (펌웨어가 시간이 지나면 스스로 모터를 끔)
  x        = 즉시 사격 정지

사격 명령은 한 번 보내고 바로 돌아오므로(대기 없음) 사격 중에도 추적 루프가 계속 돈다.
"""

import time

import serial


class TurretSerialLink:
    def __init__(self, port, baud=9600, timeout=0.05, send_interval=0.03):
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self._send_interval = send_interval
        self._last_sent = 0.0
        time.sleep(2.0)  # 포트를 열면 아두이노가 자동 리셋되므로 부팅 대기

    def send_angles(self, pan_deg, tilt_deg):
        now = time.monotonic()
        if now - self._last_sent < self._send_interval:
            return
        self._last_sent = now
        self._ser.write(f"s2 {round(pan_deg)}\n".encode("ascii"))
        self._ser.write(f"s3 {round(tilt_deg)}\n".encode("ascii"))

    def send_fire(self, duration_s, motor_speed=255):
        """전동건 모터를 duration_s초 동안 돌린다 (논블로킹, 정지는 펌웨어가 처리)."""
        ms = max(1, int(round(duration_s * 1000)))
        self._ser.write(f"v{int(motor_speed)}\n".encode("ascii"))
        self._ser.write(f"t{ms}\n".encode("ascii"))

    def stop_fire(self):
        self._ser.write(b"x\n")

    def read_responses(self):
        """아두이노가 보낸 응답 라인을 모두 읽어 리스트로 반환 (없으면 빈 리스트)."""
        lines = []
        while self._ser.in_waiting:
            raw = self._ser.readline().decode("ascii", errors="ignore").strip()
            if raw:
                lines.append(raw)
        return lines

    def close(self):
        try:
            self.stop_fire()  # 종료 시 모터가 돌고 있으면 확실히 정지
        finally:
            self._ser.close()


class DummyLink:
    """--no-serial 옵션 사용 시 TurretSerialLink 대신 쓰는 더미 구현."""

    def send_angles(self, pan_deg, tilt_deg):
        pass

    def send_fire(self, *args, **kwargs):
        pass

    def stop_fire(self):
        pass

    def read_responses(self):
        return []

    def close(self):
        pass
