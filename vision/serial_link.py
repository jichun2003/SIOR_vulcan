# -*- coding: utf-8 -*-
"""아두이노 pan/tilt 포탑 컨트롤러와의 시리얼 통신.

프로토콜은 ardoino/ 폴더의 실제 배선용 테스트 스케치(및 firmware/turret_control.ino)와
동일: 9600 baud, s번호는 핀 순서(D2,D3,D9) 고정이라 기능과 아래처럼 매칭된다.
  s1 = D2 = 피더(공 미는 부분, 여기서는 안 씀)
  s2 = D3 = pan (포탑 베이스 회전)
  s3 = D9 = tilt (포신 고저각, 총구 근처)
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

    def close(self):
        self._ser.close()


class DummyLink:
    """--no-serial 옵션 사용 시 TurretSerialLink 대신 쓰는 더미 구현."""

    def send_angles(self, pan_deg, tilt_deg):
        pass

    def close(self):
        pass
