# -*- coding: utf-8 -*-
"""아두이노 pan/tilt/피더/플라이휠 포탑 컨트롤러와의 시리얼 통신.

프로토콜은 firmware/model_control/model_control.ino와 동일: 9600 baud,
s번호는 핀 순서(D2,D3,D9) 고정이라 기능과 아래처럼 매칭된다.
  s1 = D2 = 피더(공 미는 부분)
  s2 = D3 = pan (포탑 베이스 회전)
  s3 = D9 = tilt (포신 고저각, 총구 근처)
  v<speed> + f = 플라이휠(DC모터 2개) 목표 속도 설정 + 발사 방향 회전, x = 정지
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

    def send_fire(self, flywheel_speed, feeder_push_deg, feeder_rest_deg, spin_up_s=0.6, push_s=0.3):
        """플라이휠을 목표 속도로 돌리고, 스핀업 후 피더 서보로 공을 한 번 밀어 넣는다.

        사격은 프레임마다 일어나는 동작이 아니라 ready 이후 조준 완료 시 1회만
        일어나므로, 여기서의 짧은 대기(sleep)는 메인 추적 루프를 막지 않는다.
        """
        self._ser.write(f"v{int(flywheel_speed)}\n".encode("ascii"))
        self._ser.write(b"f\n")
        time.sleep(spin_up_s)
        self._ser.write(f"s1 {int(feeder_push_deg)}\n".encode("ascii"))
        time.sleep(push_s)
        self._ser.write(f"s1 {int(feeder_rest_deg)}\n".encode("ascii"))
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
        self._ser.close()


class DummyLink:
    """--no-serial 옵션 사용 시 TurretSerialLink 대신 쓰는 더미 구현."""

    def send_angles(self, pan_deg, tilt_deg):
        pass

    def send_fire(self, *args, **kwargs):
        pass

    def read_responses(self):
        return []

    def close(self):
        pass
