# -*- coding: utf-8 -*-
"""아두이노 pan/tilt + 전동건 DC모터 포탑 컨트롤러와의 시리얼 통신.

프로토콜은 firmware/model_control/model_control.ino와 동일 (9600 baud):
  s2 <deg> = pan  (D3, 포탑 베이스 회전)
  s3 <deg> = tilt (D9, 포신 고저각)
  v<0-255> = 전동건 모터 PWM 세기
  t<ms>    = ms 동안 사격 (펌웨어가 시간이 지나면 스스로 모터를 끔)
  x        = 즉시 사격 정지

모든 시리얼 입출력은 백그라운드 스레드에서 한다. 메인 루프는 "최신 각도"와 명령을
넘겨놓기만 하고 바로 돌아오므로, USB 드라이버가 멈추거나 아두이노가 리셋돼도
영상 창/키 입력/추적 루프는 멈추지 않는다.
"""

import glob
import sys
import threading
import time
from collections import deque

import serial


def find_arduino_port():
    """Mac/Linux에서 아두이노로 보이는 USB 시리얼 포트를 찾는다. 정확히 하나일 때만 반환, 아니면 None.
    (Windows는 COM 번호를 자동으로 고르지 않으므로 --port로 지정)"""
    if sys.platform == "win32":
        return None
    if sys.platform == "darwin":
        cands = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*")
    else:
        cands = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    cands = sorted(set(cands))
    if len(cands) == 1:
        return cands[0]
    if cands:
        print(f"[serial] 포트 후보가 여러 개입니다: {cands} -> --port로 지정하세요")
    return None


class TurretSerialLink:
    def __init__(self, port, baud=9600, timeout=0.01, write_timeout=0.1, send_interval=0.03):
        print(f"[serial] {port} 여는 중...")
        self._ser = serial.Serial(port, baud, timeout=timeout, write_timeout=write_timeout)
        self._send_interval = send_interval
        self._timeouts = 0
        self._dead = False

        self._lock = threading.Lock()
        self._angles = None          # 아직 안 보낸 최신 각도 (덮어쓰기: 밀린 각도는 버림)
        self._cmds = deque()         # 사격/정지 명령 (순서대로 전부 보냄)
        self._responses = deque()    # 아두이노가 보낸 줄
        self._wake = threading.Event()
        self._stop = threading.Event()

        time.sleep(2.0)  # 포트를 열면 아두이노가 자동 리셋되므로 부팅 대기
        self._read_into_queue()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        print(f"[serial] {port} 연결됨")

    # ---------- 메인 루프에서 호출 (절대 막히지 않음) ----------
    def send_angles(self, pan_deg, tilt_deg):
        with self._lock:
            self._angles = (round(pan_deg), round(tilt_deg))
        self._wake.set()

    def send_fire(self, duration_s, motor_speed=255):
        """전동건 모터를 duration_s초 동안 돌린다 (논블로킹, 정지는 펌웨어가 처리)."""
        ms = max(1, int(round(duration_s * 1000)))
        with self._lock:
            self._cmds.append(f"v{int(motor_speed)}\nt{ms}\n".encode("ascii"))
        self._wake.set()

    def stop_fire(self):
        with self._lock:
            self._cmds.append(b"x\n")
        self._wake.set()

    def read_responses(self):
        """아두이노가 보낸 응답 라인을 모두 꺼내 리스트로 반환 (없으면 빈 리스트)."""
        with self._lock:
            lines = list(self._responses)
            self._responses.clear()
        return lines

    def close(self):
        self.stop_fire()  # 종료 시 모터가 돌고 있으면 확실히 정지
        time.sleep(0.1)   # 워커가 정지 명령을 보낼 시간
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=1.0)
        try:
            self._ser.close()
        except serial.SerialException:
            pass

    # ---------- 백그라운드 스레드 ----------
    def _worker(self):
        last_angle_sent = 0.0
        while not self._stop.is_set():
            self._wake.wait(self._send_interval)
            self._wake.clear()
            if self._dead:
                continue

            with self._lock:
                cmds = list(self._cmds)
                self._cmds.clear()
            for data in cmds:
                self._write(data)

            now = time.monotonic()
            if now - last_angle_sent >= self._send_interval:
                with self._lock:
                    angles, self._angles = self._angles, None
                if angles is not None:
                    last_angle_sent = now
                    self._write(f"s2 {angles[0]}\ns3 {angles[1]}\n".encode("ascii"))

            self._read_into_queue()

    def _write(self, data):
        if self._dead:
            return
        try:
            self._ser.write(data)
        except serial.SerialTimeoutException:
            self._timeouts += 1
            if self._timeouts == 1 or self._timeouts % 50 == 0:
                print(f"[serial] 쓰기 타임아웃 {self._timeouts}회 (아두이노 응답 없음/USB 끊김/전원 부족 의심)")
            try:
                self._ser.reset_output_buffer()
            except serial.SerialException:
                pass
        except serial.SerialException as e:
            self._dead = True
            print(f"[serial] 연결 끊김, 이후 명령 무시: {e}")

    def _read_into_queue(self):
        if self._dead:
            return
        try:
            while self._ser.in_waiting:
                raw = self._ser.readline().decode("ascii", errors="ignore").strip()
                if raw:
                    with self._lock:
                        self._responses.append(raw)
        except serial.SerialException as e:
            self._dead = True
            print(f"[serial] 연결 끊김, 이후 명령 무시: {e}")


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
