# -*- coding: utf-8 -*-
"""카메라 없이 시리얼만 확인: 아두이노 응답을 출력하고 pan/tilt를 좌우/상하로 움직여 본다.
사용: python test_serial.py COM3            (Windows)
      python3 test_serial.py                 (Mac: 아두이노 포트 자동 탐색)
      python3 test_serial.py /dev/cu.usbmodem1101
"""
import sys
import time

import serial

from serial_link import find_arduino_port

port = sys.argv[1] if len(sys.argv) > 1 else (find_arduino_port() or "COM3")
ser = serial.Serial(port, 9600, timeout=0.1, write_timeout=0.5)
print(f"{port} 열림, 아두이노 부팅 대기...")
time.sleep(2.0)


def dump():
    while ser.in_waiting:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if line:
            print("[arduino]", line)


dump()
for cmd in ["c", "s2 60", "s2 120", "s2 90", "s3 60", "s3 120", "s3 90", "s2 abc"]:
    print(">>", cmd)
    ser.write((cmd + "\n").encode("ascii"))
    time.sleep(1.0)
    dump()
ser.close()
print("끝")
