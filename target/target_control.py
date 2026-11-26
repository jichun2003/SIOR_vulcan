# -*- coding: utf-8 -*-
"""
이동 표적 왕복 제어 — Arduino Uno 2개(각 L298N + TT모터)를 USB 시리얼로 동기 구동한다.

기구: 양 끝 모터 2개를 줄로 타겟에 연결(줄 중간에 고무줄 한 토막) -> 좌우 왕복.
동작: 매 순간 한 모터는 감고 반대 모터는 같은 속도로 푼다.
    왼쪽 이동  = A 감기(+) + B 풀기(-)
    오른쪽 이동 = A 풀기(-) + B 감기(+)
    좌 -> 정지 -> 우 -> 정지 를 반복.
펌웨어(firmware/target_motor, 두 보드 동일): +=감기(당김), -=풀기, 0=정지, 3초 watchdog.

사용 예 (target 폴더에서):
  # 포트 먼저 확인
  python3 target_control.py --list
  # 왕복 시작 (Mac)
  python3 target_control.py --port-a /dev/cu.usbserial-10 --port-b /dev/cu.usbserial-20 \
      --speed 180 --stroke 1.5 --pause 0.4 --kb 1.0
  # (Windows) python target_control.py --port-a COM3 --port-b COM4 --speed 180 --stroke 1.5

  # 방향 확인: A 보드만 1.5초 감기(당김). 반대로 돌면 그 보드 모터선 2가닥 교체.
  python3 target_control.py --port-a /dev/cu.usbserial-10 --port-b /dev/cu.usbserial-20 --test a

튜닝:
  --speed(S)  : 왕복 속도(PWM 0~255). TT모터는 너무 낮으면(≲80) 안 움직일 수 있음.
  --stroke(T) : 한 방향 이동 시간(초). 스풀 굵기 변화로 어긋남이 쌓이므로 짧게가 유리.
  --pause     : 양 끝에서 멈추는 시간(초).
  --kb        : 좌우 속도 편차 보정. 한쪽이 빠르면 그 보드 속도에 곱함(--kb-board로 대상 지정).
Ctrl+C = 양쪽 정지 후 종료.
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial이 필요합니다: pip3 install pyserial  (또는 pip install -r requirements.txt)")


def list_serial():
    ports = list(list_ports.comports())
    if not ports:
        print("시리얼 포트를 찾지 못했습니다. 아두이노 USB 연결을 확인하세요.")
        return
    print("시리얼 포트 목록:")
    for p in ports:
        print(f"  {p.device}   ({p.description})")


def open_board(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"포트를 열 수 없습니다: {port}  ({e})")
    time.sleep(2.0)            # 포트를 열면 아두이노가 리셋됨 -> 부팅 대기
    ser.reset_input_buffer()
    return ser


def clamp(v):
    return max(-255, min(255, int(round(v))))


def send(ser, value):
    ser.write(f"{clamp(value)}\n".encode("ascii"))


def run_phase(a, b, va, vb, duration, resend=1.0):
    """duration초 동안 A=va, B=vb 명령을 유지한다.
    펌웨어 watchdog(3초)에 안 걸리도록 resend초마다 같은 명령을 다시 보낸다."""
    end = time.time() + duration
    while True:
        send(a, va)
        send(b, vb)
        remain = end - time.time()
        if remain <= 0:
            break
        time.sleep(min(resend, remain))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--list", action="store_true", help="시리얼 포트 목록만 출력하고 종료")
    p.add_argument("--port-a", help="A 보드 포트 (Mac: /dev/cu.usbserial-..., Win: COM3)")
    p.add_argument("--port-b", help="B 보드 포트")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--speed", type=int, default=180, help="왕복 속도 PWM 0~255 (S)")
    p.add_argument("--stroke", type=float, default=1.5, help="한 방향 이동 시간(초) (T)")
    p.add_argument("--pause", type=float, default=0.4, help="양 끝 정지 시간(초)")
    p.add_argument("--kb", type=float, default=1.0, help="좌우 속도 편차 보정 배수")
    p.add_argument("--kb-board", choices=["a", "b"], default="b", help="--kb를 곱할 보드 (기본 b)")
    p.add_argument("--cycles", type=int, default=0, help="왕복 횟수 (0=무한, Ctrl+C로 종료)")
    p.add_argument("--test", choices=["a", "b"], help="해당 보드만 --speed로 1.5초 감기(방향 확인용)")
    return p.parse_args()


def main():
    args = parse_args()
    if args.list:
        list_serial()
        return
    if not args.port_a or not args.port_b:
        sys.exit("--port-a 와 --port-b 가 필요합니다. 포트는 --list 로 확인하세요.")

    a = open_board(args.port_a, args.baud)
    b = open_board(args.port_b, args.baud)
    S = args.speed

    def spd(board, base):
        """base(부호 포함)에 해당 보드면 kb를 곱해 좌우 편차 보정."""
        return base * args.kb if board == args.kb_board else base

    try:
        if args.test:
            ser = a if args.test == "a" else b
            print(f"[test] 보드 {args.test.upper()} 를 {S}로 1.5초 감기 (반대로 돌면 그 보드 모터선 교체)")
            end = time.time() + 1.5
            while time.time() < end:
                send(ser, S)
                time.sleep(0.5)
            send(ser, 0)
            return

        print(f"왕복 시작: speed={S}, stroke={args.stroke}s, pause={args.pause}s, "
              f"kb={args.kb}(board {args.kb_board}).  Ctrl+C로 정지.")
        n = 0
        while args.cycles == 0 or n < args.cycles:
            # 왼쪽: A 감기(+S), B 풀기(-S)
            run_phase(a, b, spd("a", S), spd("b", -S), args.stroke)
            send(a, 0); send(b, 0); time.sleep(args.pause)
            # 오른쪽: A 풀기(-S), B 감기(+S)
            run_phase(a, b, spd("a", -S), spd("b", S), args.stroke)
            send(a, 0); send(b, 0); time.sleep(args.pause)
            n += 1
    except KeyboardInterrupt:
        pass
    finally:
        for ser in (a, b):
            try:
                send(ser, 0)
            except Exception:
                pass
        time.sleep(0.2)
        a.close()
        b.close()
        print("\n정지 및 종료")


if __name__ == "__main__":
    main()
