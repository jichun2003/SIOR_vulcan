# -*- coding: utf-8 -*-
"""
AI-G 보드가 보고 있는 카메라 화면을 PC에서 실시간으로 본다 (탐지 박스 + 포탑 상태 오버레이).

보드 앱을 --preview-port로 실행해야 한다 (standalone / detect-only 둘 다 가능):
    ./aig_turret --mode standalone --network /home/root/yolov8s_target_quantized --preview-port 5001
    ./aig_turret --mode detect-only --network /home/root/yolov8s_target_quantized --json-port 5000 --preview-port 5001

PC (Mac/Windows 공통, opencv-python만 있으면 됨):
    python3 view_aig.py --board 192.168.0.100
    python3 view_aig.py --board 192.168.0.100 --port 5001 --zoom 2

detect-only + main_aig.py(PC 제어)와 동시에 띄워도 된다 (포트가 달라서 서로 영향 없음).

키: q=종료, s=현재 원본 프레임 저장(../../dataset/live_capture/aig_<시각>/, 재학습용),
    c=빨강/파랑 바꾸기 (보드 카메라 버퍼 색 순서가 다를 때)

화면 표시:
  초록 박스 = 보드가 추적 중인 표적(best), 노랑 = 그 밖의 탐지
  상단 = 모드 / 사격 상태 / 서보각, 하단 = 보드 루프 fps / NPU 추론 ms / 영상 수신 fps / 영상 지연
"""

import argparse
import json
import socket
import struct
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from config import Config

HEADER = struct.Struct("<4sIIB3x")   # "AIGF", meta_len, img_len, fmt
FMT_RGB, FMT_JPEG = 0, 1
CAPTURE_ROOT = Path(__file__).resolve().parents[2] / "dataset" / "live_capture"


def recv_exact(sock, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError("보드가 연결을 끊었습니다")
        got += k
    return bytes(buf)


class PreviewReceiver:
    """백그라운드에서 보드에 접속해 최신 프레임만 보관 (끊기면 계속 재접속)."""

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.status = "접속 중..."
        self._lock = threading.Lock()
        self._latest = None
        self._rx_times = []
        self._stop = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()

    def latest(self):
        with self._lock:
            return self._latest

    def rx_fps(self):
        with self._lock:
            ts = [t for t in self._rx_times if time.monotonic() - t < 2.0]
            self._rx_times = ts
        return len(ts) / 2.0

    def stop(self):
        self._stop.set()

    def _worker(self):
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.host, self.port), timeout=3.0) as sock:
                    sock.settimeout(5.0)
                    self.status = f"연결됨 {self.host}:{self.port}"
                    while not self._stop.is_set():
                        magic, meta_len, img_len, fmt = HEADER.unpack(recv_exact(sock, HEADER.size))
                        if magic != b"AIGF":
                            raise ConnectionError("프레임 형식이 맞지 않습니다 (보드 앱 버전 확인)")
                        meta = json.loads(recv_exact(sock, meta_len))
                        data = recv_exact(sock, img_len)
                        if fmt == FMT_JPEG:
                            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                        else:
                            rgb = np.frombuffer(data, np.uint8).reshape(meta["img_h"], meta["img_w"], 3)
                            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                        if img is None:
                            continue
                        with self._lock:
                            self._latest = (img, meta, time.monotonic())
                            self._rx_times.append(time.monotonic())
            except (OSError, ConnectionError, ValueError) as e:
                self.status = f"재접속 대기 ({e})"
                time.sleep(1.0)


def draw(img, meta, rx_fps, zoom):
    out = img.copy()
    sx = meta["img_w"] / float(meta["frame_w"])
    sy = meta["img_h"] / float(meta["frame_h"])
    for i, d in enumerate(meta.get("dets", [])):
        color = (0, 255, 0) if i == meta.get("best", -1) else (0, 220, 255)
        p1 = (int(d["x1"] * sx), int(d["y1"] * sy))
        p2 = (int(d["x2"] * sx), int(d["y2"] * sy))
        cv2.rectangle(out, p1, p2, color, 2)
        cv2.putText(out, f"{d['conf']:.2f}", (p1[0], max(12, p1[1] - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    h, w = out.shape[:2]
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
    if zoom != 1.0:
        out = cv2.resize(out, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_LINEAR)
    top = (f"{meta.get('mode', '?')}  fire={meta.get('fire', '?')}  "
           f"pan={meta.get('pan', 0):.1f} tilt={meta.get('tilt', 0):.1f}")
    bottom = (f"board {meta.get('fps', 0):.1f}fps  npu {meta.get('infer_ms', 0):.1f}ms  "
              f"view {rx_fps:.1f}fps  det {len(meta.get('dets', []))}")
    for text, y in ((top, 22), (bottom, out.shape[0] - 10)):
        cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--board", default=Config.board_host, help=f"AI-G 보드 IP (기본 {Config.board_host})")
    p.add_argument("--port", type=int, default=5001, help="보드 --preview-port (기본 5001)")
    p.add_argument("--zoom", type=float, default=1.0, help="창 확대 배율 (영상이 작을 때 2)")
    p.add_argument("--swap-rb", action="store_true",
                   help="빨강/파랑이 바뀌어 보이면 사용 (보드 카메라 버퍼가 BGR인 경우). 실행 중 c 키로도 전환")
    args = p.parse_args()

    rx = PreviewReceiver(args.board, args.port)
    win = f"AI-G {args.board}:{args.port}"
    save_dir = None
    n_saved = 0
    last_shown = None
    swap_rb = args.swap_rb
    try:
        while True:
            frame = rx.latest()
            if frame is None:
                blank = np.zeros((360, 640, 3), np.uint8)
                cv2.putText(blank, rx.status[:70], (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(win, blank)
            elif frame is not last_shown:
                img, meta, _ = frame
                if swap_rb:
                    img = img[:, :, ::-1]
                cv2.imshow(win, draw(img, meta, rx.rx_fps(), args.zoom))
                last_shown = frame
            key = cv2.waitKey(10) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c"):
                swap_rb = not swap_rb
                last_shown = None
            if key == ord("s") and frame is not None:
                if save_dir is None:
                    save_dir = CAPTURE_ROOT / f"aig_{datetime.now():%m%d_%H%M%S}"
                    save_dir.mkdir(parents=True, exist_ok=True)
                path = save_dir / f"{n_saved:04d}.jpg"
                cv2.imwrite(str(path), frame[0])
                n_saved += 1
                print(f"저장: {path}")
    finally:
        rx.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
