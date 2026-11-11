# -*- coding: utf-8 -*-
"""
AI-G 보드(board_app, --mode detect-only)가 TCP로 내보내는 탐지 결과를 받는 클라이언트.

aaa_code_1111의 vision.py에서 Camera + Detector가 하던 일을 대신한다. 즉 PC는 카메라도
YOLO도 돌리지 않고, 보드가 NPU로 뽑은 박스만 받아서 기존 추적/PID/시리얼 코드를 그대로 쓴다.

보드가 보내는 줄 (한 줄 = JSON 한 개, 개행 구분):
    {"t": 1234.5, "infer_ms": 15.4, "frame_w": 1280, "frame_h": 720,
     "dets": [{"cls":0,"conf":0.87,"x1":..,"y1":..,"x2":..,"y2":..}], "best": 0}

좌표는 보드 카메라의 원본 프레임 픽셀 기준이다 (보드가 NPU 입력 좌표에서 되돌려 보냄).

수신은 백그라운드 스레드에서 하고, 메인 루프는 read_latest()로 "가장 최근 프레임"만
가져간다 (밀린 프레임은 버린다 — 추적에서는 오래된 위치가 해로우므로).
"""

import json
import socket
import threading
import time


class AigDetectionLink:
    """보드의 검출 결과 스트림. 끊기면 백그라운드에서 계속 재접속한다."""

    def __init__(self, host, port=5000, target_class=0, class_names=("target",),
                 connect_timeout=3.0):
        self.host = host
        self.port = port
        self.target_class = target_class
        self.class_names = tuple(class_names)
        self._connect_timeout = connect_timeout

        self._lock = threading.Lock()
        self._latest = None        # 아직 안 읽은 최신 프레임
        self._frames = 0
        self._last_rx = 0.0
        self._connected = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    # ---------- 메인 루프에서 호출 ----------
    def read_latest(self, max_age_s=0.5):
        """가장 최근 프레임을 (dets, meta)로 반환. 없거나 너무 오래됐으면 (None, meta).

        dets는 1111 pick_target()이 쓰는 딕셔너리 형식과 같다.
        """
        with self._lock:
            frame, self._latest = self._latest, None
            meta = {
                "connected": self._connected,
                "frames": self._frames,
                "last_rx": self._last_rx,
            }
        if frame is None:
            return None, meta
        if max_age_s is not None and (time.monotonic() - frame["rx_mono"]) > max_age_s:
            return None, meta   # 통신이 밀린 프레임은 쓰지 않는다
        meta["infer_ms"] = frame.get("infer_ms", 0.0)
        meta["frame_w"] = frame["frame_w"]
        meta["frame_h"] = frame["frame_h"]
        return frame["dets"], meta

    @property
    def connected(self):
        with self._lock:
            return self._connected

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.5)

    # ---------- 백그라운드 ----------
    def _worker(self):
        while not self._stop.is_set():
            sock = None
            try:
                sock = socket.create_connection((self.host, self.port), self._connect_timeout)
                sock.settimeout(1.0)
                with self._lock:
                    self._connected = True
                print(f"[aig_link] 보드에 연결됨: {self.host}:{self.port}")
                self._read_loop(sock)
            except OSError as e:
                if not self._stop.is_set():
                    print(f"[aig_link] 연결 실패/끊김 ({e}) - 1초 후 재시도")
            finally:
                with self._lock:
                    self._connected = False
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            self._stop.wait(1.0)

    def _read_loop(self, sock):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise OSError("보드가 연결을 닫았습니다")
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if line:
                    self._handle_line(line)

    def _handle_line(self, line):
        try:
            msg = json.loads(line.decode("utf-8", errors="ignore"))
        except ValueError:
            return
        frame_w = msg.get("frame_w", 1280)
        frame_h = msg.get("frame_h", 720)
        dets = []
        for d in msg.get("dets", []):
            x1, y1 = float(d["x1"]), float(d["y1"])
            x2, y2 = float(d["x2"]), float(d["y2"])
            cls_id = int(d.get("cls", 0))
            dets.append({
                "conf": float(d.get("conf", 0.0)),
                "cls": cls_id,
                "cls_name": self.class_names[cls_id] if cls_id < len(self.class_names) else str(cls_id),
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "cx": (x1 + x2) / 2.0,
                "cy": (y1 + y2) / 2.0,
                "w": x2 - x1,
                "h": y2 - y1,
            })
        with self._lock:
            self._latest = {
                "rx_mono": time.monotonic(),
                "board_t": msg.get("t", 0.0),
                "infer_ms": msg.get("infer_ms", 0.0),
                "frame_w": frame_w,
                "frame_h": frame_h,
                "dets": dets,
            }
            self._frames += 1
            self._last_rx = time.monotonic()


def pick_target(dets, target_class=None, last_center=None, max_jump_px=None):
    """1111 vision.py의 pick_target과 같은 규칙 (여기선 YOLO 결과 객체가 아니라 dict 목록).

    target_class는 클래스 "이름"(문자열) 또는 None(필터 없음).
    """
    import math

    if not dets:
        return None
    candidates = [d for d in dets
                  if target_class is None or d["cls_name"] == target_class]
    if not candidates:
        return None
    if last_center is not None and max_jump_px:
        near = [d for d in candidates
                if math.hypot(d["cx"] - last_center[0], d["cy"] - last_center[1]) <= max_jump_px]
        if near:
            candidates = near   # 가까운 후보가 있으면 그중에서만, 없으면 전체(재포착)
    return max(candidates, key=lambda d: d["conf"])
