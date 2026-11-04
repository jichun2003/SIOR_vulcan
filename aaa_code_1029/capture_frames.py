# -*- coding: utf-8 -*-
"""
재학습용 카메라 프레임 자동 저장 (키보드에서 멀리 떨어져 있어도 촬영 가능).

카운트다운 후 --interval초마다 원본 프레임을 저장한다. 화면에는 현재 모델의 탐지 결과를
같이 보여주지만, 저장되는 것은 오버레이 없는 원본이다.

    # 표적 없는 장면 (오탐 제거용). 카메라를 조금씩 돌려가며
    python3 capture_frames.py --name empty --duration 60

    # 표적을 들고 거리/높이/배경을 바꿔가며
    python3 capture_frames.py --name target --duration 120

저장 위치: ../dataset/live_capture/<name>_<시각>/
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2

from config import Config
from vision import Camera, Detector

OUT_ROOT = Path(__file__).resolve().parent.parent / "dataset" / "live_capture"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", required=True, help="폴더 이름 앞부분 (예: empty, target)")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--interval", type=float, default=0.5, help="저장 간격(초)")
    p.add_argument("--duration", type=float, default=60, help="촬영 시간(초)")
    p.add_argument("--countdown", type=float, default=10, help="시작 전 대기(초). 자리로 이동할 시간")
    p.add_argument("--no-detect", action="store_true", help="화면에 탐지 결과를 표시하지 않음 (더 빠름)")
    args = p.parse_args()

    cfg = Config()
    out_dir = OUT_ROOT / f"{args.name}_{datetime.now():%m%d_%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)
    cam = Camera(args.camera, cfg.frame_width, cfg.frame_height)
    det = None if args.no_detect else Detector(cfg.model_path, cfg.conf_threshold, cfg.target_class, cfg.imgsz)

    t0 = time.time()
    last_save = 0.0
    n = 0
    while True:
        ok, frame = cam.read()
        if not ok:
            print("카메라 프레임을 읽지 못했습니다.", file=sys.stderr)
            break
        now = time.time() - t0
        view = frame.copy()
        if det is not None:
            for b in det.predict(frame).boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(view, f"{float(b.conf):.2f}", (x1, max(15, y1 - 5)), 0, 0.6, (0, 255, 0), 2)
        if now < args.countdown:
            msg = f"start in {args.countdown - now:.0f}s"
        elif now < args.countdown + args.duration:
            if now - last_save >= args.interval:
                cv2.imwrite(str(out_dir / f"{n:04d}.jpg"), frame)
                n += 1
                last_save = now
            msg = f"REC {n} saved, {args.countdown + args.duration - now:.0f}s left"
        else:
            break
        cv2.putText(view, msg, (20, 50), 0, 1.2, (0, 0, 255), 3)
        cv2.imshow("capture", view)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cam.release()
    cv2.destroyAllWindows()
    print(f"{n}장 저장 -> {out_dir}")


if __name__ == "__main__":
    main()
