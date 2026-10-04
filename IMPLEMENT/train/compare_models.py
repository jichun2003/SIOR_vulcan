# -*- coding: utf-8 -*-
"""
여러 체크포인트를 같은 카메라 프레임에 동시에 돌려 좌우로 나란히 보여준다.
--models에 넘긴 순서대로 왼쪽부터 배치. 어느 쪽이 실제 환경에서 더 잘 잡는지 눈으로 비교하는 용도.

화면 하단에 지금까지의 탐지율(표적을 잡은 프레임 비율)이 누적 표시된다.
q=종료, s=현재 원본 프레임 저장(dataset/live_capture/), c=탐지율 카운터 초기화

사용 예:
    python3 IMPLEMENT/train/compare_models.py      # 기본: best0928 | best1004_retrain
    python3 IMPLEMENT/train/compare_models.py --models checkpoint/best.pt checkpoint/best1004_retrain.pt
    python3 IMPLEMENT/train/compare_models.py \
        --models checkpoint/best.pt checkpoint/best0928.pt checkpoint/best1004_retrain.pt
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--models", nargs="+",
        default=[str(ROOT / "checkpoint" / "best0928.pt"), str(ROOT / "checkpoint" / "best1004_retrain.pt")],
        help="비교할 체크포인트들 (2개 이상, 왼쪽부터 순서대로 표시)",
    )
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--conf", type=float, default=0.4, help="main.py 기본값과 동일")
    return p.parse_args()


def draw(frame, result, label, hits, total):
    out = frame.copy()
    boxes = result.boxes
    best = None
    if boxes is not None and len(boxes) > 0:
        for xyxy, conf in zip(boxes.xyxy.tolist(), boxes.conf.tolist()):
            x1, y1, x2, y2 = map(int, xyxy)
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(out, f"{conf:.2f}", (x1, max(y1 - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        best = max(boxes.conf.tolist())
    status = f"best conf {best:.2f}" if best is not None else "NO DETECTION"
    color = (0, 255, 0) if best is not None else (0, 0, 255)
    cv2.putText(out, label, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
    cv2.putText(out, status, (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    rate = hits / total if total else 0.0
    cv2.putText(out, f"detect rate {rate:.0%} ({hits}/{total})", (10, out.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return out


def main():
    args = parse_args()
    if len(args.models) < 2:
        raise ValueError("--models에는 체크포인트를 2개 이상 지정해야 합니다")
    models = [YOLO(m) for m in args.models]
    labels = [f"{chr(ord('A') + i)}: {Path(m).name}" for i, m in enumerate(args.models)]

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise RuntimeError(f"카메라를 열 수 없습니다: index={args.camera}")

    hits = [0] * len(models)
    total = 0
    capture_dir = ROOT / "dataset" / "live_capture"
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            results = [m.predict(frame, conf=args.conf, verbose=False)[0] for m in models]
            total += 1
            for i, r in enumerate(results):
                hits[i] += int(len(r.boxes) > 0)

            view = np.hstack([draw(frame, r, lbl, h, total) for r, lbl, h in zip(results, labels, hits)])
            scale = min(1800 / view.shape[1], 1.0)
            cv2.imshow("SIOR - model compare", cv2.resize(view, None, fx=scale, fy=scale))

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c"):
                hits = [0] * len(models)
                total = 0
            if key == ord("s"):
                capture_dir.mkdir(parents=True, exist_ok=True)
                out = capture_dir / f"{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_cmp.jpg"
                cv2.imwrite(str(out), frame)
                print(f"프레임 저장: {out}")
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if total:
        for lbl, h in zip(labels, hits):
            print(f"{lbl}: 탐지율 {h / total:.1%} ({h}/{total})")


if __name__ == "__main__":
    main()
