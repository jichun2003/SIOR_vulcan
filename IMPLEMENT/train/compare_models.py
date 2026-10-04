# -*- coding: utf-8 -*-
"""
두 체크포인트를 같은 카메라 프레임에 동시에 돌려 좌우로 나란히 보여준다.
왼쪽=--model-a, 오른쪽=--model-b. 어느 쪽이 실제 환경에서 더 잘 잡는지 눈으로 비교하는 용도.

화면 하단에 지금까지의 탐지율(표적을 잡은 프레임 비율)이 누적 표시된다.
q=종료, s=현재 원본 프레임 저장(dataset/live_capture/), c=탐지율 카운터 초기화

사용 예:
    python3 IMPLEMENT/train/compare_models.py
    python3 IMPLEMENT/train/compare_models.py --model-a checkpoint/best0928.pt \
        --model-b checkpoint/best1004_retrain.pt --camera 0 --conf 0.4
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
    p.add_argument("--model-a", default=str(ROOT / "checkpoint" / "best0928.pt"))
    p.add_argument("--model-b", default=str(ROOT / "checkpoint" / "best1004_retrain.pt"))
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
    model_a, model_b = YOLO(args.model_a), YOLO(args.model_b)
    label_a, label_b = f"A: {Path(args.model_a).name}", f"B: {Path(args.model_b).name}"

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise RuntimeError(f"카메라를 열 수 없습니다: index={args.camera}")

    hits_a = hits_b = total = 0
    capture_dir = ROOT / "dataset" / "live_capture"
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            ra = model_a.predict(frame, conf=args.conf, verbose=False)[0]
            rb = model_b.predict(frame, conf=args.conf, verbose=False)[0]
            total += 1
            hits_a += int(len(ra.boxes) > 0)
            hits_b += int(len(rb.boxes) > 0)

            view = np.hstack([draw(frame, ra, label_a, hits_a, total), draw(frame, rb, label_b, hits_b, total)])
            scale = 1600 / view.shape[1]
            cv2.imshow("SIOR - model compare (A | B)", cv2.resize(view, None, fx=scale, fy=scale))

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c"):
                hits_a = hits_b = total = 0
            if key == ord("s"):
                capture_dir.mkdir(parents=True, exist_ok=True)
                out = capture_dir / f"{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_cmp.jpg"
                cv2.imwrite(str(out), frame)
                print(f"프레임 저장: {out}")
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if total:
        print(f"{label_a}: 탐지율 {hits_a / total:.1%} ({hits_a}/{total})")
        print(f"{label_b}: 탐지율 {hits_b / total:.1%} ({hits_b}/{total})")


if __name__ == "__main__":
    main()
