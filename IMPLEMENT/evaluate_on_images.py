# -*- coding: utf-8 -*-
"""
학습 때와 동일한 조건(imgsz=640, train_and_export.py 기준)으로 dataset/target 폴더의
사진들에 대해 현재 체크포인트(best.pt)가 표적을 제대로 탐지하는지 검증한다.

주의: dataset/target 사진은 Roboflow 라벨링에 쓰인 원본 소스 사진이므로 완전한
홀드아웃(hold-out) 테스트셋은 아니다. 순수한 일반화 성능이 아니라 "학습이 의도대로
됐는지"를 눈으로 확인하는 스모크 테스트 용도로 사용한다.

사용 예:
    python evaluate_on_images.py
    python evaluate_on_images.py --images-dir ../dataset/target --conf 0.25
"""

import argparse
import csv
from pathlib import Path

import cv2
from ultralytics import YOLO

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--model", default="../checkpoint/best.pt")
    p.add_argument("--images-dir", default="../dataset/target")
    p.add_argument("--out-dir", default="../dataset/target_eval")
    p.add_argument(
        "--imgsz", type=int, default=640,
        help="train_and_export.py 학습 시 imgsz와 동일하게 유지해야 공정한 비교가 됨",
    )
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--no-annotated", action="store_true", help="주석 이미지를 저장하지 않음 (CSV만 생성)")
    return p.parse_args()


def size_bucket(area_ratio):
    """bbox 면적 / 전체 프레임 면적 비율로 근거리/원거리를 대략 추정하는 프록시."""
    if area_ratio >= 0.05:
        return "near(large)"
    if area_ratio >= 0.01:
        return "mid"
    return "far(small)"


def main():
    args = parse_args()
    model = YOLO(args.model)

    images_dir = Path(args.images_dir)
    image_paths = sorted(p for p in images_dir.iterdir() if p.suffix.lower() in IMG_EXTS)
    if not image_paths:
        raise RuntimeError(f"이미지가 없습니다: {images_dir}")

    out_dir = Path(args.out_dir)
    save_annotated = not args.no_annotated
    if save_annotated:
        out_dir.mkdir(parents=True, exist_ok=True)

    csv_path = Path(args.out_dir) / "eval_summary.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    rows = []

    detected_count = 0
    confs = []
    bucket_counts = {"near(large)": 0, "mid": 0, "far(small)": 0}

    for img_path in image_paths:
        frame = cv2.imread(str(img_path))
        if frame is None:
            continue
        h, w = frame.shape[:2]

        result = model.predict(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        boxes = result.boxes

        if boxes is not None and len(boxes) > 0:
            best_idx = int(boxes.conf.argmax())
            conf = float(boxes.conf[best_idx])
            x1, y1, x2, y2 = boxes.xyxy[best_idx].tolist()
            area_ratio = ((x2 - x1) * (y2 - y1)) / (w * h)
            bucket = size_bucket(area_ratio)

            detected_count += 1
            confs.append(conf)
            bucket_counts[bucket] += 1
            rows.append([img_path.name, 1, f"{conf:.3f}", f"{area_ratio:.4f}", bucket])

            if save_annotated:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(
                    frame, f"target {conf:.2f}", (int(x1), max(int(y1) - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
        else:
            rows.append([img_path.name, 0, "", "", ""])

        if save_annotated:
            cv2.imwrite(str(out_dir / img_path.name), frame)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "detected", "conf", "bbox_area_ratio", "size_bucket"])
        writer.writerows(rows)

    total = len(image_paths)
    detect_rate = detected_count / total if total else 0.0
    mean_conf = sum(confs) / len(confs) if confs else float("nan")

    print(f"검증 이미지 수: {total}")
    print(f"탐지된 이미지 수: {detected_count} ({detect_rate:.1%})")
    print(f"평균 신뢰도: {mean_conf:.3f}" if confs else "평균 신뢰도: 탐지 없음")
    print(f"거리(bbox 크기) 구간별 탐지 수: {bucket_counts}")
    if save_annotated:
        print(f"주석 이미지 -> {out_dir}")
    print(f"요약 CSV -> {csv_path}")


if __name__ == "__main__":
    main()
