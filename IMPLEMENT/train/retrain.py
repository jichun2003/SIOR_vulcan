# -*- coding: utf-8 -*-
"""
Roboflow COCO export -> YOLO 형식 변환 후 YOLOv8 재학습.

기존 학습(best0928.pt)의 문제점과 대응:
  1) 연속 촬영 프레임이 train/val에 무작위로 섞여 val mAP50=0.995로 부풀려짐
     -> 원본 파일명 순서대로 --val-block장씩 묶어 블록 단위로 val을 떼어냄
  2) Roboflow COCO가 프로젝트명 상위 클래스(Target)를 추가해 클래스가 2개로 갈림
     -> 모든 bbox를 단일 클래스 'target'으로 통합
  3) 실제 실험 환경(어수선한 배경)에서 일반화 부족
     -> 표적이 없는 배경 이미지/영상(--negatives-dir/--negatives-video)을 빈 라벨로 추가,
        더 큰 모델(yolov8s)·해상도(960)·강한 색/스케일 증강

사용 예 (Roboflow에서 COCO로 export한 폴더: train/valid/test 각각 _annotations.coco.json 포함):
    python train/retrain.py --coco-dir ~/Downloads/vulcan-coco \
        --negatives-video ~/Downloads/empty_room.mp4
"""

import argparse
import json
import random
import shutil
from collections import defaultdict
from datetime import date
from pathlib import Path

import cv2
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--coco-dir", required=True, help="Roboflow COCO export 폴더")
    p.add_argument("--negatives-dir", default=None, help="표적이 없는 배경 이미지 폴더")
    p.add_argument("--negatives-video", default=None, help="표적이 없는 배경 영상 (프레임 추출해 사용)")
    p.add_argument("--negatives-every", type=int, default=15, help="배경 영상에서 N프레임마다 1장 추출")
    p.add_argument("--out-dir", default=str(ROOT / "dataset" / "yolo_retrain"))
    p.add_argument("--val-frac", type=float, default=0.2)
    p.add_argument("--val-block", type=int, default=40, help="연속 프레임을 이 개수씩 묶어 블록 단위로 분할")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--model", default="yolov8s.pt")
    p.add_argument("--imgsz", type=int, default=960)
    p.add_argument("--epochs", type=int, default=150)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--device", default="mps")
    p.add_argument("--prepare-only", action="store_true", help="데이터셋 변환만 하고 학습은 생략")
    return p.parse_args()


def original_name(file_name):
    # Roboflow는 "img_0012_jpg.rf.<hash>.jpg" 형태로 이름을 바꾸므로 원본 순서를 복원
    return Path(file_name).name.split(".rf.")[0]


def load_coco(coco_dir):
    """train/valid/test 분할을 모두 합쳐 [(이미지경로, w, h, [bbox...])] 반환."""
    items = []
    for ann_path in sorted(Path(coco_dir).rglob("_annotations.coco.json")):
        data = json.loads(ann_path.read_text())
        boxes = defaultdict(list)
        for a in data["annotations"]:
            boxes[a["image_id"]].append(a["bbox"])
        for img in data["images"]:
            items.append((ann_path.parent / img["file_name"], img["width"], img["height"], boxes[img["id"]]))
    if not items:
        raise RuntimeError(f"_annotations.coco.json을 찾지 못했습니다: {coco_dir}")
    return items


def block_split(items, val_frac, block, seed):
    items = sorted(items, key=lambda it: original_name(it[0].name))
    blocks = [items[i:i + block] for i in range(0, len(items), block)]
    rng = random.Random(seed)
    order = list(range(len(blocks)))
    rng.shuffle(order)
    n_val = max(1, round(len(blocks) * val_frac))
    val_ids = set(order[:n_val])
    train = [it for i, b in enumerate(blocks) if i not in val_ids for it in b]
    val = [it for i, b in enumerate(blocks) if i in val_ids for it in b]
    return train, val


def write_split(items, out_dir, split):
    img_dir = out_dir / "images" / split
    lbl_dir = out_dir / "labels" / split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)
    for src, w, h, bboxes in items:
        dst = img_dir / src.name
        shutil.copy2(src, dst)
        lines = []
        for x, y, bw, bh in bboxes:
            if bw <= 0 or bh <= 0:
                continue
            lines.append(f"0 {(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} {bw / w:.6f} {bh / h:.6f}")
        (lbl_dir / f"{dst.stem}.txt").write_text("\n".join(lines))


def collect_negatives(neg_dir, neg_video, every, tmp_dir):
    paths = []
    if neg_dir:
        paths += sorted(p for p in Path(neg_dir).iterdir() if p.suffix.lower() in IMG_EXTS)
    if neg_video:
        tmp_dir.mkdir(parents=True, exist_ok=True)
        cap = cv2.VideoCapture(neg_video)
        idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if idx % every == 0:
                out = tmp_dir / f"neg_{idx:06d}.jpg"
                cv2.imwrite(str(out), frame)
                paths.append(out)
            idx += 1
        cap.release()
    return [(p, 1, 1, []) for p in paths]


def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    if out_dir.exists():
        shutil.rmtree(out_dir)

    items = load_coco(args.coco_dir)
    train, val = block_split(items, args.val_frac, args.val_block, args.seed)

    negatives = collect_negatives(args.negatives_dir, args.negatives_video, args.negatives_every,
                                  out_dir / "_neg_frames")
    random.Random(args.seed).shuffle(negatives)
    n_neg_val = round(len(negatives) * args.val_frac)
    val += negatives[:n_neg_val]
    train += negatives[n_neg_val:]

    write_split(train, out_dir, "train")
    write_split(val, out_dir, "val")
    shutil.rmtree(out_dir / "_neg_frames", ignore_errors=True)

    data_yaml = out_dir / "data.yaml"
    data_yaml.write_text(f"path: {out_dir}\ntrain: images/train\nval: images/val\nnames:\n  0: target\n")
    print(f"train={len(train)}장, val={len(val)}장 (배경 이미지 {len(negatives)}장 포함) -> {data_yaml}")

    if args.prepare_only:
        return

    model = YOLO(args.model)
    model.train(
        data=str(data_yaml),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        device=args.device,
        patience=40,
        hsv_h=0.02, hsv_s=0.7, hsv_v=0.5,
        degrees=5.0, translate=0.1, scale=0.6,
        fliplr=0.5, mosaic=1.0, mixup=0.1, close_mosaic=15,
        project=str(ROOT / "runs"), name=f"retrain_{date.today():%m%d}",
    )

    best = Path(model.trainer.best)
    dst = ROOT / "checkpoint" / f"best{date.today():%m%d}_retrain.pt"
    shutil.copy2(best, dst)
    print(f"최종 가중치 -> {dst}")


if __name__ == "__main__":
    main()
