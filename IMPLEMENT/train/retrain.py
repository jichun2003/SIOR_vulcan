# -*- coding: utf-8 -*-
"""
Roboflow COCO export(여러 개) -> YOLO 형식 변환 후 YOLOv8 재학습.

기존 학습(best0928.pt)의 문제점과 대응:
  1) 연속 촬영 프레임·증강 사본이 train/val에 섞여 val mAP50=0.995로 부풀려짐
     -> 거의 같은 이미지(dHash)와 같은 원본의 사본을 한 묶음으로 만들어 묶음 단위로 분할
  2) 같은 이미지가 여러 export(v1, v3)에 중복 포함됨
     -> export 간 거의 같은 이미지는 하나만 남김
  3) Roboflow COCO가 프로젝트명 상위 클래스를 추가해 클래스가 2개로 갈림
     -> 모든 bbox를 단일 클래스 'target'으로 통합
  4) 실제 실험 환경에서 일반화 부족
     -> 표적이 없는 배경 이미지/영상(--negatives-dir/--negatives-video)을 빈 라벨로 추가,
        yolov8s + 강한 색/스케일 증강

사용 예 (--coco-dir 아래 모든 _annotations.coco.json을 찾아 합침):
    python IMPLEMENT/train/retrain.py --coco-dir coco --negatives-video empty_room.mp4
"""

import argparse
import json
import random
import shutil
from collections import Counter, defaultdict
from datetime import date
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
DUP_BITS = 2      # export 간 이 비트 수 이하로 다르면 같은 이미지로 보고 제거
GROUP_BITS = 3    # 이 비트 수 이하로 다르면 같은 묶음(연속 프레임)으로 보고 train/val을 함께 배정


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--coco-dir", required=True, help="Roboflow COCO export들이 들어있는 폴더")
    p.add_argument("--negatives-dir", default=None, help="표적이 없는 배경 이미지 폴더")
    p.add_argument("--negatives-video", default=None, help="표적이 없는 배경 영상 (프레임 추출해 사용)")
    p.add_argument("--negatives-every", type=int, default=15, help="배경 영상에서 N프레임마다 1장 추출")
    p.add_argument("--out-dir", default=str(ROOT / "dataset" / "yolo_retrain"))
    p.add_argument("--val-frac", type=float, default=0.2)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--model", default="yolov8s.pt")
    p.add_argument("--imgsz", type=int, default=640, help="Roboflow export가 512/640이라 그 이상은 업스케일일 뿐")
    p.add_argument("--epochs", type=int, default=150)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--device", default="mps")
    p.add_argument("--prepare-only", action="store_true", help="데이터셋 변환만 하고 학습은 생략")
    return p.parse_args()


def load_coco(coco_dir):
    """모든 export의 모든 분할을 합쳐 dict 리스트로 반환."""
    coco_dir = Path(coco_dir)
    items = []
    for ann_path in sorted(coco_dir.rglob("_annotations.coco.json")):
        project = ann_path.relative_to(coco_dir).parts[0]
        data = json.loads(ann_path.read_text())
        boxes = defaultdict(list)
        for a in data["annotations"]:
            boxes[a["image_id"]].append(a["bbox"])
        for img in data["images"]:
            items.append({
                "project": project,
                "path": ann_path.parent / img["file_name"],
                "w": img["width"], "h": img["height"],
                "bboxes": boxes[img["id"]],
                # Roboflow는 "img_0012_jpg.rf.<hash>.jpg"로 이름을 바꾸므로 원본 이름을 복원
                "orig": img["file_name"].split(".rf.")[0],
            })
    if not items:
        raise RuntimeError(f"_annotations.coco.json을 찾지 못했습니다: {coco_dir}")
    return items


def dhash_bits(path):
    g = cv2.resize(cv2.imread(str(path), cv2.IMREAD_GRAYSCALE), (9, 8), interpolation=cv2.INTER_AREA)
    return (g[:, 1:] > g[:, :-1]).ravel()


def dedupe_and_group(items):
    bits = np.array([dhash_bits(it["path"]) for it in items], dtype=bool)
    n = len(items)

    # 1) export 간 중복 제거 (앞서 나온 쪽을 유지)
    keep = np.ones(n, dtype=bool)
    for i in range(n):
        if not keep[i]:
            continue
        d = (bits[i + 1:] != bits[i]).sum(1)
        for j in np.nonzero(d <= DUP_BITS)[0] + i + 1:
            if keep[j] and items[j]["project"] != items[i]["project"]:
                keep[j] = False
    removed = Counter(items[j]["project"] for j in range(n) if not keep[j])
    items = [it for it, k in zip(items, keep) if k]
    bits = bits[keep]
    n = len(items)

    # 2) 같은 원본 이름(같은 export) 또는 거의 같은 이미지끼리 union-find로 묶음
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    first_by_name = {}
    for i, it in enumerate(items):
        key = (it["project"], it["orig"])
        if key in first_by_name:
            union(first_by_name[key], i)
        else:
            first_by_name[key] = i
        d = (bits[i + 1:] != bits[i]).sum(1)
        for j in np.nonzero(d <= GROUP_BITS)[0] + i + 1:
            union(i, j)

    groups = defaultdict(list)
    for i, it in enumerate(items):
        groups[find(i)].append(it)
    return items, list(groups.values()), removed


def group_split(groups, val_frac, seed):
    rng = random.Random(seed)
    groups = groups[:]
    rng.shuffle(groups)
    total = sum(len(g) for g in groups)
    target = total * val_frac
    train, val = [], []
    for g in groups:
        # 너무 큰 묶음 하나가 val을 독차지하지 않도록, 넣었을 때 목표를 크게 넘으면 train으로
        if len(val) < target and len(val) + len(g) <= target * 1.2:
            val += g
        else:
            train += g
    return train, val


def write_split(items, out_dir, split):
    img_dir = out_dir / "images" / split
    lbl_dir = out_dir / "labels" / split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)
    for k, it in enumerate(items):
        src = it["path"]
        dst = img_dir / f"{k:05d}_{src.name}"
        shutil.copy2(src, dst)
        w, h = it["w"], it["h"]
        lines = []
        for x, y, bw, bh in it["bboxes"]:
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
    return [{"path": p, "w": 1, "h": 1, "bboxes": []} for p in paths]


def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    if out_dir.exists():
        shutil.rmtree(out_dir)

    raw = load_coco(args.coco_dir)
    items, groups, removed = dedupe_and_group(raw)
    print(f"원본 {len(raw)}장 -> export 간 중복 {len(raw) - len(items)}장 제거 {dict(removed)}")
    sizes = sorted((len(g) for g in groups), reverse=True)
    print(f"{len(items)}장을 {len(groups)}개 묶음으로 그룹화 (가장 큰 묶음: {sizes[:5]})")

    train, val = group_split(groups, args.val_frac, args.seed)

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
