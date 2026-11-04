# -*- coding: utf-8 -*-
"""
먼 거리(작은 표적) / 어두운 배경 / 무늬 배경 합성 데이터 생성.

best1004_retrain.pt는 검증셋에선 99%였지만 실제 실험에서 2m 이상 멀어지거나 배경이
어둡거나 무늬가 있으면 놓쳤다. 1280x720 화면에서 표적 폭이 50px 이하로 작아지면
640 추론 기준 탐지율이 40% 아래로 떨어지는데, 학습 데이터에 그만큼 작은 표적이 거의 없기 때문.

방법:
  1) retrain.py가 만든 YOLO 데이터셋(라벨 있는 실제 사진)에서 표적을 GrabCut으로 오려냄
  2) 배경(실제 학습 사진 / --bg-dir의 일반 사진 / 절차적 무늬)에 작게 붙임
     - 표적 폭 12~120px(1280 기준, 로그 균등), 회전, 좌우 모션 블러, 줄(끈) 그리기
     - 절반 정도는 화면 전체를 어둡게(감마) + 센서 노이즈
  3) 붙인 위치를 YOLO 라벨로 저장. 실제 사진 배경이면 원래 라벨도 유지

train 분할의 표적은 train 합성에만, val 분할의 표적은 val 합성에만 사용해 누수를 막는다.

사용 예:
    python Imp_Window\\train\\retrain.py --coco-dir coco --prepare-only
    python Imp_Window\\train\\make_synthetic.py --bg-dir coco128\\images --out-dir dataset\\synthetic
    python Imp_Window\\train\\retrain.py --coco-dir coco --extra-dir dataset/synthetic \
        --model checkpoint/best1004_retrain.pt --imgsz 960 --epochs 50
"""

import argparse
import math
import random
import shutil
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
CANVAS_W, CANVAS_H = 1280, 720


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src", default=str(ROOT / "dataset" / "yolo_retrain"), help="retrain.py가 만든 YOLO 데이터셋")
    p.add_argument("--bg-dir", default=None, help="표적이 없는 일반 배경 사진 폴더 (하위 폴더 포함)")
    p.add_argument("--out-dir", default=str(ROOT / "dataset" / "synthetic"))
    p.add_argument("--n-train", type=int, default=2500)
    p.add_argument("--n-val", type=int, default=400)
    p.add_argument("--min-w", type=float, default=12, help="표적 최소 폭(px, 1280 기준)")
    p.add_argument("--max-w", type=float, default=120)
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args()


def read_labels(img_path):
    lp = img_path.parent.parent.parent / "labels" / img_path.parent.name / (img_path.stem + ".txt")
    if not lp.exists():
        return []
    return [list(map(float, l.split()[1:])) for l in lp.read_text().splitlines() if l.strip()]


def extract_cutouts(split_dir, rng, limit=600):
    """라벨 박스가 충분히 큰 사진에서 GrabCut으로 표적 RGBA 조각을 뽑는다."""
    cands = []
    for p in sorted(split_dir.iterdir()):
        b = read_labels(p)
        if len(b) == 1 and b[0][2] * b[0][3] > 0.02:
            cands.append((p, b[0]))
    rng.shuffle(cands)
    cuts = []
    for p, (x, y, w, h) in cands:
        if len(cuts) >= limit:
            break
        im = cv2.imread(str(p))
        H, W = im.shape[:2]
        x1, y1, x2, y2 = int((x - w / 2) * W), int((y - h / 2) * H), int((x + w / 2) * W), int((y + h / 2) * H)
        pad = int(0.1 * max(x2 - x1, y2 - y1))
        X1, Y1, X2, Y2 = max(0, x1 - pad), max(0, y1 - pad), min(W, x2 + pad), min(H, y2 + pad)
        crop = im[Y1:Y2, X1:X2]
        rect = (x1 - X1, y1 - Y1, x2 - x1, y2 - y1)
        if rect[2] < 20 or rect[3] < 20:
            continue
        mask = np.zeros(crop.shape[:2], np.uint8)
        try:
            cv2.grabCut(crop, mask, rect, np.zeros((1, 65)), np.zeros((1, 65)), 5, cv2.GC_INIT_WITH_RECT)
        except cv2.error:
            continue
        m = np.where((mask == 1) | (mask == 3), 255, 0).astype(np.uint8)
        frac = m.mean() / 255
        if not 0.15 < frac < 0.9:      # 분리 실패(거의 비었거나 박스 전체)는 버림
            continue
        ys, xs = np.nonzero(m)
        crop, m = crop[ys.min():ys.max() + 1, xs.min():xs.max() + 1], m[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
        cuts.append(np.dstack([crop, m]))
    return cuts


def procedural_bg(rng):
    """무늬 배경: 줄무늬, 체크, 노이즈 질감, 그라디언트를 섞는다."""
    h, w = CANVAS_H, CANVAS_W
    kind = rng.choice(["stripes", "checker", "noise", "gradient"])
    c1 = np.array([rng.randint(0, 255) for _ in range(3)], np.float32)
    c2 = np.array([rng.randint(0, 255) for _ in range(3)], np.float32)
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    if kind == "stripes":
        ang = rng.uniform(0, math.pi)
        period = rng.uniform(6, 80)
        t = (np.sin((xx * math.cos(ang) + yy * math.sin(ang)) * 2 * math.pi / period) > 0).astype(np.float32)
    elif kind == "checker":
        s = rng.randint(8, 80)
        t = (((xx // s) + (yy // s)) % 2).astype(np.float32)
    elif kind == "noise":
        s = rng.choice([4, 8, 16, 32])
        t = cv2.resize(np.random.rand(h // s + 1, w // s + 1).astype(np.float32), (w, h), interpolation=cv2.INTER_CUBIC)
    else:
        t = xx / w if rng.random() < 0.5 else yy / h
    t = np.clip(t, 0, 1)[..., None]
    img = c1 * (1 - t) + c2 * t
    img += np.random.normal(0, rng.uniform(2, 15), img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def fit_canvas(img):
    """배경을 1280x720으로 맞춘다(비율 유지 후 중앙 크롭). 원본 좌표 변환 함수도 반환."""
    h, w = img.shape[:2]
    s = max(CANVAS_W / w, CANVAS_H / h)
    img = cv2.resize(img, (math.ceil(w * s), math.ceil(h * s)))
    ox, oy = (img.shape[1] - CANVAS_W) // 2, (img.shape[0] - CANVAS_H) // 2
    img = img[oy:oy + CANVAS_H, ox:ox + CANVAS_W]

    def tf(b):
        x, y, bw, bh = b
        x1, y1 = (x - bw / 2) * w * s - ox, (y - bh / 2) * h * s - oy
        x2, y2 = (x + bw / 2) * w * s - ox, (y + bh / 2) * h * s - oy
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(CANVAS_W, x2), min(CANVAS_H, y2)
        return (x1, y1, x2, y2) if x2 - x1 > 2 and y2 - y1 > 2 else None
    return img, tf


def transform_cut(cut, target_w, rng):
    if rng.random() < 0.5:
        cut = cut[:, ::-1]
    s = target_w / cut.shape[1]
    cut = cv2.resize(cut, (max(3, round(cut.shape[1] * s)), max(3, round(cut.shape[0] * s))), interpolation=cv2.INTER_AREA)
    ang = rng.uniform(-30, 30)
    h, w = cut.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), ang, 1.0)
    nw = int(h * abs(M[0, 1]) + w * abs(M[0, 0])) + 2
    nh = int(h * abs(M[0, 0]) + w * abs(M[0, 1])) + 2
    M[0, 2] += nw / 2 - w / 2
    M[1, 2] += nh / 2 - h / 2
    cut = cv2.warpAffine(cut, M, (nw, nh), flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0, 0))
    rgb = cut[..., :3].astype(np.float32) * rng.uniform(0.4, 1.2)   # 조명 차이
    a = cut[..., 3].astype(np.float32) / 255
    if rng.random() < 0.5:   # 좌우 이동 모션 블러
        k = rng.randint(2, max(3, target_w // 6))
        ker = np.zeros((1, k), np.float32)
        ker[:] = 1 / k
        rgb = cv2.filter2D(rgb * a[..., None], -1, ker)
        a = cv2.filter2D(a, -1, ker)
        rgb = rgb / np.maximum(a[..., None], 1e-3)
    elif target_w < 40 or rng.random() < 0.3:
        rgb = cv2.GaussianBlur(rgb, (3, 3), 0)
    a = cv2.GaussianBlur(a, (3, 3), 0)
    return np.clip(rgb, 0, 255), a


def iou_any(box, boxes):
    x1, y1, x2, y2 = box
    for b in boxes:
        iw = min(x2, b[2]) - max(x1, b[0])
        ih = min(y2, b[3]) - max(y1, b[1])
        if iw > 0 and ih > 0:
            return True
    return False


def make_one(cuts, real_imgs, bgs, args, rng):
    r = rng.random()
    if r < 0.35 and real_imgs:
        p = rng.choice(real_imgs)
        img, tf = fit_canvas(cv2.imread(str(p)))
        boxes = [b for b in (tf(l) for l in read_labels(p)) if b]
    elif r < 0.7 and bgs:
        img, _ = fit_canvas(cv2.imread(str(rng.choice(bgs))))
        boxes = []
    else:
        img = procedural_bg(rng)
        if bgs and rng.random() < 0.5:   # 사진 위에 무늬를 반투명으로 섞어 질감 추가
            photo, _ = fit_canvas(cv2.imread(str(rng.choice(bgs))))
            img = cv2.addWeighted(img, rng.uniform(0.3, 0.7), photo, 0.5, 0)
        boxes = []
    img = img.astype(np.float32)

    for _ in range(rng.choice([1, 1, 1, 2, 3])):
        tw = math.exp(rng.uniform(math.log(args.min_w), math.log(args.max_w)))
        rgb, a = transform_cut(rng.choice(cuts), tw, rng)
        h, w = a.shape
        if w >= CANVAS_W or h >= CANVAS_H:
            continue
        for _try in range(10):
            x, y = rng.randint(0, CANVAS_W - w), rng.randint(0, CANVAS_H - h)
            ys, xs = np.nonzero(a > 0.3)
            if len(xs) == 0:
                break
            box = (x + xs.min(), y + ys.min(), x + xs.max() + 1, y + ys.max() + 1)
            if not iou_any(box, boxes):
                break
        else:
            continue
        if len(xs) == 0:
            continue
        if rng.random() < 0.5:   # 도르래 줄: 표적 위쪽에서 좌우 화면 끝까지 가는 얇은 선
            cy = int(box[1] + rng.uniform(0, 0.3) * (box[3] - box[1]))
            col = [rng.randint(150, 255)] * 3 if rng.random() < 0.7 else [rng.randint(0, 80)] * 3
            ly = cy + rng.randint(-40, 40)
            ry = cy + rng.randint(-40, 40)
            mid = (box[0] + box[2]) // 2
            cv2.line(img, (0, ly), (mid, cy), col, 1, cv2.LINE_AA)
            cv2.line(img, (mid, cy), (CANVAS_W - 1, ry), col, 1, cv2.LINE_AA)
        roi = img[y:y + h, x:x + w]
        roi[:] = roi * (1 - a[..., None]) + rgb * a[..., None]
        boxes.append(box)

    if rng.random() < 0.5:   # 어두운 환경
        g = rng.uniform(1.5, 3.0)
        img = 255 * (img / 255) ** g * rng.uniform(0.5, 1.0)
        img += np.random.normal(0, rng.uniform(2, 8), img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    return img, boxes


def generate(split, n, cuts, real_imgs, bgs, out_dir, args, rng):
    img_dir = out_dir / "images" / split
    lbl_dir = out_dir / "labels" / split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)
    widths = []
    for i in range(n):
        img, boxes = make_one(cuts, real_imgs, bgs, args, rng)
        name = f"syn_{split}_{i:05d}"
        cv2.imwrite(str(img_dir / f"{name}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, rng.randint(70, 95)])
        lines = []
        for x1, y1, x2, y2 in boxes:
            lines.append(f"0 {(x1 + x2) / 2 / CANVAS_W:.6f} {(y1 + y2) / 2 / CANVAS_H:.6f} "
                         f"{(x2 - x1) / CANVAS_W:.6f} {(y2 - y1) / CANVAS_H:.6f}")
            widths.append(x2 - x1)
        (lbl_dir / f"{name}.txt").write_text("\n".join(lines))
    w = np.array(widths)
    print(f"{split}: {n}장, 표적 {len(w)}개, 폭(px) 중앙값 {np.median(w):.0f}, 50px 이하 {np.mean(w <= 50) * 100:.0f}%")


def main():
    args = parse_args()
    rng = random.Random(args.seed)
    np.random.seed(args.seed)
    src, out_dir = Path(args.src), Path(args.out_dir)
    if out_dir.exists():
        shutil.rmtree(out_dir)
    bgs = sorted(p for p in Path(args.bg_dir).rglob("*") if p.suffix.lower() in IMG_EXTS) if args.bg_dir else []
    rng.shuffle(bgs)
    n_bg_val = len(bgs) // 5
    for split, n, split_bgs in [("train", args.n_train, bgs[n_bg_val:]), ("val", args.n_val, bgs[:n_bg_val])]:
        split_dir = src / "images" / split
        cuts = extract_cutouts(split_dir, rng)
        real_imgs = sorted(split_dir.iterdir())
        print(f"{split}: 표적 조각 {len(cuts)}개, 배경 사진 {len(split_bgs)}장")
        generate(split, n, cuts, real_imgs, split_bgs, out_dir, args, rng)


if __name__ == "__main__":
    main()
