# -*- coding: utf-8 -*-
"""
API 키 없이 쓰는 공개 풍선 데이터셋 준비 스크립트.

Matterport 'balloon' 데이터셋(VIA 폴리곤 주석, train 61 / val 13장)을 받아
YOLOv8 형식(단일 클래스 'balloon')으로 변환하고 data.yaml을 만든다.
변환된 data.yaml을 train_balloon.py --data 로 넘기면 바로 파인튜닝할 수 있다.

출처: https://github.com/matterport/Mask_RCNN/releases/download/v2.1/balloon_dataset.zip
(Flickr CC 이미지 기반. 수업/연구 데모용 시작 데이터셋으로 사용)

사용 예 (balloon_code 폴더에서):
    python3 train/prepare_matterport.py --out dataset_matterport
    python3 train/train_balloon.py --data dataset_matterport/data.yaml
"""

import argparse
import io
import json
import urllib.request
import zipfile
from pathlib import Path

import cv2

URL = "https://github.com/matterport/Mask_RCNN/releases/download/v2.1/balloon_dataset.zip"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src", default=None,
                   help="이미 받아 푼 balloon/ 폴더 경로. 생략하면 인터넷에서 자동 다운로드")
    p.add_argument("--out", default="dataset_matterport", help="YOLO 데이터셋 출력 폴더")
    p.add_argument("--class-name", default="balloon")
    return p.parse_args()


def ensure_src(src, workdir):
    if src:
        return Path(src)
    print(f"[download] {URL}")
    with urllib.request.urlopen(URL) as r:  # noqa: S310 (신뢰된 GitHub 릴리스 URL)
        data = r.read()
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        z.extractall(workdir)
    return workdir / "balloon"


def via_to_yolo(split_dir, out_img_dir, out_lbl_dir):
    """VIA json 하나(폴더당 via_region_data.json)를 읽어 폴리곤->bbox->YOLO 라벨로 변환."""
    out_img_dir.mkdir(parents=True, exist_ok=True)
    out_lbl_dir.mkdir(parents=True, exist_ok=True)
    jsons = list(split_dir.glob("*.json"))
    if not jsons:
        raise SystemExit(f"VIA json을 찾을 수 없습니다: {split_dir}")
    via = json.loads(jsons[0].read_text(encoding="utf-8"))

    n_img = n_box = 0
    for entry in via.values():
        fname = entry["filename"]
        img_path = split_dir / fname
        if not img_path.exists():
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        lines = []
        for region in entry.get("regions", {}).values():
            sa = region["shape_attributes"]
            xs, ys = sa.get("all_points_x"), sa.get("all_points_y")
            if not xs or not ys:
                continue
            x1, x2 = max(min(xs), 0), min(max(xs), w)
            y1, y2 = max(min(ys), 0), min(max(ys), h)
            if x2 <= x1 or y2 <= y1:
                continue
            xc = (x1 + x2) / 2.0 / w
            yc = (y1 + y2) / 2.0 / h
            bw = (x2 - x1) / w
            bh = (y2 - y1) / h
            lines.append(f"0 {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")
            n_box += 1
        # 이미지/라벨 복사 (라벨이 비면 배경 네거티브로 빈 파일)
        cv2.imwrite(str(out_img_dir / fname), img)
        (out_lbl_dir / (Path(fname).stem + ".txt")).write_text("\n".join(lines), encoding="utf-8")
        n_img += 1
    return n_img, n_box


def main():
    args = parse_args()
    out = Path(args.out).resolve()
    src = ensure_src(args.src, out / "_download")

    totals = {}
    for split in ("train", "val"):
        ni, nb = via_to_yolo(src / split, out / "images" / split, out / "labels" / split)
        totals[split] = (ni, nb)
        print(f"[{split}] 이미지 {ni}장, bbox {nb}개 변환")

    data_yaml = out / "data.yaml"
    data_yaml.write_text(
        f"path: {out.as_posix()}\ntrain: images/train\nval: images/val\n"
        f"nc: 1\nnames:\n  0: {args.class_name}\n",
        encoding="utf-8",
    )
    print(f"\ndata.yaml -> {data_yaml}")
    print(f"다음: python3 train/train_balloon.py --data {data_yaml}")


if __name__ == "__main__":
    main()
