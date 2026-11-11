# -*- coding: utf-8 -*-
"""
EnlightSDK quantizer(int8) 캘리브레이션용 이미지 폴더를 만든다.

NPU 툴킷은 float 네트워크를 int8로 양자화할 때 활성값 분포를 보려고 실제 이미지를
쓴다 (converter의 --dataset Custom --dataset-root, --enable-track). 학습 데이터
전체를 넣을 필요는 없고, "실제 운용 때 들어올 영상"을 대표하는 수백 장이면 된다.

중요: 캘리브레이션 이미지가 실제 장면과 다르면 int8 정확도가 떨어진다. 그래서
합성 데이터(synthetic)보다 live_capture 쪽을 우선해서 뽑는다.

사용 예:
    python tools/make_calib_set.py --out ../dataset/calib_npu --count 300
"""

import argparse
import random
import shutil
from pathlib import Path

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
# 앞쪽이 우선순위 높음 (실제 카메라로 찍은 것 우선)
PREFERRED_ORDER = ["live_capture", "live_labeled", "extra_1105", "yolo_retrain", "synthetic"]


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--dataset-root", default="../dataset", help="원본 이미지들이 있는 상위 폴더")
    p.add_argument("--out", default="../dataset/calib_npu", help="캘리브레이션 이미지 복사 위치")
    p.add_argument("--count", type=int, default=300, help="뽑을 이미지 수")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--clean", action="store_true", help="출력 폴더를 먼저 비운다")
    return p.parse_args()


def collect(root: Path):
    """하위 폴더를 PREFERRED_ORDER 우선순위로 묶어서 이미지 목록을 반환."""
    groups = {}
    for p in root.rglob("*"):
        if p.suffix.lower() not in IMG_EXTS or not p.is_file():
            continue
        try:
            top = p.relative_to(root).parts[0]
        except ValueError:
            continue
        groups.setdefault(top, []).append(p)

    ordered = []
    for name in PREFERRED_ORDER:
        if name in groups:
            ordered.append((name, sorted(groups.pop(name))))
    for name in sorted(groups):  # 목록에 없던 폴더는 뒤에
        ordered.append((name, sorted(groups[name])))
    return ordered


def main():
    args = parse_args()
    root = Path(args.dataset_root).resolve()
    out = Path(args.out).resolve()
    if not root.is_dir():
        raise SystemExit(f"폴더가 없습니다: {root}")
    if out.is_relative_to(root) and out.exists() and args.clean:
        pass  # --clean은 아래에서 처리

    groups = collect(root)
    groups = [(n, fs) for n, fs in groups if out not in fs[0].parents]
    if not groups:
        raise SystemExit(f"이미지를 찾지 못했습니다: {root}")

    print(f"[calib] 원본 {root}")
    for name, files in groups:
        print(f"[calib]   {name:14s} {len(files)}장")

    # 폴더별 할당량: 실제 카메라 영상에 가중치를 더 주되 한 폴더에만 몰리지 않게 섞는다
    # (표적 거리/크기 다양성이 캘리브레이션 품질을 좌우하므로 합성·재학습 셋도 일부 포함)
    rng = random.Random(args.seed)
    weights = {"live_capture": 0.40, "live_labeled": 0.25, "extra_1105": 0.15,
               "yolo_retrain": 0.10, "synthetic": 0.10}
    present = [(n, fs) for n, fs in groups]
    wsum = sum(weights.get(n, 0.05) for n, _ in present)
    quota = {n: max(1, int(round(args.count * weights.get(n, 0.05) / wsum))) for n, _ in present}

    picked = []
    for name, files in present:  # 1차: 할당량만큼 등간격 추출
        need = min(quota[name], len(files), max(0, args.count - len(picked)))
        if need <= 0:
            continue
        step = len(files) / need
        take = [files[int(i * step)] for i in range(need)]
        picked += take
        print(f"[calib]   -> {name}에서 {len(take)}장 선택")
    for name, files in present:  # 2차: 반올림/부족분을 남은 파일로 채움
        if len(picked) >= args.count:
            break
        rest = [f for f in files if f not in set(picked)]
        take = rest[: args.count - len(picked)]
        if take:
            picked += take
            print(f"[calib]   -> {name}에서 {len(take)}장 추가")
    rng.shuffle(picked)

    if args.clean and out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)
    for i, src in enumerate(picked):
        shutil.copy2(src, out / f"calib_{i:04d}{src.suffix.lower()}")

    print(f"[calib] 완료: {len(picked)}장 -> {out}")
    print(f"[calib] converter 인자: --dataset Custom --dataset-root {out}")


if __name__ == "__main__":
    main()
