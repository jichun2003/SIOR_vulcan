# -*- coding: utf-8 -*-
"""
Roboflow Universe의 공개 '풍선' 데이터셋을 받아 yolov8s를 파인튜닝해
balloon_code/balloon.pt (풍선 추적용 첫 가중치)를 만든다.

왜 학습이 필요한가:
  바로 받아 쓸 수 있는 '풍선 전용' 공개 .pt 가 사실상 없다.
  - 기본 YOLOv8(yolov8n/s.pt)은 COCO 학습이라 balloon 클래스가 없고
  - Roboflow Universe엔 풍선 데이터셋은 많지만 '학습된 가중치'는 내려받을 수 없어
    데이터셋을 받아 직접 학습해야 한다. (이 프로젝트의 기존 재학습 흐름과 동일)

준비:
  1) pip install -r requirements.txt   (roboflow, ultralytics 포함)
  2) https://app.roboflow.com 무료 가입 후 Settings > API Key 복사
  3) 쓸 풍선 데이터셋을 Roboflow Universe(universe.roboflow.com)에서 고른 뒤,
     데이터셋 페이지 > Download Dataset > "show download code"에 나오는
       rf.workspace("WS").project("PROJECT").version(N)
     값을 --ws / --project / --version 으로 넘긴다. (기본값은 아래 DEFAULT_*)

사용 예 (balloon_code 폴더에서):
    export ROBOFLOW_API_KEY=xxxxxxxx          # Windows: set ROBOFLOW_API_KEY=xxxxxxxx
    python3 train/train_balloon.py            # 기본 데이터셋 다운로드 + 학습
    python3 train/train_balloon.py --ws elbrus-bootcamp --project balloon-wirjw --version 8 --epochs 80

API 키 없이 쓰려면 (Universe에서 YOLOv8 형식 zip을 수동으로 받아 압축 해제한 경우):
    python3 train/train_balloon.py --data /경로/풍선데이터셋/data.yaml

NVIDIA GPU가 있으면 CUDA 버전 PyTorch가 설치돼 있어야 GPU로 학습한다 (pip 기본 torch는 CPU 전용):
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
"""

import argparse
import os
import sys
from datetime import date
from pathlib import Path

import torch
import yaml
from ultralytics import YOLO

BALLOON_DIR = Path(__file__).resolve().parents[1]   # balloon_code/
ROOT = Path(__file__).resolve().parents[2]           # 프로젝트 루트 (runs/ 저장용)

# Roboflow Universe 기본 풍선 데이터셋. 접속이 안 되거나 404면 Universe에서 아무 'balloon'
# 데이터셋이나 골라 그 페이지의 다운로드 코드 값으로 --ws/--project/--version 을 바꾸면 된다.
DEFAULT_WS = "elbrus-bootcamp"
DEFAULT_PROJECT = "balloon-wirjw"
DEFAULT_VERSION = 8


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--api-key", default=os.environ.get("ROBOFLOW_API_KEY"),
                   help="Roboflow API 키 (없으면 환경변수 ROBOFLOW_API_KEY 사용)")
    p.add_argument("--ws", default=DEFAULT_WS, help="Roboflow workspace")
    p.add_argument("--project", default=DEFAULT_PROJECT, help="Roboflow project")
    p.add_argument("--version", type=int, default=DEFAULT_VERSION, help="Roboflow dataset version")
    p.add_argument("--data", default=None,
                   help="다운로드 대신 이미 받아둔 YOLOv8 data.yaml 경로를 직접 지정 (API 키 불필요)")
    p.add_argument("--class-name", default="balloon",
                   help="최종 단일 클래스 이름. main.py의 --target-class와 반드시 같아야 한다")
    p.add_argument("--model", default="yolov8s.pt", help="파인튜닝 베이스 가중치")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--epochs", type=int, default=80)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--device",
                   default="0" if torch.cuda.is_available() else ("mps" if torch.backends.mps.is_available() else "cpu"),
                   help="CUDA GPU 번호(0), mps(Mac), cpu. 기본: NVIDIA -> Mac GPU -> cpu")
    p.add_argument("--workers", type=int, default=4, help="데이터 로더 수 (Windows에서 멈추면 0)")
    p.add_argument("--download-only", action="store_true", help="데이터셋 다운로드만 하고 학습은 생략")
    return p.parse_args()


def download_dataset(args):
    """Roboflow Universe에서 YOLOv8 형식 데이터셋을 받아 data.yaml 경로를 반환."""
    try:
        from roboflow import Roboflow
    except ImportError:
        sys.exit("roboflow 패키지가 없습니다. `pip install roboflow` 후 다시 실행하세요.")
    if not args.api_key:
        sys.exit(
            "Roboflow API 키가 없습니다. 환경변수 ROBOFLOW_API_KEY를 설정하거나 --api-key로 넘기세요.\n"
            "(https://app.roboflow.com > Settings > API Key)\n"
            "키 없이 쓰려면 Universe에서 YOLOv8 zip을 수동으로 받아 --data 로 data.yaml을 지정하세요."
        )
    out = BALLOON_DIR / "dataset_roboflow"
    rf = Roboflow(api_key=args.api_key)
    project = rf.workspace(args.ws).project(args.project)
    print(f"[roboflow] {args.ws}/{args.project} v{args.version} 다운로드 -> {out}")
    dataset = project.version(args.version).download("yolov8", location=str(out), overwrite=True)
    return Path(dataset.location) / "data.yaml"


def force_single_class(data_yaml_path, class_name):
    """다운로드한 데이터셋을 단일 클래스(class_name)로 강제 통일한다.

    Universe 풍선 데이터셋은 클래스 이름이 'Balloon', 'balloons' 등 제각각이거나
    여러 클래스를 섞기도 한다. main.py는 target_class 이름으로 필터링하므로,
    모든 라벨의 클래스 인덱스를 0으로 바꾸고 data.yaml의 names를 [class_name] 하나로 맞춘다.
    """
    data_yaml_path = Path(data_yaml_path)
    data = yaml.safe_load(data_yaml_path.read_text(encoding="utf-8"))

    base = data_yaml_path.parent
    # data.yaml의 train/val 경로(상대/절대 모두 가능)에서 label 폴더를 찾아 첫 열을 0으로
    changed = 0
    for split_key in ("train", "val", "test"):
        split = data.get(split_key)
        if not split:
            continue
        img_dir = (base / split).resolve() if not Path(split).is_absolute() else Path(split)
        # yolov8 구조: .../images/... <-> .../labels/...
        label_dir = Path(str(img_dir).replace("images", "labels"))
        if not label_dir.exists():
            continue
        for txt in label_dir.rglob("*.txt"):
            lines = txt.read_text(encoding="utf-8").splitlines()
            new = []
            for ln in lines:
                parts = ln.split()
                if len(parts) >= 5:
                    parts[0] = "0"
                    new.append(" ".join(parts))
            if new != lines:
                changed += 1
            txt.write_text("\n".join(new) + ("\n" if new else ""), encoding="utf-8")

    data["nc"] = 1
    data["names"] = [class_name]
    data_yaml_path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"[single-class] 라벨 {changed}개 파일을 클래스 0으로, names=['{class_name}'] 로 통일")


def main():
    args = parse_args()

    data_yaml = Path(args.data) if args.data else download_dataset(args)
    if not data_yaml.exists():
        sys.exit(f"data.yaml을 찾을 수 없습니다: {data_yaml}")

    force_single_class(data_yaml, args.class_name)

    if args.download_only:
        print(f"다운로드/정리 완료: {data_yaml}")
        return

    model = YOLO(args.model)
    model.train(
        data=str(data_yaml),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        patience=30,
        # 실제 풍선은 색/조명/크기 변화가 커서 색·스케일 증강을 강하게 (retrain.py와 동일 기조)
        hsv_h=0.02, hsv_s=0.7, hsv_v=0.5,
        degrees=5.0, translate=0.1, scale=0.6,
        fliplr=0.5, mosaic=1.0, mixup=0.1, close_mosaic=15,
        project=str(ROOT / "runs"), name=f"balloon_{date.today():%m%d}",
    )

    best = Path(model.trainer.best)
    import shutil
    dst = BALLOON_DIR / "balloon.pt"
    shutil.copy2(best, dst)
    print(f"\n최종 가중치 -> {dst}")
    print("이제 balloon_code 폴더에서 `python3 main.py --no-serial` 로 풍선 추적을 테스트하세요.")


if __name__ == "__main__":
    main()
