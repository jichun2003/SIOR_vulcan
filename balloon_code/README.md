# balloon_code — 풍선 표적 버전 (AIG 보드 없음, PC 전용)

표적을 **풍선**으로 바꾼 자율추적 대공포 코드. `aaa_code_1111`(기존 표적 버전)을 그대로
복사해, 탐지 가중치(`balloon.pt`)와 탐지 클래스(`balloon`)만 풍선용으로 바꾼 자립형 폴더다.
추적 제어(2축 PID / 2dof)·시리얼·사격 로직은 모두 동일하며, AIG NPU 보드 없이 Mac/Windows
PC에서 카메라로 바로 돌린다.

## 폴더 구성
- `main.py` — 진입점 (카메라 → YOLO 풍선 탐지 → 2축 추적 → 사격 신호)
- `config.py` — 기본 파라미터 (`model_path=balloon.pt`, `target_class=balloon`)
- `vision.py` / `track_target.py` / `tracking_filter.py` / `turret_control.py` / `fire_control.py` / `serial_link.py`
- `train/train_balloon.py` — **첫 풍선 가중치 만들기** (Roboflow Universe 데이터셋 → yolov8s 파인튜닝)
- `train/retrain.py` — **못 잡는 풍선 재학습** (캡처 → Roboflow 라벨링 → 재학습)
- `train/compare_models.py` — 두 가중치를 같은 카메라로 나란히 비교
- `balloon.pt` — 풍선 학습 가중치 (학습하면 생성됨, 기본 가중치)

## 0. 설치
```bash
cd balloon_code
pip install -r requirements.txt     # ultralytics, opencv, roboflow 등
```

## 1. 첫 풍선 가중치 (`balloon.pt`) — 이미 포함됨
`balloon.pt`는 **이미 학습해 폴더에 넣어뒀다**. 바로 2단계로 가서 실행하면 된다.
(공개 Matterport 풍선 데이터셋 train 61/val 13장으로 yolov8s 파인튜닝,
 val mAP50 0.84 / mAP50-95 0.67 / P 0.85 / R 0.78, 클래스명 `balloon`)

> 왜 직접 학습? 바로 받아 쓸 "풍선 전용" 공개 `.pt`가 사실상 없다. 기본 YOLOv8은 COCO라
> balloon 클래스가 없고, Roboflow Universe엔 풍선 **데이터셋**은 많지만 학습된 가중치는
> 내려받을 수 없다. 그래서 데이터셋을 받아 한 번 파인튜닝한 것.

다시 만들거나 다른 데이터셋으로 바꾸고 싶을 때만:
```bash
# (A) API 키 불필요 — 지금 balloon.pt를 만든 방식 (Matterport 자동 다운로드+변환)
python3 train/prepare_matterport.py --out dataset_matterport
python3 train/train_balloon.py --data dataset_matterport/data.yaml --model ../yolov8s.pt

# (B) Roboflow Universe 데이터셋으로 (키 필요, 보통 더 큼/다양함)
export ROBOFLOW_API_KEY=xxxxxxxx          # Windows: set ROBOFLOW_API_KEY=xxxxxxxx
python3 train/train_balloon.py            # 기본 데이터셋 다운로드 + 학습 -> balloon.pt
#   다른 데이터셋: 그 페이지의 rf.workspace("WS").project("PROJECT").version(N) 값을
#   --ws/--project/--version 으로 전달
```

## 2. 실행 (풍선 추적)
```bash
# 아두이노 없이 비전/추적만 확인 (먼저 이걸로 풍선이 잡히는지 본다)
python3 main.py --no-serial

# 포탑 연결해서 추적
python3 main.py --port /dev/cu.usbmodem1101        # Windows: --port COM3
```
화면 키: `q`=종료, `r`=리셋, **`s`=현재 프레임 저장**(`captures/`), `x`=사격취소, `스페이스`=사격신호.

## 3. 못 잡는 풍선 → 재학습 (실험 계획)
처음엔 실제 풍선(색·조명·크기)이 다 잡히지 않을 수 있다. 그럴 때:
1. 실험 중 **잘 못 잡히는 장면에서 `s` 키**를 눌러 프레임을 저장한다.
   → `captures/`에 저장되며, 풍선을 놓친 프레임은 파일명이 `..._miss.jpg`.
2. `captures/`의 (특히 `miss`) 이미지를 Roboflow 프로젝트에 올려 **풍선으로 라벨링**한다.
3. COCO 형식으로 export 해 `coco/` 폴더(또는 원하는 곳)에 모은 뒤 재학습:
   ```bash
   python3 train/retrain.py --coco-dir coco
   # (풍선 없는 배경 영상이 있으면 오탐 억제: --negatives-video empty_room.mp4)
   ```
   → `balloon.pt`가 갱신되고, `balloon_MMDD_retrain.pt` 백업이 남는다.
4. `train/compare_models.py`로 이전/신규 가중치를 같은 카메라로 비교해 개선을 확인한다.

> 탐지 클래스 이름은 코드 전체에서 **`balloon`** 으로 통일돼 있다. 데이터셋 클래스 이름이
> 다르더라도 `train_balloon.py`가 단일 클래스 `balloon`으로 맞춰주고, `retrain.py`는
> `--class-name balloon`(기본값)으로 통합한다.
