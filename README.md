
## 파일 구조
SIOR Project/
  vision/
    requirements.txt       # ultralytics, opencv-python, pyserial, matplotlib, scipy
    pid.py                 # 안티 와인드업 포함 PID 클래스
    serial_link.py          # 아두이노 시리얼 송신 (+ --no-serial용 더미)
    track_target.py         # 메인: YOLO 탐지 → 2축 PID → 서보 각도 전송 (+ 리드샷 모드)
    lead_estimator.py       # bbox 크기 기반 단안 거리 추정 + BallisticModel 연결
    analyze_log.py          # 거리/크기별 실험 로그(csv) 비교 플롯
    evaluate_on_images.py   # 학습과 동일한 imgsz로 이미지 폴더 일괄 탐지 검증
  firmware/turret_control/
    turret_control.ino      # 아두이노: 서보2(D3)=pan, 서보3(D9)=tilt, 서보1(D2)=피더 + DC모터2개(L298N), 슬루레이트 제한
  ballistic/
    ballistic_model_fit.py  # 발사 영상에서 v0(초속)/Cd(항력계수) 역산 → json 저장
    lead_solver.py           # v0/Cd로 발사각·선도각(리드샷)·비행시간 계산 (BallisticModel)
  ardoino/
    javascript               # 실제 배선 기준 Tinkercad 테스트 스케치 원본 (서보3+DC모터2 통합)
    Neat Bigery-Trug.png      # Tinkercad 회로도 (서보 전원/DC모터 전원 분리 배선 확인용)

## 실제 하드웨어 배선 (2026-09-23 현장 확인)
서보 3개는 s번호가 기능이 아니라 핀 순서(D2,D3,D9)로 고정되어 있고, 실제 물리적 역할은:
- 서보1 (D2, 본체에서 "공을 밀어주는 부분") = 피더/푸셔 — 발사 매커니즘, 조준 축 아님. 2단계 실험에서는 사용 안 함
- 서보2 (D3, "본체 맨 아래") = **pan** (포탑 베이스 회전)
- 서보3 (D9, "총구 근처 우측") = **tilt** (포신 고저각)
- DC모터 2개(L298N: ENA=5/IN1=7/IN2=8, ENB=6/IN3=11/IN4=12) = 플라이휠 발사 모터 — 2단계 실험에서는 사용 안 함
- 서보/DC모터 전원이 각각 별도 AA 배터리팩으로 분리되어 있음 (아두이노 5V 직결 아님)
- 시리얼 9600 baud, 텍스트 명령(`s1 90`=피더, `s2 45`=pan, `s3 120`=tilt, `c`=전체중앙, DC모터 `f/b/l/r/x`, `v150`)
- `track_target.py`/`serial_link.py`는 매 프레임 `s2 <pan>`, `s3 <tilt>`만 보냄

## 리드샷(선도각) 모드
아직 거리 센서가 없어서, 표적의 실제 폭(--target-width-m)을 안다고 가정하고 bbox
폭 + 카메라 화각으로 사거리를 역산하는 단안 추정을 쓴다. 정밀 사격용은 아니고
ballistic/lead_solver.py의 리드샷 계산 로직을 미리 연결/검증해두는 용도.

사용 순서:
1. 발사 영상으로 v0/Cd 캘리브레이션 (영상 있을 때):
   `python ballistic/ballistic_model_fit.py --video gel_shot.mp4 --mass-kg 0.00025 --diameter-m 0.0075 --launch-angle-deg 20 --meters-per-pixel 0.005`
   → `ballistic_model.json` 생성
2. 추적에 연결:
   `python vision/track_target.py --no-serial --ballistic-json ../ballistic/ballistic_model.json --target-width-m 0.03`
   → 화면 중앙 정렬 대신, 현재 포탑 각도+표적 위치/속도로 계산한 리드샷 각도를 목표로 서보를 움직임 (정지 표적이면 자동으로 기존 중앙 정렬과 동일하게 수렴)

## 동작방식
track_target.py가 카메라 프레임마다 checkpoint/best.pt로 추론 → 신뢰도가 가장 높은 박스(단일 표적 가정)를 선택
--target-class 기본값은 "target"

표적 중심과 프레임 중심의 픽셀 오차를 카메라 화각(--hfov/--vfov) 기준 각도 오차로 변환

pan/tilt 각각 별도 PID로 오차를 보정해 서보 각도를 갱신(프레임당 최대 ±8도로 제한해 오버슈트/진동 억제)

"s2 <pan>\n", "s3 <tilt>\n" 형식으로 아두이노에 전송 → 아두이노는 슬루레이트 제한(±1도/15ms)으로 한 번 더 부드럽게 움직여 기계적 진동을 억제

표적을 15프레임 이상 놓치면 PID를 리셋하고 현재 각도 유지(재포착 시 튀는 것 방지)


## 실행코드
cd vision
pip install -r requirements.txt

# 아두이노 없이 비전/PID 로직만 먼저 확인
python3 track_target.py --model ../checkpoint/best.pt --no-serial --camera 0

python3 track_target.py --model ../checkpoint/best.pt --no-serial --camera 0 --conf 0.05


# 실제 포탑 연결 후
python track_target.py --model ../checkpoint/best.pt --port /dev/tty.usbmodemXXXX

# 원거리/근거리 등 조건별로 로그를 남겨 비교
python track_target.py --model ../checkpoint/best.pt --no-serial --log-csv logs/near.csv
python analyze_log.py --log near=logs/near.csv --log far=logs/far.csv



