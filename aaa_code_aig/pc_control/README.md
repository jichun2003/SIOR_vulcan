# pc_control — PC 제어형 (보드는 검출만)

```
[AI-G 보드]  카메라 -> NPU(yolov8s int8) -> 박스
                                 |
                          TCP NDJSON (기본 5000)
                                 v
[PC]  aig_link.py -> aaa_code_1111의 추적/PID/사격 통제 그대로 -> pyserial -> 아두이노
```

검증이 끝난 `aaa_code_1111`의 제어 코드(`tracking_filter.py`, `turret_control.py`,
`track_target.py`, `fire_control.py`, `serial_link.py`)를 **복사하지 않고 import해서 쓴다.**
1111을 고치면 여기도 같이 바뀐다.

## 파일

- `aig_link.py` — 보드의 검출 스트림 수신 (백그라운드 스레드, 끊기면 자동 재접속).
  1111 `vision.py`의 Camera+Detector 자리를 대신하고, `pick_target()`도 같은 규칙으로 제공.
- `main_aig.py` — PC 쪽 진입점. 1111 `main.py`의 2dof 루프와 같은 구조.

## 실행

보드:
```sh
./aig_turret --network /home/root/yolov8s_target_quantized --mode detect-only --json-port 5000
```

PC (보드 IP는 `board_check.sh`의 `ip a` 결과):
```sh
python main_aig.py --board 192.168.0.100 --port COM3 --latency 0.08 --latency-tilt 0.08
python main_aig.py --board 192.168.0.100 --no-serial            # 아두이노 없이 로직만
python main_aig.py --board 192.168.0.100 --no-serial --log-csv logs/aig.csv
```

필요 패키지: `pyserial`, `numpy` (1111의 `track_target.py`가 numpy를 씀). opencv/ultralytics는
필요 없다 — PC가 추론을 하지 않기 때문.

신호는 터미널 입력: `shot`(1번=선도각, 2번=사격) / `cancel` / `r` / `q`

## 보드가 보내는 줄 형식

한 줄에 JSON 한 개(NDJSON). 좌표는 보드 카메라의 원본 프레임 픽셀 기준.

```json
{"t":1234.567,"infer_ms":15.49,"frame_w":1280,"frame_h":720,
 "dets":[{"cls":0,"conf":0.8700,"x1":612.0,"y1":343.1,"x2":692.0,"y2":376.9}],"best":0}
```

Telechips의 Vision Protocol(RTPM, 포트 9998/9999)을 쓰지 않은 이유: 메시지 프레이밍이
공개 문서에 없고 `libvisionprotocol` 바이너리에 묶여 있다. 우리가 보드 앱을 직접 쓰는
이상 표준 socket만으로 읽히는 NDJSON이 더 단순하고 확인하기 쉽다. RTPM GUI로 눈으로
보고 싶을 때는 기본 앱(`tcnnapp -i camera -o rtpm`)을 그대로 쓰면 된다.

## 주의

- **지연 L**: `--latency`는 "보드 추론 + 네트워크 전송 + PC 처리 + 서보 반응" 전체다.
  PC 경유라서 보드 단독보다 크다. `--sweep`(보드) 또는 1111의 `--latency-sweep`과
  `analyze_log.py --latency`로 실측해서 넣어야 한다. 틀리면 과보상으로 오차가 몇 배 커진다.
  `--latency`만 주면 tilt는 `config.py` 값이 남으므로 `--latency-tilt`도 같이 주는 게 안전하다.
- **통신 끊김과 표적 상실을 구분한다**: 프레임이 안 오는 동안에는 제어를 하지 않는다.
  끊김을 "표적 상실"로 처리하면 포가 멋대로 coast 한다.
- 화각(`--hfov/--vfov`)은 **보드에 달린 카메라** 기준이다. 1111의 64.0/34.2는 기존 USB
  웹캠 실측값이라, 카메라가 바뀌면 다시 재야 한다.

## 검증

보드 대신 스텁 보드 앱(`../board_app`, `-DAIG_STUB_NPU`)을 127.0.0.1에 띄우고 연결해
확인했다: 5초간 133프레임 수신, α-β 속도 추정 ±77 deg/s, CSV 160줄 기록, 서보 명령각이
가짜 표적의 왕복을 따라감.
