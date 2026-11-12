# pc_control — PC 제어형 (보드는 검출만)

```
[AI-G 보드]  카메라 -> NPU(yolov8s int8) -> 박스
                                 |
                          TCP NDJSON (기본 5000)
                                 v
[PC]  aig_link.py -> aaa_code_1111의 추적/PID/사격 통제 그대로 -> pyserial -> 아두이노
```

`aaa_code_aig`만 있으면 돌아가도록 `aaa_code_1111`의 제어 코드를 이 폴더로 복사해왔다.

## 파일

### 새로 만든 것

- `aig_link.py` — 보드의 검출 스트림 수신 (백그라운드 스레드, 끊기면 자동 재접속).
  1111 `vision.py`의 Camera+Detector 자리를 대신하고, `pick_target()`도 같은 규칙으로 제공.
- `main_aig.py` — PC 쪽 진입점. 1111 `main.py`의 2dof 루프와 같은 구조.
- `view_aig.py` — 보드 카메라 화면 실시간 보기 (보드 `--preview-port`). 탐지 박스 + 서보각 +
  사격 상태 오버레이, `s` 키로 원본 프레임 저장(재학습용). 제어와 무관해서 아무 때나 켜고 끌 수 있다.

### 1111에서 가져온 코드

| 파일 | 1111과 비교 | 바뀐 내용 |
|---|---|---|
| `tracking_filter.py` | **그대로** (바이트 동일) | — |
| `turret_control.py` | **그대로** | — |
| `track_target.py` | **그대로** | — |
| `fire_control.py` | **그대로** | — |
| `serial_link.py` | **그대로** | — |
| `analyze_log.py` | **그대로** | — |
| `ballistic/` 3개 | **그대로** | `lead_solver.py`, `ballistic_model_fit.py`, `ballistic_model.json` |
| `config.py` | **수정** | 아래 참고 |

`config.py`에서 바뀐 것:

1. **삭제**: `model_path`, `camera_index`, `frame_width`, `frame_height`, `conf_threshold`,
   `device`, `no_show` — 탐지를 보드가 하므로 PC는 모델도 카메라도 쓰지 않는다. 해상도는
   보드가 매 프레임 알려주고, 신뢰도 임계값은 보드 실행 옵션(`--conf`)이다.
   (복사해온 다른 모듈 중 이 항목들을 쓰는 코드는 없다 — 확인 후 삭제)
2. **추가**: `board_host`, `board_port` — 검출 결과를 받아올 보드 주소.
3. **변경**: `latency_s` 0.25 → **0.12**, `latency_tilt_s` 0.265 → **0.12**.
   1111 값은 Windows + CPU 추론 실측값이고, NPU는 추론이 ~15ms라 L이 훨씬 작다.
   0.12도 **실측값이 아니라 어림값**이라 반드시 재측정해야 한다 (아래 "주의" 참고).

1111에서 **가져오지 않은 것**: `vision.py`(PC가 추론하지 않음), `main.py`(→ `main_aig.py`),
`evaluate_on_images.py`·`capture_frames.py`·`test_serial.py`·`train/`(학습·점검용, AI-G와 무관).

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

필요 패키지: `pyserial`, `numpy` (`track_target.py`가 numpy를 씀). 선도각을 쓸 때만 `scipy`,
로그 플롯에 `matplotlib`. opencv/ultralytics/torch는 필요 없다 — PC가 추론을 하지 않는다.
(`../requirements.txt`)

신호는 터미널 입력: `shot`(1번=선도각, 2번=사격) / `cancel` / `r` / `q`

### 보드 카메라 화면 보기

보드 실행 명령에 `--preview-port 5001`을 붙이고 (standalone/detect-only 둘 다 됨):
```sh
python3 view_aig.py --board 192.168.0.100              # 기본 포트 5001
python3 view_aig.py --board 192.168.0.100 --zoom 2     # 창 크게
```
`main_aig.py`와 동시에 띄워도 된다 (포트가 다름). 이건 opencv-python이 필요하다.

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
  (`test_control` 시뮬레이션: 실제 L이 0.08s인데 0.25s를 넣으면 추종 오차 RMS가
  2.05도 → 14.65도로 7배가 된다)
- **보드 단독 모드와 설정이 두 군데 있다**: `config.py`(PC 제어형)와
  `../board_app/aig_config.h`(보드 단독형)에 같은 값들이 따로 들어있다. 한쪽만 고치면
  두 모드가 다르게 동작하므로 같이 고칠 것.
- **통신 끊김과 표적 상실을 구분한다**: 프레임이 안 오는 동안에는 제어를 하지 않는다.
  끊김을 "표적 상실"로 처리하면 포가 멋대로 coast 한다.
- 화각(`--hfov/--vfov`)은 **보드에 달린 카메라** 기준이다. 1111의 64.0/34.2는 기존 USB
  웹캠 실측값이라, 카메라가 바뀌면 다시 재야 한다.

## 검증

보드 대신 스텁 보드 앱(`../board_app`, `-DAIG_STUB_NPU`)을 127.0.0.1에 띄우고 연결해
확인했다: 5초간 133프레임 수신, α-β 속도 추정 ±77 deg/s, CSV 160줄 기록, 서보 명령각이
가짜 표적의 왕복을 따라감.
