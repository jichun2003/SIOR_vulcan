# aaa_code_aig — TOPST AI-G 포팅

`aaa_code_1111`(Windows + NVIDIA CUDA + COM 포트 기준 최종 운용 코드)을 TOPST AI-G
싱글보드컴퓨터로 옮긴 폴더. **이 폴더만 있으면 돌아간다** — 필요한 제어 코드는
1111에서 복사해왔고, 무엇이 어떻게 바뀌었는지는 `pc_control/README.md`의
"1111에서 가져온 코드" 표에 있다.

```
aaa_code_aig/
  board_app/            보드에서 돌아가는 C 앱 (--mode standalone | detect-only)
    *.c /*.h            1111 제어 로직의 C 포팅 + NPU/카메라/UART
    stub/               카메라·NPU 가짜 API (PC에서 배관 확인용)
    test_control.c      제어 로직 닫힌 루프 시뮬레이션
  pc_control/           PC 제어형 (보드는 검출만)
    aig_link.py         보드의 NDJSON 수신 (1111 vision.py 자리)
    main_aig.py         PC 진입점 (1111 main.py 자리)
    config.py 외        1111에서 복사한 추적/PID/사격/로그 모듈
    ballistic/          1111에서 복사 (선도각 계산)
  tools/                모델 변환 (.pt -> NPU) + 보드 조사 스크립트
  firmware/             아두이노 펌웨어 (1111에서 복사, 수정 없음)
  requirements.txt
  BOARD_CHECK.md        보드 실측 체크리스트
```

## 1. 먼저 알아야 할 제약

AI-G 공식 문서(`topst-development/Documentation`, Single Board Computer / G Model / AI-G)를
확인한 결과, **기존 파이썬 코드를 보드에 그대로 올리는 경로는 없다.**

| 항목 | 현재 1111 코드 | AI-G 보드 |
|---|---|---|
| 추론 | ultralytics(파이썬) + CUDA | NPU (TCC750x). **C API만 존재** (`libtcndnpu`, `npu_api.h`) |
| 모델 | `.pt` 런타임 로드 | PC에서 `.onnx` → int8 변환 → `net.so` + `npu_cmd.bin` + `quantized_network.bin` |
| OS | Windows | Yocto 4.0 Kirkstone (`telechips-topst-ai-image`), aarch64 |
| 파이썬 | 3.x + torch/ultralytics/opencv | 문서에 보드용 파이썬/pip/opencv 언급 없음 (호스트 빌드 환경 얘기만 있음) |
| 카메라 | `cv2.VideoCapture(0, CAP_DSHOW)` | `/dev/video0`~`/dev/video2`, 샘플은 `camera_api.c` (`CameraOpenDevice` 등) |
| 시리얼 | `COM3` (pyserial) | 리눅스 tty 디바이스 (termios), 보드-아두이노 UART |

핵심: **NPU 추론 자체는 C로 가야 한다.** 보드에 파이썬 NPU 바인딩이 없고, int8 양자화도
PC 쪽 툴킷(EnlightSDK)을 거쳐야 한다.

### 단, 파이썬 연결 경로는 공식적으로 있다 (Vision Protocol)

보드는 탐지 결과를 **JSON으로 TCP** 로 PC에 보낼 수 있고, PC 쪽 수신 앱(RTPM)이 바로
**파이썬 3.10** 애플리케이션이다. 즉 "보드=NPU 추론 / PC=파이썬 제어" 구성이
SDK가 원래 지원하는 경로다.

- 포트: 스트림 **9998**, 메시지 **9999** (`tc-nn-app/src/NnRtpm.h`의 `RTPM_STEAM_PORT`/`RTPM_MESSAGE_PORT`)
- 보드 실행: `tcnnapp -i camera -o rtpm -n <네트워크폴더>`
- 메시지: `VISION_MSG_EVENT_RESULT_DATA_JSON`, 페이로드는 COCO 스타일 JSON
  (`NnRtpm.c: RtpmSendResultDataAsJson`)

```json
{ "info": [syncStamp],
  "cluster1": { "type": 2,
                "od": [ { "id": 0, "category_id": 0, "score": 0.84,
                          "bbox": [x, y, w, h], "desc": "target", "distance": 0.0 } ] },
  "cluster2": { ... } }
```
`bbox`는 COCO식 `[x, y, width, height]`(좌상단+크기, 정수 반올림)이고, 좌표는 RTPM 출력
프레임 크기로 스케일된 값이다. 1111의 `pick_target()`이 쓰는 `cx/cy/w/h`로 바로 바꿀 수 있다.

- PC용 `libvisionprotocol`은 **빌드된 바이너리가 공개 저장소에 들어있다**
  (`visionprotocol/build/linux-x86_64/libvisionprotocol.so`,
  `visionprotocol/build/windows-x86_64/libvisionprotocol.dll`) → 파이썬에서 ctypes로 쓸 수 있다.
  API는 `Vision_API_Connection` / `Vision_API_SendMessage` / `Vision_API_RecvMessage`
  (`include/vision_api.h`, 핸들에 `char ip[16]; uint16_t port;`).

이 경로를 쓰면 검증이 끝난 제어 코드(`tracking_filter.py`, `turret_control.py`,
`track_target.py`, `fire_control.py`, `serial_link.py`)를 **그대로 파이썬으로 유지**하고,
`vision.py`의 Camera+Detector만 "Vision Protocol 수신 클라이언트"로 바꾸면 된다.

반대로 좋은 소식: `tcnnapp`의 **기본 네트워크 1번이 `/usr/share/yolov8s_quantized/`** 이고
모델 주 성능표에 yolov8s 640×640 / **15.49 ms (약 64 FPS)** / mAP@50 0.576이 나와 있다.
우리 모델(`best1105_live.pt`)도 **yolov8s(11.1M 파라미터, 클래스 1개 `target`)** 이라
모델 계열을 바꾸지 않고 그대로 간다.

## 2. 지금까지 된 것 (검증 완료)

### 모델 → NPU용 ONNX 추출
`tools/export_onnx_npu.py`

ultralytics 기본 ONNX export는 Detect 헤드의 DFL 디코드까지 그래프에 넣어서
NPU 미지원 연산(Softmax/Div/Sub 등)이 섞인다. 그래서 Telechips 모델 주의
`*_extracted.onnx`와 같은 방식으로 **헤드의 conv 출력까지만** 내보내고, DFL 디코드 +
sigmoid + NMS는 보드 CPU 후처리로 넘긴다.

```
python tools/export_onnx_npu.py --weights ../checkpoint/best1105_live.pt \
    --imgsz 640 --out yolov8s_target_extracted.onnx \
    --verify --verify-image ../dataset/live_capture/target_1105_021739/0071.jpg
```

출력 6개 (전부 conv 출력):

```
box_s8  (1,64,80,80)   cls_s8  (1,1,80,80)
box_s16 (1,64,40,40)   cls_s16 (1,1,40,40)
box_s32 (1,64,20,20)   cls_s32 (1,1,20,20)
```
`box` 64채널 = 4변(l,t,r,b) × reg_max 16, `cls` = sigmoid 전 raw logit.

중간에 막혔던 것 2개와 해결:
- C2f의 `chunk()`가 ONNX에 `Shape/Gather/Div`를 만든다 → NPU 지원 목록에 없음.
  ultralytics 공식 export와 동일하게 `C2f.forward_split`(정적 `Split`)으로 바꿔 해결.
- `onnxsim`으로 접으려 했으나 onnxsim 0.4.36 + onnx 1.19는 macOS에서 segfault.
  위 `forward_split` 처리로 애초에 필요가 없어져서 기본은 끄고 `--simplify`로 선택.

**결과: 그래프 연산이 `Add, Concat, Constant, Conv, MaxPool, Mul, Resize, Sigmoid, Split`
뿐이고 전부 NPU toolkit 문서 표 5.1(지원 연산) 안에 있다.**

검증 (`--verify`):
1. torch 원본 vs ONNX raw 출력 → 최대 오차 `1.0e-04`
2. raw 출력을 numpy로 DFL 디코드 + NMS 한 결과 vs ultralytics 정식 후처리
   → 이미지 60장(검출 72개) 전수 비교에서 **개수 불일치 0장, 박스 최대 오차 0.0001 px,
   점수 최대 오차 1.5e-06**

→ `decode_raw_outputs()` / `nms()` 가 보드 `post_process.c`의 **기준 구현**이다.
   C로 옮길 때 이 함수와 숫자가 맞는지 비교하면 된다.

### 캘리브레이션 데이터 준비
`tools/make_calib_set.py` — int8 양자화용 이미지 폴더를 만든다. 실제 카메라 영상에
가중치를 더 주되 한 폴더에 몰리지 않게 섞는다 (표적 거리/크기 다양성이 양자화 정확도를 좌우).

```
python tools/make_calib_set.py --out ../dataset/calib_npu --count 300
# live_capture 120 / live_labeled 75 / extra_1105 45 / yolo_retrain 30 / synthetic 30
```

### 변환 파이프라인
`tools/convert_npu.sh` — EnlightSDK 4단계(converter → quantizer → compiler → build_network)
+ 보드 전송까지의 명령을 정리. **AI-G SDK가 깔린 Ubuntu 20.04 호스트에서 실행**한다
(Mac/Windows에서는 안 돌아감).

입력 정규화는 ultralytics와 같게 `x = pixel / 255` → `--mean 0 0 0 --std 255 255 255`.
이 값이 틀리면 양자화 스케일이 어긋나 검출이 통째로 망가진다.

> SDK가 TOPST 전용 배포라 공개 문서에 모든 인자가 없다. 스크립트에 `CHECK`로 표시한
> 인자(`--type`, `--force-output`, 후처리 옵션)는 설치된 SDK의 `--help`로 확인 후 맞출 것.

### 애플리케이션 (양쪽 구조 모두 작성 완료)

보드 실측 결과에 따라 어느 쪽으로든 갈 수 있게 두 구조를 다 만들어두었다.
**한 실행 파일이 `--mode`로 둘 다 지원**하므로 실측 후 옵션만 바꾸면 된다.

| 폴더 | 구조 | 내용 |
|---|---|---|
| [board_app/](board_app/) | 보드 단독 (`--mode standalone`) | 카메라→NPU→α-β 예측→2자유도 PID→UART. 1111의 제어 모듈을 C로 포팅 |
| [board_app/](board_app/) | 검출만 (`--mode detect-only`) | 박스를 TCP NDJSON으로 내보냄 |
| [pc_control/](pc_control/) | PC 제어 | 보드의 JSON을 받아 **1111 파이썬 모듈을 그대로 import해서** 제어 |

1111 → C 포팅 대응표:

| 1111 파이썬 | board_app | 상태 |
|---|---|---|
| `config.py` | `aig_config.h` | 현장 실측값 그대로 (단 지연 L은 재측정 필요) |
| `turret_control.py` RatePID | `pid.c` | 포팅 완료 |
| `turret_control.py` TurretControl | `turret.c` | 포팅 완료 |
| `tracking_filter.py` | `tracking_filter.c` | 포팅 완료 (α-β, 서보각 이력 보간, coast) |
| `vision.py` | `npu_vision.c` | `npu_api`/`camera_api`/`scaler_api` 사용 + `pick_target` |
| `serial_link.py` | `uart.c` | termios + 백그라운드 스레드, 프로토콜 동일 |
| `fire_control.py` | `fire.c` | 2단계 신호 상태머신 (선도각 계산은 미포팅) |
| `main.py` | `main.c` | 메인 루프, 인자, CSV 로그, `--sweep` |
| `track_target.py` | `main.c`의 픽셀→각도 | 단안 거리/3D는 선도각용이라 제외 |

**검증 (PC에서 실제로 돌린 결과)**

`board_app/test_control.c` — 닫힌 루프 시뮬레이션 (표적 사인파 → 지연 → α-β → 2자유도
PID → 슬루 제한 서보):

| 조건 | 결과 |
|---|---|
| 움직이는 표적 ±25도/4초, L 일치(0.08s) | 오차 RMS **2.05도**, 최대 3.07도 |
| 같은 표적, L을 0.25s로 잘못 넣음 | RMS **14.65도** (7배) |
| 정지 표적 | RMS **0.50도** (불감대 0.4도) |
| 표적 상실 | coast 후 서보 각속도 **0.0 deg/s** |

`board_app/stub/` — 카메라/NPU 가짜 API로 전체 앱을 PC에서 실행, CSV 로그와 JSON 출력
경로 확인. 그 스텁 보드 앱에 `pc_control/main_aig.py`를 붙여 **end-to-end도 확인**
(5초간 133프레임 수신, 서보 명령각이 가짜 표적을 따라감).

### 문서엔 없지만 공개 소스에서 확인된 것

문서(`6. NPU API.md`)에는 구조체 정의와 라이브러리 이름이 빠져 있는데, 보드 소스가
전부 공개 저장소에 있어서 직접 확인했다 (`topst-development/libtc-ndnpu`,
`tc-nn-app`, `visionprotocol`, 기본 브랜치는 `main`이 아니라 **`develop`**).

링크 라이브러리: **`-ltcndnpu`** (`libtc-ndnpu/lib/tcndnpu.pc.in`)

탐지 결과 구조체 (`libtc-ndnpu/lib/include/npu_api.h`) — 후처리가 bbox/클래스/점수를
그대로 채워주므로, DFL 디코드와 NMS를 우리가 C로 다시 쓸 필요가 없을 수도 있다:

```c
typedef struct {
    float   x_min, x_max, y_min, y_max;  /* object box position */
    int16_t cls;                         /* object class */
    float   score;                       /* object confidence score */
    int16_t img_w, img_h;                /* input image width/height */
} enlight_obj_t;

typedef struct { int cnt; enlight_obj_t obj[256]; } enlight_objs_t;  /* MAX_DETECT_BOX=256 */
```

그래서 변환 시 `--type obj`(Enlight detector 후처리 사용)가 되면 `post_process.c`를
직접 구현하지 않아도 된다. 반대로 `--type unknown`으로 가면 우리가 후처리를 써야 하고,
그때 `tools/export_onnx_npu.py`의 `decode_raw_outputs()`가 기준 구현이 된다.
**어느 쪽이 되는지는 SDK에서 한 번 돌려봐야 확정된다.**

입력 포맷: `image_fmt_t { IMAGE_FMT_RGBA32, IMAGE_FMT_RGB24 }` (`tc-nn-app/include/NnType.h`),
색 포맷 설정은 `network_set_color_format(net, NPU_COLOR_RGB | NPU_COLOR_YUV)`.
입력 크기는 `network_get_input_width/height()`로 질의.

### 아직 보드에서 확인해야 하는 것

`BOARD_CHECK.md` 참고 (실측 체크리스트). 핵심은:
- 보드에 파이썬/opencv가 있는지
- 아두이노가 `/dev/ttyUSB*`로 잡히는지 (보드 단독 구성의 전제)
- 기존 USB 웹캠이 보드에서 잡히는지 (카메라가 바뀌면 화각 재측정 필요)
- 보드에 gcc가 있어서 직접 빌드가 되는지

## 3. 남은 작업

1. **보드 실측** (`BOARD_CHECK.md`, `tools/board_check.sh`) → 카메라/UART 디바이스 이름 확정
2. **모델 변환** (SDK 호스트 필요): `tools/make_calib_set.py` → `tools/convert_npu.sh`
   → `--type obj`가 되는지 확인 (되면 `post_process.c` 직접 구현 불필요)
3. **int8 정확도 확인**: 보드 검출 결과를 FP32와 같은 이미지로 비교 (원거리 작은 표적부터 깨진다)
4. **지연 L 재측정**: `./aig_turret --sweep` → `analyze_log.py --latency`. 위 표의 2번째 줄이
   이유다. 그 다음 `Kp*L <= 0.75` 기준으로 PID 게인 재조정
5. 카메라가 바뀌면 화각(`--hfov/--vfov`) 재측정
6. (선택) 선도각 계산 포팅, `/dev/overlay` 영상 표시


## 4. 참고 문서

- `Software/1. Linux SDK Getting Started.md` — Yocto 빌드, 시리얼 콘솔 115200, 로그인 topst/topst
- `Software/2. TOPST AI-G NPU toolkit.md` — converter/quantizer/compiler, 지원 연산 표 5.1
- `Software/3. Telechips-Model-Zoo.md` — yolov8n/s/m/l/x 640×640 성능표
- `Software/4. TCNNAPP.md` — 샘플 앱 구조, 옵션, `camera_api`/`display_api`/`scaler_api`
- `Software/6. NPU API.md` — `npu_open`/`network_load_from_file`/`network_run_sync`/`network_run_postprocess`
