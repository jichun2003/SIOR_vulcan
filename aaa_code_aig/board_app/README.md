# board_app — AI-G 보드에서 돌아가는 C 앱

`aaa_code_1111`의 2dof 추적 루프를 TOPST AI-G 보드용 C로 옮긴 것. 두 가지 모드를 한
실행 파일이 지원한다.

```
--mode standalone    (기본) 카메라 -> NPU -> α-β 예측 -> 2자유도 PID -> UART로 아두이노
                     PC 없이 보드 단독 동작
--mode detect-only   검출만 하고 TCP JSON으로 내보냄. 제어는 PC(../pc_control)가 담당
```

## 파일

| 파일 | 1111 대응 | 내용 |
|---|---|---|
| `aig_config.h` | `config.py` | 모든 튜닝 값 (현장 실측값 그대로) |
| `pid.c/.h` | `turret_control.py` RatePID | 속도형 PID (불감대·EMA·안티와인드업) |
| `turret.c/.h` | `turret_control.py` TurretControl | 서보각 상태, 리밋, coast, 상실 처리 |
| `tracking_filter.c/.h` | `tracking_filter.py` | α-β 필터, 서보각 이력 보간, 지연 보상, 2자유도 |
| `npu_vision.c/.h` | `vision.py` | 카메라 → 스케일러 → NPU → `pick_target` |
| `uart.c/.h` | `serial_link.py` | termios UART, 백그라운드 스레드, 같은 프로토콜 |
| `fire.c/.h` | `fire_control.py` | 2단계 신호 상태머신 |
| `json_out.c/.h` | — | detect-only 모드의 TCP NDJSON 서버 |
| `preview_out.c/.h` | — | PC 미리보기 영상 TCP 서버 (`--preview-port`, JPEG 또는 원시 RGB) |
| `main.c` | `main.py` | 인자 파싱, 메인 루프, 로그 |
| `test_control.c` | — | 제어 로직 닫힌 루프 시뮬레이션 (PC에서 실행) |
| `stub/` | — | 카메라/NPU 가짜 API (PC에서 배관 확인용) |

## 빌드

### PC에서 로직만 확인 (보드·SDK 불필요)

```sh
cc -std=c11 -Wall -Wextra -O2 -o test_control \
   test_control.c pid.c turret.c tracking_filter.c fire.c -lm
./test_control

cc -std=c11 -Wall -Wextra -O2 -DAIG_STUB_NPU=1 -Istub -o aig_turret \
   main.c npu_vision.c uart.c json_out.c pid.c turret.c tracking_filter.c fire.c \
   stub/stub.c -lpthread -lm
./aig_turret --no-serial --latency 0.08 --log-csv sim.csv
```

cmake가 있으면 `cmake -DAIG_STUB_NPU=ON .. && make && ctest`도 된다.

### 보드용 빌드

`camera_api.c` / `scaler_api.c`는 라이브러리가 아니라 `tc-nn-app`의 소스라서 같이
컴파일한다. NPU는 `-ltcndnpu`로 링크한다.

```sh
git clone -b develop https://github.com/topst-development/tc-nn-app.git
cmake -DAIG_TCNNAPP_DIR=$PWD/tc-nn-app \
      -DCMAKE_C_COMPILER=aarch64-none-linux-gnu-gcc ..
make
scp aig_turret root@<보드IP>:/home/root/
```

보드에 gcc가 있으면 (board_check.sh 7번 항목) 보드에서 바로 컴파일하는 게 더 빠르다.

## 실행

```sh
# 보드 단독
./aig_turret --network /home/root/yolov8s_target_quantized \
             --camera /dev/video0 --port /dev/ttyUSB0 --latency 0.08

# 검출만 (PC가 제어)
./aig_turret --network /home/root/yolov8s_target_quantized --mode detect-only --json-port 5000

# 지연 L 실측 (정지 표적을 화면에 두고)
./aig_turret --sweep --sweep-axis pan --log-csv /home/root/latency.csv --port /dev/ttyUSB0
# -> CSV를 PC로 가져와서  python analyze_log.py latency.csv --latency
```

신호는 표준입력에 한 줄: `shot`(1번=선도각, 2번=사격) / `cancel` / `r`(리셋) / `q`(종료)

### PC에서 보드 카메라 화면 보기 (`--preview-port`)

두 모드 어디에든 `--preview-port 5001`을 붙이면 보드가 카메라 프레임을 줄여서(기본 1/2,
1280x720 -> 640x360) 탐지 박스·서보각·사격 상태와 함께 TCP로 보낸다. PC에서는
`pc_control/view_aig.py`로 본다 (Mac/Windows, opencv만 필요).

```sh
# 보드
./aig_turret --network /home/root/yolov8s_target_quantized --port /dev/ttyUSB0 --preview-port 5001
# PC
python3 view_aig.py --board <보드IP> --port 5001
```

- 보낼 클라이언트가 없으면 프레임 복사도 하지 않는다 (추적 루프 부담 0)
- 받는 쪽이 느리면 프레임을 건너뛴다 (추적 루프는 절대 기다리지 않음)
- `--preview-fps`(기본 10), `--preview-scale`(기본 2), `--preview-quality`(기본 70)
- CMake가 libjpeg를 찾으면 JPEG(640x360 약 30~50KB), 없으면 원시 RGB(약 690KB)라
  무선이면 `--preview-scale 4 --preview-fps 5` 정도로 낮출 것
- 카메라 버퍼를 RGB888로 가정한다 (`npu_vision.c`가 스케일러에 넘기는 형식과 같다).
  PC 화면에서 빨강/파랑이 바뀌어 보이면 `view_aig.py --swap-rb`(또는 `c` 키),
  화면이 줄무늬/깨짐이면 버퍼가 YUV라는 뜻이니 알려줄 것 (변환 추가 필요)

PC 스텁 빌드로 확인한 것: 박스 위치와 영상 속 표적 위치 일치, 요청 fps 10/15 그대로 수신,
루프 fps보다 높게 요청하면 루프 fps로 상한, detect-only JSON(포트 5000)과 동시 사용 가능.

## 검증 결과 (PC 시뮬레이션)

`test_control`의 닫힌 루프(표적 사인파 → 지연 → α-β → 2자유도 PID → 슬루 제한 서보):

| 조건 | 결과 |
|---|---|
| 움직이는 표적 ±25도/4초, L 일치(0.08s) | 정상상태 오차 RMS **2.05도**, 최대 3.07도 |
| 같은 표적, L을 0.25s로 잘못 넣음 | RMS **14.65도**, 최대 26.2도 |
| 정지 표적 (10도 떨어진 곳) | RMS **0.50도** (불감대 0.4도) |
| 5초에 표적 상실 | coast_max 경과 후 서보 각속도 **0.0 deg/s** (정상 정지) |

**두 번째 줄이 중요하다.** 1111의 `latency_s=0.25`를 그대로 쓰면(= CPU 추론 기준값)
오차가 7배로 커진다. NPU는 추론이 ~15ms라 L이 훨씬 작으므로 `--sweep`으로 반드시
재측정해서 넣어야 한다.

## 포팅하지 않은 것

- **선도각(리드샷) 계산**: `ballistic/lead_solver.py`는 궤적표를 만드는 수치해석이라
  별도 작업이 필요하다. 1111도 `--ballistic-json`이 없으면 선도각 0으로 동작하므로,
  현재는 "신호 2번 → n초 사격" 흐름만 같다.
- **영상 미리보기**: 보드는 `/dev/overlay`(DSI 디스플레이)로 출력한다. 박스를 그려
  띄우려면 `tc-nn-app`의 `display_api` / `opencv_api`를 끌어와야 한다. 지금은
  `--log-csv`와 표준출력으로 확인한다.
- **단안 거리 추정 / 3D 위치**(`track_target.py`의 `estimate_range_m` 등): 선도각을
  쓰지 않으면 필요 없어서 제외했다.

## 보드에서 확인해야 맞춰야 하는 값

`../BOARD_CHECK.md` 실행 결과에 따라 고칠 것:
- `--camera`: `/dev/video0`인지 `/dev/video2`인지
- `--port`: `/dev/ttyUSB0` / `/dev/ttyACM0` / `/dev/ttyAMA*`
- `--fit`: 기본 `stretch`(하드웨어 스케일러가 종횡비를 무시하고 눌러 담음). 원거리
  작은 표적을 놓치면 `--fit crop`(가운데 정사각형)과 비교해볼 것
- `--latency`: 반드시 `--sweep`으로 실측
