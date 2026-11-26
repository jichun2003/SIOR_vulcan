# 발칸 터렛 AI-G 실행 묶음 — I2C 버전 (2026-11-26)

AI-G 보드가 카메라 → NPU로 표적을 찾고, **I2C로 아두이노 우노에 서보·사격 명령**을 보내는 전체 묶음.
다른 우분투 노트북에서 이 zip 하나로 처음부터 실행할 수 있게 정리함.

> 주의: `tc-nn-app/`, `npu_include/`에는 Telechips SDK 소스·헤더가 들어 있음. 팀 내부용으로만 쓰고 공개 저장소에 올리지 않음.

---

## 1. 들어 있는 것

| 경로 | 어디서 씀 | 내용 |
|---|---|---|
| `board_app/` | 보드 | 보드 앱 소스 (`aig_turret`). `uart.c/.h`는 **I2C 송신 추가본** |
| `tc-nn-app/`, `npu_include/` | 보드 | 빌드에 필요한 카메라 소스·NPU 헤더 |
| `vulcan_s16_quantized/` | 보드 | 표적 모델 (YOLOv8s int8, 960 입력) |
| `setup.sh` | 보드 | 빌드 (필요한 패키지 설치 포함, i2c-tools 포함) |
| `run_i2c.sh` | 보드 | **실행 (우노 I2C 연결 + PC 미리보기)** |
| `run.sh`, `conf_watch.sh` | 보드 | 인지부만 실행 (우노 없이) |
| `arduino/model_control_i2c_v2/` | 우노 | **우노 펌웨어** (I2C 주소 0x08 수신, `d` = 서보 힘 풀기, USB 115200) |
| `arduino/i2c_test/` | 우노 | I2C 연결 확인용 스케치 (받은 바이트를 USB로 출력) |
| `pc_viewer/` | 노트북 | 보드 카메라 화면 + 탐지 박스 보기 (`view_aig.py`) |
| `pc_tools/` | 노트북 | `find_spikes.py`(로그에서 명령각 급변 찾기), `grab_frames.py`, `eval_csi_int8.py` |
| `docs/` | — | 트러블슈팅 기록, 1124 변경사항, 1124 실행 설명 |

---

## 2. 하드웨어 연결 요약

| 연결 | 핀 |
|---|---|
| 보드 → 우노 I2C | 40핀 **3번(SDA) → 우노 A4**, **5번(SCL) → A5**, **9번(GND) → GND** |
| 우노 전원 (하나만) | 노트북 USB **또는** 40핀 **4번(5V) → 우노 5V** |
| USB-TTL (부팅용) | USB-TTL **TX → 보드 디버그 RX**, **GND → GND** (TX·VCC 핀은 연결 안 함) |
| 서보 | pan D3, tilt D9, 전원은 7.4V LiPo 별도 (GND 공통) |
| 사격 모터 | MOSFET 게이트 D5, 별도 배터리 |

- 보드 디버그 헤더의 **TX는 고장**이라 쓰지 않음. USB-TTL은 콘솔용이 아니라 **보드 RX를 HIGH로 잡아 부팅이 멈추지 않게** 하는 용도.
- tilt 서보는 반드시 **일반 180° 서보**. 360° 연속 회전 서보를 쓰면 위아래가 폭주함 (docs 12번 참고).

---

## 3. 노트북 준비 (새 우분투 컴퓨터에서 처음 한 번)

### 3-1. 패키지
```
sudo apt update && sudo apt install -y picocom nmap unzip python3-venv curl
```

### 3-2. 랜선 공유 설정 (보드에 10.42.0.x 주소 주기)
```
nmcli device
```
유선 장치 이름(예: `enp3s0`, `eno1`)을 확인하고:
```
sudo nmcli connection add type ethernet ifname <유선장치이름> con-name aig-share ipv4.method shared
sudo nmcli connection up aig-share
```
이후 이 노트북은 `10.42.0.1`, 보드는 `10.42.0.x`를 받음.

### 3-3. 아두이노 업로드 도구 (arduino-cli)
```
mkdir -p ~/.local/bin && curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin sh
export PATH=$PATH:$HOME/.local/bin
arduino-cli core update-index && arduino-cli core install arduino:avr && arduino-cli lib install Servo
sudo usermod -aG dialout $USER     # 다시 로그인하면 sudo chmod 없이 포트 사용 가능
```

### 3-4. 화면 보기용 파이썬 환경
```
python3 -m venv ~/venvs/aig && ~/venvs/aig/bin/pip install opencv-python numpy
```

### 3-5. 이 묶음 풀기
```
cd ~/Downloads && unzip -o vulcan_aig_bundle_i2c.zip
```
→ `~/Downloads/vulcan_aig_bundle_i2c/`

---

## 4. 우노 펌웨어 업로드 (펌웨어가 바뀌었을 때만)

- 우노를 보드와 **완전히 분리**하고 노트북 USB에만 꽂음 (D0/D1에 아무것도 연결하지 않음).
```
ls /dev/ttyUSB* /dev/ttyACM*
```
- 나온 포트(호환 우노는 보통 `/dev/ttyUSB0`, 정품은 `/dev/ttyACM0`)로 업로드:
```
export PATH=$PATH:$HOME/.local/bin
sudo chmod 666 /dev/ttyUSB0
arduino-cli compile --fqbn arduino:avr:uno ~/Downloads/vulcan_aig_bundle_i2c/arduino/model_control_i2c_v2
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno ~/Downloads/vulcan_aig_bundle_i2c/arduino/model_control_i2c_v2
```
- 확인: `sudo picocom -b 115200 /dev/ttyUSB0` → `[link] AI-G cmd input = I2C addr 0x08 ...`가 뜨면 성공 (종료 Ctrl+A → Ctrl+X).
- 업로드 실패(`not in sync`): picocom이 열려 있는지, D0/D1에 선이 있는지 확인.

---

## 5. 보드에 설치 (보드가 바뀌었거나 처음일 때 한 번)

1. 보드를 부팅하고 IP를 찾음 (6-1, 6-2 참고).
2. 노트북에서 묶음을 보드로 보냄:
```
cd ~/Downloads && scp vulcan_aig_bundle_i2c.zip topst@10.42.0.<IP>:~/
```
3. 보드에서 풀고 빌드 (sudo 비밀번호 `topst`, 1~2분):
```
ssh topst@10.42.0.<IP>
python3 -m zipfile -e vulcan_aig_bundle_i2c.zip ~/
cd ~/vulcan_aig_bundle_i2c && bash setup.sh
```
- `npu test: Success!`와 마지막 `완료`가 나오면 성공.
- `/dev/video2 없음` 경고: 전원을 끄고 CSI 케이블을 다시 꽂고 부팅.

---

## 6. 실행 순서 (매번)

### 6-1. 보드 부팅 (우노는 연결하지 않은 상태)
- 노트북 ↔ 보드 **랜선**, USB-TTL **TX → 보드 RX**, **GND → GND**, USB-TTL은 노트북에 꽂음.
- 보드 전원을 넣고 **2분** 기다림.

### 6-2. 접속
```
sudo nmap -sn 10.42.0.0/24
ssh topst@10.42.0.<IP>
```
- `10.42.0.1`이 아닌 주소가 보드. 재부팅할 때마다 바뀔 수 있음.

### 6-3. 우노 연결 (보드 부팅 후에만)
1. **사격 모터 배터리 분리** (사격 시험 전까지).
2. 40핀 3 → A4, 5 → A5, 9 → GND.
3. 우노 전원 (둘 중 하나만):
   - 노트북 USB: `ls /dev/ttyUSB*; sudo picocom -b 115200 /dev/ttyUSB1` → 받은 명령이 `[i2c] s2 90`처럼 보임
   - 40핀 4번(5V) → 우노 5V: 노트북 없이 단독
4. 서보 LiPo 켜기.
5. (선택) 연결 확인: 보드에서 `sudo i2cdetect -y 1` → `08`이 보이면 정상.

### 6-4. 실행 (보드 ssh 창)
```
cd ~/vulcan_aig_bundle_i2c && bash run_i2c.sh
```
- 시작 로그에 `[uart] I2C 연결됨 (/dev/i2c-1, 0x08)`.
- 실행 중 입력: `shot`(1번째 = 선도각 장입, 2번째 = 사격) / `cancel` / `r`(중앙 복귀) / `q`(종료).
- 옵션 (뒤에 붙임):
  - `--pan-sign -1`, `--tilt-sign -1`: 포탑이 표적 반대로 갈 때
  - `--conf 0.35`: 탐지 문턱 (기본 0.3)
  - `--jump-reject-frames 3`: 점프 필터 대기 (기본 10프레임)
  - `--latency <초>`: 지연 L (기본 0.12, `--sweep`으로 실측 권장)
- 카메라가 3초 끊기면(exit 3) 자동 재시작 (최대 5회, `RESTART=0`이면 끔).

### 6-5. 화면 보기 (노트북 새 터미널)
```
~/venvs/aig/bin/python ~/Downloads/vulcan_aig_bundle_i2c/pc_viewer/view_aig.py --board 10.42.0.<IP> --port 5001
```
- 초록 박스 = 추적 중인 표적, 위쪽에 pan/tilt 각도와 사격 상태. 창에서 `q`로 닫음.

### 6-6. 종료
1. 보드 창에서 `q` (또는 Ctrl+C) → 우노에 `x`(사격 정지) + `d`(서보 힘 풀기) 자동 전송.
2. 화면 창 `q`, picocom Ctrl+A → Ctrl+X.
3. 서보 LiPo 끄기 → **우노 전원 분리** → USB-TTL 분리.
4. `sudo shutdown -h now` → 20초 뒤 보드 어댑터 분리.

---

## 7. 지켜야 할 규칙
- 우노는 **보드 부팅 후에만** 켜고, 끌 때는 **우노를 먼저** 끔. (우노를 연결한 채 보드를 켜면 부팅이 안 된 적 있음)
- 우노 USB와 40핀 4번 5V를 **동시에 연결하지 않음**.
- 보드를 켤 때 USB-TTL(TX→RX, GND)을 연결해 둠. 없으면 부팅이 부트로더에서 멈출 수 있음.
- 사격 시험 전에 **사격 배터리만 연결했을 때 모터가 돌지 않는지** 먼저 확인.

---

## 8. 안 될 때

| 증상 | 조치 |
|---|---|
| nmap에 보드가 안 잡힘 | USB-TTL(TX→RX, GND)을 연결한 상태로 보드 전원을 **껐다 켬**. 노트북 랜 공유(3-2) 확인 |
| `[i2c] 쓰기 실패` 반복 | 우노 전원, 3/5/9번 배선 확인. `sudo i2cdetect -y 1`에 `08`이 보여야 함 |
| `I2C 주소 설정 실패` / `/dev/i2c-1 열기 실패` | `ls /dev/i2c-*` 확인. 다른 보드라면 40핀 I2C 버스 번호가 다를 수 있음 → `i2cdetect`로 우노가 보이는 버스를 찾아 `run_i2c.sh`의 `/dev/i2c-1`을 바꿈 |
| `CameraSetConfig 실패` | 이전 실행이 카메라를 잡고 있음 → `sudo pkill -9 aig_turret` |
| 카메라 끊김(exit 3) 반복 | CSI 케이블 고정·여유, 발열(팬) 확인 |
| 포탑이 표적 반대로 감 | `--pan-sign -1` / `--tilt-sign -1` |
| 따라가다 끝까지 치솟거나 꽂힘 | tilt 서보가 360°인지 확인 (시리얼 `s3 70`에 20°만 가고 멈춰야 정상) |
| 강제 종료 후 서보 힘이 남음 | `sudo i2ctransfer -y 1 w2@0x08 0x64 0x0a` |
| 로그로 명령/모터 문제 구분 | 보드 `perception.csv`를 노트북으로 가져와 `python3 pc_tools/find_spikes.py perception.csv` |

자세한 경위는 `docs/AI-G_Arduino_UART_트러블슈팅_2026-11-24.md` 참고.
