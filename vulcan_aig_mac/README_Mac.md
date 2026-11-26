# vulcan_aig_mac — AI-G 발칸 묶음 (Mac 노트북용)

`vulcan_aig_bundle_i2c_1126`(우분투 노트북 기준)을 **Mac 노트북에서 쓰도록** 옮긴 것.
보드(TOPST AI-G)가 카메라→NPU로 표적을 찾고 **I2C로 아두이노 우노**에 서보·사격 명령을 보내는
구조는 동일하다. 보드에서 도는 코드(`board_app/` 등)는 그대로이고, **노트북 쪽 준비·실행만 Mac용**으로
바뀐다. 보드 안의 실행/배선/종료 절차는 원본 `README_실행.md`를 그대로 따른다.

> 표적 전환이 추가됨: 보드 실행 시 `AIG_TARGET`로 **회색 비행기 / 풍선**을 고른다 (아래 4번).
> 보드 NPU는 `.pt`가 아니라 변환된 모델 폴더(`*_s16_quantized`)를 쓰므로, 풍선은 먼저 변환해야 한다(5번).

---

## 1. 보드/우노 쪽은 원본과 동일
`board_app/`, `tc-nn-app/`, `npu_include/`, `*_s16_quantized/`, `setup.sh`, `arduino/`는
원본 묶음과 같다. 보드에 올리고 빌드·실행하는 절차(`setup.sh`, `run_i2c.sh`)는 `README_실행.md`
5~8번을 그대로 따른다. 아래는 **노트북이 Mac일 때 달라지는 부분만** 정리한다.

## 2. Mac 노트북 준비 (처음 한 번)

### 2-1. 패키지 (Homebrew)
```bash
brew install nmap picocom arduino-cli
python3 -m venv ~/venvs/aig && ~/venvs/aig/bin/pip install opencv-python numpy
```

### 2-2. 보드에 IP 주기 (우분투의 nmcli 공유 대신)
Mac은 **시스템 설정 > 일반 > 공유 > 인터넷 공유**를 쓴다. 공유 출처는 Wi‑Fi, 공유 대상은
USB‑이더넷 어댑터(랜선)로 체크하면 Mac이 `192.168.2.1`이 되고 보드는 `192.168.2.x`를 받는다.
보드 IP 찾기:
```bash
arp -a | grep -i "192.168.2"        # 또는: nmap -sn 192.168.2.0/24
ssh topst@192.168.2.<IP>            # 비밀번호 topst
```
> 원본 README의 `10.42.0.x`는 우분투 nmcli 공유 대역이다. Mac 인터넷 공유는 보통 `192.168.2.x`.
> `run_i2c.sh`/`view_aig.py`에 넣는 IP만 이 대역으로 바꾸면 나머지는 동일하다.

### 2-3. 아두이노 펌웨어 업로드 (Mac 포트는 /dev/cu.*)
```bash
arduino-cli core update-index && arduino-cli core install arduino:avr && arduino-cli lib install Servo
ls /dev/cu.usbserial* /dev/cu.usbmodem*          # 우노 포트 확인
arduino-cli compile --fqbn arduino:avr:uno arduino/model_control_i2c_v2
arduino-cli upload -p /dev/cu.usbserial-XXXX --fqbn arduino:avr:uno arduino/model_control_i2c_v2
```
`sudo chmod 666 ...`는 Mac에선 필요 없다. 연결 확인: `picocom -b 115200 /dev/cu.usbserial-XXXX`.

### 2-4. 묶음을 보드로 보내기
```bash
scp -r . topst@192.168.2.<IP>:~/vulcan_aig_mac
ssh topst@192.168.2.<IP> 'cd ~/vulcan_aig_mac && bash setup.sh'
```

## 3. 실행 (매번) — 원본 6번과 동일, IP/포트만 Mac 대역
- 보드 ssh 창에서: `cd ~/vulcan_aig_mac && AIG_TARGET=airplane bash run_i2c.sh`
- 화면 보기 (Mac 새 터미널): `~/venvs/aig/bin/python pc_viewer/view_aig.py --board 192.168.2.<IP> --port 5001`

## 4. 표적 전환 (회색 비행기 ↔ 풍선)
보드 실행 시 환경변수로 고른다 (기본 airplane):
```bash
AIG_TARGET=airplane bash run_i2c.sh     # 회색 비행기 (vulcan_s16_quantized)
AIG_TARGET=balloon  bash run_i2c.sh     # 풍선       (balloon_s16_quantized)
```
`balloon_s16_quantized/`가 없으면 5번으로 먼저 만든다.

## 5. 풍선 NPU 변환 (balloon.pt → balloon_s16_quantized)
보드 NPU는 `.pt`를 못 돌리므로, `balloon_code/balloon.pt`를 Telechips EnlightSDK로
양자화·컴파일해야 한다. 툴킷은 **x86-64 리눅스 전용**이라 Docker 컨테이너에서 돌린다.
(Apple Silicon Mac은 `PLATFORM=linux/amd64` 에뮬레이션 — 느리고 공식 지원은 아님. x86 리눅스나
보드가 있으면 그쪽이 빠르고 안전하다.)

```bash
# (1) pt -> NPU용 ONNX (Mac에서 바로 됨)
python3 tools/export_onnx_npu.py --weights ../balloon_code/balloon.pt \
    --imgsz 640 --out work/balloon_extracted.onnx

# (2) 캘리브레이션 이미지 모으기 (풍선 실제 장면 수백 장이 좋음)
python3 tools/make_calib_set.py --dataset-root ../dataset --out work/calib_npu --count 300
#   (풍선 데이터가 dataset/에 없으면 balloon_code가 쓴 Matterport 이미지 등을 work/calib_npu/에 직접 복사)

# (3) Docker(x86-64 Ubuntu 20.04) 안에서 변환
PLATFORM=linux/amd64 tools/docker/run_sdk.sh ../tc-nn-toolkit work
#   컨테이너 안에서:
#     cd /work && NET_NAME=balloon ./convert_npu.sh balloon_extracted.onnx calib_npu 1
#   결과: work/npu_build/.. 와 work/balloon_quantized/ (net.so, npu_cmd.bin, quantized_network.bin)

# (4) 결과를 모델 폴더 이름으로 두고 보드에 올린다
mv work/balloon_quantized balloon_s16_quantized
scp -r balloon_s16_quantized topst@192.168.2.<IP>:~/vulcan_aig_mac/
```
그 뒤 `AIG_TARGET=balloon bash run_i2c.sh`로 풍선 표적 실행.

> 변환 상세·주의(mean/std, 격자 파일, letterbox, output-order)는 `tools/convert_npu.sh` 주석 참고.
> `tc-nn-toolkit/`(EnlightSDK)·`npu_include/`·`tc-nn-app/`는 Telechips 비공개 소스라 **공개 저장소에
> 올리지 않는다**(이 폴더의 `.gitignore`가 제외).
