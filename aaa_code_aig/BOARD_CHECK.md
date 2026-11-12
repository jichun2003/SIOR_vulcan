# AI-G 보드 실측 체크리스트

포팅 구조를 확정하기 전에 보드에서 직접 확인할 것들. 위에서부터 순서대로 하고,
**굵게 표시한 값은 결과를 적어두면** 그에 맞춰 코드 구조를 정한다.

접속: USB-TTL 시리얼 콘솔 **115200 bps, 흐름제어 None**, 로그인 `topst` / `topst`
(`Software/1. Linux SDK Getting Started.md`). 네트워크가 잡히면 ssh가 더 편하다.

---

## 1. 기본 환경

```sh
uname -a                      # 커널, aarch64 확인
cat /etc/os-release           # Yocto 이미지 버전
free -h                       # 램 용량  <- **기록**
df -h /                       # 남은 저장공간 (eMMC 약 7.3GiB)  <- **기록**
nproc                         # CPU 코어 수 (NnType.h 기준 4개 예상)
```

## 2. 파이썬이 보드에 있는지 (구조 결정에 가장 중요)

```sh
which python3 && python3 -V   # <- **기록** (없으면 "없음"이라고)
python3 -c "import cv2; print(cv2.__version__)"
python3 -c "import numpy; print(numpy.__version__)"
which pip3
```

- **있으면**: 제어 로직 일부를 보드에서 파이썬으로 돌릴 수 있다 (단, NPU는 C API뿐이라
  추론은 여전히 C 앱이 담당. 파이썬은 C 앱의 결과를 받아 쓰는 쪽).
- **없으면**: 보드는 C만, 파이썬 제어는 PC에서 (Vision Protocol로 결과 수신 — 3번 참고).

## 3. NPU 동작 + 우리 모델 성능

```sh
ls /dev/npu*                          # NPU 디바이스 노드  <- **기록**
ls /usr/share/ | grep -i quantized    # 미리 깔린 네트워크 폴더  <- **기록**
ls /usr/lib/libtcndnpu*               # NPU 라이브러리 (-ltcndnpu)
ls /usr/lib/libvisionprotocol*        # Vision Protocol 라이브러리

# 기본 yolov8s로 NPU 단독 성능 측정 (카메라/디스플레이 없이)
tcnputestapp -n /usr/share/yolov8s_quantized -t 100
#   -> 추론 시간 ms  <- **기록** (모델 주 표의 15.49ms와 비교)
```

## 4. 카메라

```sh
ls /dev/video*                        # 노드 번호  <- **기록**
                                      # (tcnnapp 기본값이 /dev/video2, 문서 예시는 /dev/video0)
v4l2-ctl --list-devices               # 명령이 없으면 건너뜀
v4l2-ctl -d /dev/video0 --list-formats-ext   # 지원 포맷/해상도/FPS  <- **기록**

tcnncameraapp -p /dev/video0          # 화면에 영상 나오는지 (DSI 디스플레이 필요)
```

확인할 것: **우리가 쓰던 USB 웹캠이 보드에서 잡히는지**. 안 잡히면 보드 지원 카메라로
바꿔야 하고, 그러면 화각(hfov 64.0 / vfov 34.2)을 다시 측정해야 한다.

## 5. 탐지 데모 (카메라 → 화면)

PC에서 보려면 (보드에 모니터가 없을 때): 우리 앱을 `--preview-port 5001`로 실행하고
PC에서 `python3 pc_control/view_aig.py --board <보드IP>`. 색이 뒤바뀌면 `c` 키,
화면이 깨지면 카메라 버퍼 형식(RGB/YUV) <- **기록**

```sh
ls /dev/overlay                       # 출력 노드
tcnnapp -i camera -o display -n /usr/share/yolov8s_quantized/ -p /dev/video0 \
        -w 1280 -h 720 -W 800 -H 480
#   -> 화면에 박스가 뜨는지, 체감 FPS  <- **기록**
```

## 6. UART로 아두이노 제어 (포탑이 실제로 도는지)

가장 중요한 실측. 보드 ↔ 아두이노가 연결되지 않으면 보드 단독 구성이 불가능하다.

```sh
ls /dev/tty*                          # ttyAMA*, ttyS*, ttyUSB* 중 무엇이 있는지  <- **기록**
# 아두이노를 USB로 꽂고 다시
ls /dev/ttyUSB* /dev/ttyACM*          # 새로 생긴 노드  <- **기록**
dmesg | tail -20                      # 인식 로그 (ch340/cp210x/cdc_acm 드라이버 유무)

# 9600 8N1로 맞추고 (아두이노 리셋 대기 2초 후) 수동 명령
stty -F /dev/ttyUSB0 9600 cs8 -cstopb -parenb -echo raw
sleep 2
printf 's2 90\ns3 90\n' > /dev/ttyUSB0     # 중앙 정렬 -> 포탑이 움직이면 성공
printf 's2 60\n'        > /dev/ttyUSB0     # pan만 60도
cat /dev/ttyUSB0                           # 아두이노 응답 라인 확인 (Ctrl+C로 중단)
```

프로토콜은 1111과 동일(`firmware/model_control/model_control.ino`):
`s2 <deg>`=pan, `s3 <deg>`=tilt, `v<0-255>`=모터 세기, `t<ms>`=ms간 사격, `x`=정지.

> 드라이버가 없어서 `/dev/ttyUSB*`가 안 뜨면, 보드의 UART 핀(ttyAMA 계열)을
> 아두이노 RX/TX에 직결하는 방법으로 가야 한다 (레벨 3.3V/5V 확인 필요).

## 7. 네트워크 + PC 연결 (Vision Protocol)

```sh
ip a                                  # 보드 IP  <- **기록**
ping -c 3 <PC의 IP>                   # PC와 통신되는지
```

보드가 결과를 PC로 보내는 포트는 소스에 박혀 있다 (`tc-nn-app/src/NnRtpm.h`):
스트림 **9998**, 메시지 **9999**.

```sh
tcnnapp -i camera -o rtpm -n /usr/share/yolov8s_quantized/ -p /dev/video0
#   -> PC에서 9999 포트로 JSON이 들어오는지 확인
```

## 8. 보드에서 직접 빌드가 되는지 (크로스컴파일 생략 가능 여부)

```sh
which gcc make cmake pkg-config       # <- **기록**
pkg-config --libs tcndnpu             # -L.. -ltcndnpu 가 나오면 헤더/라이브러리 설치됨
ls /usr/include/npu_api.h
```

- 보드에 gcc가 있으면 개발이 훨씬 빠르다 (보드에서 바로 컴파일).
- 없으면 Ubuntu 20.04 호스트 + `aarch64-none-linux-gnu-gcc`(gcc-arm-9.2-2019.12)로
  크로스컴파일해야 한다.

---

## 실측 결과에 따른 분기

| 확인 결과 | 가는 방향 |
|---|---|
| 아두이노가 `/dev/ttyUSB*`로 잡히고 카메라도 잡힘 | **보드 단독 C 앱** (완전 임베디드) |
| 아두이노 UART은 되는데 보드에 gcc 없음 | 보드 단독이지만 호스트 크로스컴파일 필요 |
| 카메라나 UART 중 하나가 안 됨 | **보드=추론 / PC=파이썬 제어** (Vision Protocol JSON) |
| 보드에 파이썬이 있음 | 제어부를 보드 파이썬으로 (추론은 여전히 C 앱 + IPC) |
