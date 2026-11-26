# vulcan_mac — Mac 단독 추적 버전 (비행기 / 풍선 표적 전환)

AIG 보드 없이 **Mac이 직접** 카메라 + YOLO로 표적을 탐지·추적하고, 아두이노에 USB 시리얼로
서보·사격 명령을 보낸다. 추적/PID/사격/리드샷 로직은 `aaa_code_1111`과 동일하고,
**표적만 커맨드로 고르는** 점이 다르다.

## 표적 전환 — `.pt`만 바뀐다
Mac(ultralytics)은 `.pt`를 그대로 실행하므로 표적 전환이 간단하다. 커맨드로 고른다:

| `--target` | 가중치 | 탐지 클래스 | 설명 |
|---|---|---|---|
| `airplane` (기본) | `airplane.pt` | `target` | 원래의 회색 비행기 |
| `balloon` | `balloon.pt` | `balloon` | 풍선 |

두 가중치 모두 이 폴더에 들어 있다. `--model`/`--target-class`를 직접 주면 프리셋보다 우선한다.

> 참고 — AIG **보드**에서는 이렇게 간단히 안 된다. 보드 NPU는 `.pt`가 아니라 Telechips 툴킷으로
> 양자화·컴파일한 전용 바이너리(`vulcan_s16_quantized/`)만 돌린다. 현재 비행기 모델만 그 형태로
> 변환돼 있어, 보드에서 풍선을 쓰려면 `balloon.pt → ONNX → tc-nn-toolkit`으로 풍선 NPU 모델을
> 따로 만들어 올려야 한다. (vulcan_mac은 그 변환이 필요 없다)

## 설치
핵심 패키지(ultralytics, opencv, pyserial)는 보통 이미 깔려 있다. 없으면:
```bash
pip3 install -r requirements.txt     # 또는 python3 -m pip install -r requirements.txt
```

## 실행
```bash
cd vulcan_mac

# 비행기 표적, 아두이노 없이 비전/추적만 확인
python3 main.py --no-serial

# 풍선 표적으로 전환
python3 main.py --target balloon --no-serial

# 포탑(아두이노) 연결해서 추적  (포트는 ls /dev/cu.usb* 로 확인, 하나뿐이면 --port 생략 가능)
python3 main.py --target airplane --port /dev/cu.usbmodem1101
python3 main.py --target balloon  --port /dev/cu.usbmodem1101
```
화면 키: `q`=종료, `r`=리셋, **`s`=현재 프레임 저장**(`captures/`), `x`=사격취소, `스페이스`=사격신호.
콘솔 입력: `shot`(1번째 선도각 장입 / 2번째 사격), `cancel`, `r`, `q`.

자주 쓰는 옵션: `--pan-sign -1` / `--tilt-sign -1`(포탑이 표적 반대로 갈 때), `--conf 0.35`(탐지 문턱),
`--control legacy --ballistic-json ballistic/ballistic_model.json --range-m 1.5`(탄도 선도각).

## 표적을 잘 못 잡을 때 (재학습)
`s` 키로 못 잡는 장면을 `captures/`에 모은 뒤(놓친 프레임은 `..._miss.jpg`), 풍선은
`balloon_code/train/retrain.py`, 비행기는 `aaa_code_1111/train/retrain.py`로 재학습해
해당 `.pt`를 갱신하고 이 폴더의 `airplane.pt`/`balloon.pt`를 덮어쓴다.
