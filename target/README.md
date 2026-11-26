# target — 이동 표적 리그 (좌우 왕복)

양 끝 모터 2개를 줄로 타겟에 연결해 **좌우로 왕복**시키는 장치. PC가 USB로 아두이노 2개를
동기 구동한다. (대공포 추적/사격 테스트용 움직이는 표적)

## 1. 구성
PC(두뇌) + **Arduino Uno ×2**(각자 모터 1개) + **L298N ×2** + **TT 기어드 DC모터 ×2**
기구: 양 끝에 모터, 줄로 타겟 연결 → 좌우 왕복, **줄 중간에 고무줄(번지) 한 토막**.
PC가 USB로 두 보드에 명령을 쏴서 두 모터를 동기화. (리미트 스위치 없음 → **시간 기반 왕복**)

## 2. 준비물
- Arduino Uno ×2, USB 케이블 ×2
- L298N 모터 드라이버 ×2 (보드당 1개, **채널 A만** 사용)
- TT 기어드 DC모터 ×2
- **모터용 전원 5~6V 배터리팩** (모터 전원은 USB와 별도)
- 줄 + 고무줄 한 토막

## 3. 배선 (보드당, L298N 채널 A → Uno)
| L298N | Uno | 비고 |
|---|---|---|
| ENA | **D9 (PWM)** | **점퍼 캡 제거 필수** (안 빼면 속도조절 안 됨) |
| IN1 | D7 | 방향 |
| IN2 | D8 | 방향 |
| OUT1/OUT2 | 모터 2선 | — |
| +12V | 배터리 +(5~6V) | 모터 전원 별도 |
| GND | 배터리 − **와 Uno GND 공통** | **공통 접지 필수** |

`5V-EN` 점퍼는 그대로, **모터를 Uno 5V에서 끌지 말 것.**

## 4. 명령 체계
시리얼(115200)로 정수 전송: **부호=방향, 크기=속도(PWM 0~255)**
- `+` = 감기(당김), `-` = 풀기, `0` = 정지
- 두 보드 모두 `+` = **내 쪽으로 당김**으로 통일 (반대로 돌면 그 보드 **모터선 2가닥 교체**)
- 동작: 왼쪽 = A 감기 + B 풀기 / 오른쪽 = A 풀기 + B 감기 / 좌→정지→우→정지 반복

## 5. 소프트웨어 (2개)
| 코드 | 위치 | 역할 |
|---|---|---|
| 아두이노 펌웨어 | `firmware/target_motor/target_motor.ino` | **두 보드에 동일 업로드**. 시리얼 숫자로 모터 구동, 3초 끊기면 안전정지(watchdog) |
| PC 파이썬 | `target_control.py` | 두 포트 열고 왕복 루프. 파라미터 `--speed(S) / --kb(보정) / --stroke(T)` |

## 6. 셋업 순서 체크리스트
1. **두 보드 배선** (3번 표대로)
2. **펌웨어 업로드** (A·B 보드 각각 동일). arduino-cli 예:
   ```bash
   arduino-cli compile --fqbn arduino:avr:uno firmware/target_motor
   arduino-cli upload -p <포트> --fqbn arduino:avr:uno firmware/target_motor
   ```
3. **방향 확인**: 아래 명령으로 A가 "당기는" 방향인지 본다. 반대면 그 보드 모터선 교체.
   ```bash
   pip3 install -r requirements.txt            # pyserial (없으면)
   python3 target_control.py --list            # 포트 확인
   python3 target_control.py --port-a <A> --port-b <B> --test a   # A만 1.5초 감기
   python3 target_control.py --port-a <A> --port-b <B> --test b   # B 확인
   ```
4. **줄 + 고무줄 연결**, 타겟을 가운데 위치
5. **왕복 실행**:
   ```bash
   python3 target_control.py --port-a <A> --port-b <B> --speed 180 --stroke 1.5 --pause 0.4 --kb 1.0
   ```
   (Windows: `python target_control.py --port-a COM3 --port-b COM4 ...`)
6. 왕복 보며 **`--kb`(좌우 속도 보정)·`--stroke`(스트로크 길이) 미세조정**. Ctrl+C로 정지.

## 7. 튜닝 · 주의점
- **스풀 반지름 변화**: 감을수록 스풀이 굵어져 감는 양 > 푸는 양 → 어긋남 누적. **고무줄로 흡수 + 스트로크(T) 짧게**가 해법.
- **개체 편차**: 한쪽 모터가 빠르면 `--kb`로 보정 (기본 B 보드에 곱함, `--kb-board`로 변경).
- **TT 모터는 약해서 관대**: 줄이 팽팽해져도 슬립/정지, 안 부서짐. 단 `--speed`가 너무 낮으면(≲80) 안 움직일 수 있음.
- **피드백 없음**: 위치가 서서히 밀림 → 몇 분 돌리고 수동으로 가운데 리셋.
- **3대 실수**: ① ENA 점퍼 안 뺌(속도조절 안 됨) ② 공통 접지 빠뜨림 ③ 방향 확인 안 함.
