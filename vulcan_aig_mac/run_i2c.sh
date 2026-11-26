#!/bin/bash
# [보드에서 실행] 인지부 + 구동부 실행 (I2C로 우노에 서보/사격 명령, PC 미리보기 포트 5001)
# run.sh와 같고 --no-serial 대신 --port /dev/i2c-1 을 쓴다. (2026-11-26)
#   배선: 40핀 3번(SDA) -> 우노 A4, 40핀 5번(SCL) -> 우노 A5, 40핀 9번(GND) -> 우노 GND
#   우노 펌웨어: model_control_i2c_v2.ino (I2C 주소 0x08)
# 사용법: bash run_i2c.sh                      -> 기본
#         bash run_i2c.sh --tilt-sign -1       -> 추가 옵션은 뒤에 붙이면 그대로 전달됨
#         AIG_I2C_ADDR=0x09 bash run_i2c.sh    -> 우노 주소를 바꿨을 때
#         RESTART=0 bash run_i2c.sh            -> 카메라가 끊겨도 자동 재시작하지 않음
# 신호: 실행 중 터미널에 shot(1번=선도각, 2번=사격) / cancel / r(중앙 복귀) / q(종료) 입력
# 종료: q 또는 Ctrl+C -> 사격 정지(x) + 서보 힘 풀기(d)를 우노에 보내고 끝난다
#       (안 되면 다른 창에서 sudo pkill -9 aig_turret)
#
# [v2 2026-11-26]
#  - 종료할 때 우노에 x(사격 정지), d(서보 힘 풀기)를 I2C로 보냄 (i2c-tools 필요)
#  - 카메라 프레임이 3초 끊겨 앱이 exit 3으로 끝나면 자동으로 다시 띄움 (최대 RESTART회, 기본 5)
cd "$(dirname "$0")"
D="$(pwd)"
I2C_ADDR="${AIG_I2C_ADDR:-0x08}"
MAX_RESTART="${RESTART:-5}"

# 표적 선택: AIG_TARGET=airplane(기본) -> vulcan_s16_quantized(회색 비행기)
#            AIG_TARGET=balloon         -> balloon_s16_quantized(풍선, balloon.pt를 NPU로 변환한 것)
case "${AIG_TARGET:-airplane}" in
    airplane) NET_DIR="$D/vulcan_s16_quantized" ;;
    balloon)  NET_DIR="$D/balloon_s16_quantized" ;;
    *) echo "[run_i2c] 알 수 없는 AIG_TARGET='${AIG_TARGET}' (airplane|balloon 중 하나)"; exit 2 ;;
esac
[ -d "$NET_DIR" ] || { echo "[run_i2c] NPU 모델 폴더가 없습니다: $NET_DIR"; echo "  (balloon은 tools로 변환한 뒤 생깁니다. README_Mac.md의 '풍선 NPU 변환' 참고)"; exit 2; }
echo "[run_i2c] 표적=${AIG_TARGET:-airplane}, 모델=$NET_DIR"

release_turret() {
    # "x\nd\n" = 사격 정지 + 서보 detach
    sudo i2ctransfer -y 1 w4@"$I2C_ADDR" 0x78 0x0a 0x64 0x0a >/dev/null 2>&1 \
        && echo "[run_i2c] 우노에 사격 정지(x) + 서보 힘 풀기(d) 전송" \
        || echo "[run_i2c] 우노에 x/d 전송 실패 (우노 전원/배선 확인)"
}
trap release_turret EXIT

# 이전 실행이 남아 카메라를 잡고 있으면 "CameraSetConfig 실패"가 난다 -> 먼저 정리
sudo pkill -9 aig_turret 2>/dev/null; sleep 1

n=0
while :; do
    sudo AIG_CAM_ORDER="${AIG_CAM_ORDER:-BGRX}" AIG_IN_MODE="${AIG_IN_MODE:-half}" AIG_I2C_ADDR="$I2C_ADDR" \
        "$D/board_app/build/aig_turret" \
        --network "$NET_DIR" \
        --camera /dev/video2 \
        --port /dev/i2c-1 \
        --preview-port 5001 \
        --log-csv "$D/perception.csv" \
        "$@" 2>&1 | grep --line-buffered -v "object is detected"
    rc=${PIPESTATUS[0]}
    if [ "$rc" -ne 3 ]; then
        break                       # q / Ctrl+C / 그 밖의 종료
    fi
    n=$((n + 1))
    if [ "$n" -gt "$MAX_RESTART" ]; then
        echo "[run_i2c] 카메라 끊김이 ${MAX_RESTART}번 넘게 반복 -> 중단. CSI 케이블/발열 확인"
        break
    fi
    echo "[run_i2c] 카메라 프레임 끊김(exit 3) -> 2초 뒤 다시 시작 (${n}/${MAX_RESTART})"
    sleep 2
done
