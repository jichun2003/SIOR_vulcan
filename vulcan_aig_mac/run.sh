#!/bin/bash
# [보드에서 실행] 인지부 실행 (서보 없이, PC 미리보기 포트 5001)
# 사용법: bash run.sh                      -> 기본 (BGRX + half, conf 0.3, 클래스 무시)
#         bash run.sh --fit crop           -> 추가 옵션은 뒤에 붙이면 그대로 전달됨
#         AIG_CAM_ORDER=RGBX bash run.sh   -> 색 순서 바꿔 시험 (진단용)
#         AIG_IN_MODE=raw bash run.sh      -> 입력 값 변환 바꿔 시험 (진단용)
# 종료: Ctrl+C (안 되면 다른 창에서 sudo pkill -9 aig_turret)
cd "$(dirname "$0")"
D="$(pwd)"
# 표적 선택: AIG_TARGET=airplane(기본)/balloon
case "${AIG_TARGET:-airplane}" in
    airplane) NET_DIR="$D/vulcan_s16_quantized" ;;
    balloon)  NET_DIR="$D/balloon_s16_quantized" ;;
    *) echo "[run] 알 수 없는 AIG_TARGET='${AIG_TARGET}' (airplane|balloon)"; exit 2 ;;
esac
[ -d "$NET_DIR" ] || { echo "[run] NPU 모델 폴더가 없습니다: $NET_DIR (README_Mac.md의 '풍선 NPU 변환' 참고)"; exit 2; }
echo "[run] 표적=${AIG_TARGET:-airplane}, 모델=$NET_DIR"
# 이전 실행이 남아 카메라를 잡고 있으면 "CameraSetConfig 실패"가 난다 -> 먼저 정리
sudo pkill -9 aig_turret 2>/dev/null; sleep 1
sudo AIG_CAM_ORDER="${AIG_CAM_ORDER:-BGRX}" AIG_IN_MODE="${AIG_IN_MODE:-half}" "$D/board_app/build/aig_turret" \
    --network "$NET_DIR" \
    --camera /dev/video2 \
    --no-serial \
    --preview-port 5001 \
    --log-csv "$D/perception.csv" \
    "$@" 2>&1 | grep --line-buffered -v "object is detected"
