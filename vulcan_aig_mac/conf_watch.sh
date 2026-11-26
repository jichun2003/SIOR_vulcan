#!/bin/bash
# [보드에서 실행] 영상 전송 없이, 표적 인식 여부와 confidence만 출력한다 (뷰어 불필요, 발열 적음)
#   bash conf_watch.sh                    -> 1초마다 요약, Ctrl+C까지
#   DUR=40 bash conf_watch.sh             -> 40초 후 앱까지 정상 종료
#   MODE=frame bash conf_watch.sh         -> 프레임마다 한 줄
#   뒤에 앱 옵션 추가 가능 (예: --log-csv 경로, --fit crop, --conf 0.4)
# 출력: "검출 n/m 프레임" = NPU가 찾은 프레임 수 / 전체, "conf" = 확신도(0~1)
cd "$(dirname "$0")"
D="$(pwd)"
APP="$D/board_app/build/aig_turret"
NET="$D/vulcan_s16_quantized"
sudo pkill -9 aig_turret 2>/dev/null; sleep 1
TO=""; [ "${DUR:-0}" -gt 0 ] && TO="timeout -s INT $DUR"
sudo AIG_CAM_ORDER="${AIG_CAM_ORDER:-BGRX}" AIG_IN_MODE="${AIG_IN_MODE:-half}" $TO stdbuf -oL "$APP" \
    --network "$NET" --camera /dev/video2 --no-serial "$@" 2>&1 | \
awk -v mode="${MODE:-sec}" '
/object is detected/ {
    n = $1 + 0
    if (n == 0) { z++; if (mode == "frame") { printf("  미검출\n"); fflush() } }
    else { nz++; seenN = 1 }
    next
}
/score:/ {
    sc = $NF / 100; cls = "?"
    if (match($0, /class: *[0-9]+/)) { c = substr($0, RSTART, RLENGTH); gsub(/[^0-9]/, "", c); cls = c }
    s++; sum += sc; if (sc > mx) mx = sc; seen[cls] = 1
    if (mode == "frame") { printf("  검출  class=%s  conf=%.2f\n", cls, sc); fflush() }
    next
}
/^\[status\]/ {
    if (mode == "frame") next
    t++; df = seenN ? nz : s; fr = z + df
    cl = ""; for (k in seen) cl = cl k " "; if (cl == "") cl = "-"
    printf("%4ds  검출 %2d/%2d 프레임 (%3.0f%%)  class %s  conf 평균 %.2f 최대 %.2f\n",
           t, df, fr, fr ? 100 * df / fr : 0, cl, s ? sum / s : 0, mx)
    fflush(); z = nz = s = sum = mx = 0; delete seen; next
}
/^\[vision\]|실패|ERROR|error/ { print; fflush() }
'
