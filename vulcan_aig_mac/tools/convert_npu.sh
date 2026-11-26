#!/usr/bin/env bash
# TOPST AI-G NPU 변환 파이프라인 (EnlightSDK: converter -> quantizer -> compiler -> build_network)
#
# 툴킷 예시 run_example_pytorch_yolo_onnx.sh 의 yolov8 설정을 우리 모델(yolov8s, 클래스 1개)에
# 맞춘 것. 툴킷은 x86-64 리눅스 전용(PyArmor 런타임이 linux/x86_64뿐)이라
# tools/docker/run_sdk.sh 로 띄운 Ubuntu 20.04 컨테이너 안에서 실행한다.
#
# 사용 (컨테이너 안, /work 에서):
#   ./convert_npu.sh yolov8s_target_extracted.onnx calib_npu 1
# 결과:
#   ./yolov8s_target_quantized/   (net.so, npu_cmd.bin, quantized_network.bin ...)
#   -> scp -r yolov8s_target_quantized root@<보드IP>:/home/root/
#      (board_app 기본값 network_dir = /home/root/yolov8s_target_quantized 와 같은 이름)
#
# 인자: 1) 추출된 onnx  2) 캘리브레이션 이미지 폴더  3) 클래스 수 (기본 1)
# 환경변수로 바꿀 수 있는 값: NET_NAME, TH_CONF, TH_IOU, MEAN, STD, GRID, LETTERBOX=1, WORK_DIR

set -euo pipefail

ONNX="${1:?추출된 onnx 경로를 주세요 (export_onnx_npu.py 결과)}"
CALIB="${2:?캘리브레이션 이미지 폴더를 주세요 (make_calib_set.py 결과)}"
NUM_CLASS="${3:-1}"

SDK="${ENLIGHT_SDK:?ENLIGHT_SDK 환경변수에 tc-nn-toolkit 경로를 설정하세요 (컨테이너에서는 /sdk)}"
STEM="$(basename "${ONNX%.onnx}")"
NET="${NET_NAME:-${STEM%_extracted}}"          # yolov8s_target_extracted -> yolov8s_target
WORK="${WORK_DIR:-./npu_build}"
HERE="$(pwd)"

# 보드 쪽 conf 임계값(aig_config.h conf_threshold 0.4)과 맞춘다. 보드 앱이 한 번 더 거른다.
TH_CONF="${TH_CONF:-0.4}"
TH_IOU="${TH_IOU:-0.5}"

# 입력 정규화: 툴킷 yolov8 예시와 동일 (mean 0 / std 1 = ultralytics의 pixel/255).
# SDK가 먼저 255로 나눈 뒤 mean/std를 적용한다. std 255로 주면 입력이 거의 0이 되어 망가진다.
MEAN="${MEAN:-0 0 0}"
STD="${STD:-1 1 1}"

# YOLOv8 격자 파일: 640 입력의 80/40/20 격자 중심좌표만 담긴 파일이라 모델 크기(m/s)·클래스 수와
# 무관하다. 툴킷의 yolov8m.bin 을 그대로 쓴다 (값을 직접 재계산해서 동일함을 확인함).
GRID="${GRID:-$SDK/input_networks/yolov8m.bin}"

# 레터박스: 툴킷 예시는 켜지만, 이 옵션은 캘리브레이션 이미지 전처리에만 쓰인다
# (converter.py: CustomDatasetManager(letter_box=...)). 보드 앱 기본이 --fit stretch 라
# 캘리브레이션도 stretch로 맞추기 위해 기본은 끈다. 보드에서 --fit crop/레터박스를 쓰면 LETTERBOX=1.
LB_ARG=""
[ "${LETTERBOX:-0}" = "1" ] && LB_ARG="--enable-letterbox"

echo "=== 0. 입력 확인 ==="
[ -f "$ONNX" ]  || { echo "onnx가 없습니다: $ONNX" >&2; exit 1; }
[ -d "$CALIB" ] || { echo "캘리브레이션 폴더가 없습니다: $CALIB" >&2; exit 1; }
[ -f "$GRID" ]  || { echo "격자 파일이 없습니다: $GRID" >&2; exit 1; }
NUM_IMAGES="$(find "$CALIB" -type f \( -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.png' -o -iname '*.bmp' \) | wc -l | tr -d ' ')"
[ "$NUM_IMAGES" -gt 0 ] || { echo "캘리브레이션 폴더에 이미지가 없습니다" >&2; exit 1; }
echo "onnx       : $ONNX"
echo "network    : $NET"
echo "calib      : $CALIB (${NUM_IMAGES}장)"
echo "num-class  : $NUM_CLASS"
echo "mean / std : [$MEAN] / [$STD]"
echo "grid       : $GRID"
echo "letterbox  : ${LB_ARG:-off}"
mkdir -p "$WORK"
WORK="$(cd "$WORK" && pwd)"        # compiler를 /sdk에서 돌리므로 절대경로로
CALIB_ABS="$(cd "$CALIB" && pwd)"

if [ -f "$WORK/${NET}_quantized.enlight" ] && [ "${FORCE:-0}" != "1" ]; then
    echo
    echo "=== 1~2. 이미 양자화된 모델이 있어 건너뜀 ($WORK/${NET}_quantized.enlight)"
    echo "    (처음부터 다시 하려면 FORCE=1 bash convert_npu.sh ...)"
else
echo
echo "=== 1. converter: onnx -> .enlight (+ 캘리브레이션 통계) ==="
# --type obj + --yolo-version v8 + --dfl-reg-max 16: SDK 내장 YOLOv8 후처리(DFL 디코드, sigmoid, NMS)를
#   net.so에 넣는다 -> 보드에서 network_run_postprocess()가 enlight_objs_t로 박스를 바로 준다.
# --output-order lc: 우리 onnx 출력 순서가 (box, cls) x 3 이라서.
#   (툴킷 yolov8m 예시는 (cls, box) 순서라 cl. 자르는 위치 cv2/cv3 마지막 Conv는 우리와 같다)
# --num-images: 기본 100장이라 캘리브레이션 폴더 전체를 쓰도록 지정
python "$SDK/EnlightSDK/converter.py" "$ONNX" \
    --type obj \
    --add-detection-post-process "$GRID" \
    --yolo-version v8 \
    --dfl-reg-max 16 \
    --output-order lc \
    --dataset Custom --dataset-root "$CALIB" \
    --num-images "$NUM_IMAGES" \
    --enable-track \
    --mean $MEAN --std $STD \
    --num-class "$NUM_CLASS" \
    $LB_ARG \
    --output "$WORK/$NET.enlight"

echo
echo "=== 2. quantizer: float -> int8 ==="
python "$SDK/EnlightSDK/quantizer.py" "$WORK/$NET.enlight" \
    --output "$WORK/${NET}_quantized.enlight"
fi

echo
echo "=== 3. compiler: int8 .enlight -> NPU 코드 ==="
# compiler.py는 './EnlightSDK/enlight/code_emission/cfg/hw_config.py'를 상대경로로 쓰므로
# 툴킷 루트($SDK)에서 실행해야 한다 (툴킷 예시 스크립트도 루트에서 실행). 결과는 --dump-root로 작업 폴더에.
(
    cd "$SDK"
    python EnlightSDK/compiler.py "$WORK/${NET}_quantized.enlight" \
        --dump-root "$HERE/output_code" \
        --th-iou "$TH_IOU" --th-conf "$TH_CONF"
)

CODE_DIR="$HERE/output_code/${NET}_quantized"
[ -d "$CODE_DIR" ] || CODE_DIR="$(find "$HERE" -type d -path "*output_code/${NET}_quantized" | head -1)"
[ -d "$CODE_DIR" ] || { echo "컴파일 결과 폴더를 찾지 못했습니다 (output_code/${NET}_quantized)" >&2; exit 1; }
echo "컴파일 결과: $CODE_DIR"
ls -l "$CODE_DIR"

echo
echo "=== 4. build_network: net.so 빌드 (툴킷 문서 3.4.2 절차) ==="
# 툴킷 폴더를 더럽히지 않도록 build_network를 작업 폴더로 복사해서 빌드한다
BUILD="$WORK/build_network"
rm -rf "$BUILD"
cp -r "$SDK/build_network" "$BUILD"
cp -ar "$CODE_DIR" "$BUILD/"
(
    cd "$BUILD"
    rm -f network.h post_process.c
    ln -s "./${NET}_quantized/network.h"
    ln -s "./${NET}_quantized/post_process.c"
    make
    cp ./net.so "./${NET}_quantized/"
)
file "$BUILD/net.so"

OUT="$HERE/${NET}_quantized"
rm -rf "$OUT"
cp -ar "$BUILD/${NET}_quantized" "$OUT"
for f in net.so npu_cmd.bin quantized_network.bin; do
    [ -f "$OUT/$f" ] || { echo "결과물에 $f 가 없습니다" >&2; exit 1; }
done
echo "보드로 보낼 폴더: $OUT"
ls -l "$OUT/net.so" "$OUT/npu_cmd.bin" "$OUT/quantized_network.bin"

echo
echo "=== 5. (확인용) PC 시뮬레이터로 int8 모델 한 장 돌려보기 ==="
SAMPLE="$(find "$CALIB_ABS" -type f \( -iname '*.jpg' -o -iname '*.png' \) | head -1)"
( cd "$SDK" && python EnlightSDK/enlight_sim.py "$WORK/${NET}_quantized.enlight" \
    --inputs "$SAMPLE" --th-iou "$TH_IOU" --th-conf "$TH_CONF" $LB_ARG ) \
    || echo "  (시뮬레이터 실행 실패 - 변환 결과에는 영향 없음)"

echo
echo "=== 6. 보드로 전송 ==="
echo "  scp -r ${NET}_quantized root@<보드IP>:/home/root/"
echo "  보드 기본 앱으로 먼저 확인:  tcnnapp -p /dev/video0 -n /home/root/${NET}_quantized/"
