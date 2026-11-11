#!/usr/bin/env bash
# TOPST AI-G NPU 변환 파이프라인 (EnlightSDK: converter -> quantizer -> compiler -> build_network)
#
# 이 스크립트는 "AI-G SDK가 설치된 Ubuntu 20.04 호스트 PC"에서 돌린다.
# (NPU toolkit 문서 기준: Ubuntu 20.04 / Python 3.8 / torch 1.12.0+cpu / GPU 불필요,
#  크로스 컴파일은 aarch64-none-linux-gnu-gcc gcc-arm-9.2-2019.12)
# Mac이나 Windows에서는 돌지 않는다. ONNX 추출(export_onnx_npu.py)까지만 여기서 하고,
# 이 스크립트는 SDK 호스트로 .onnx를 옮긴 뒤 실행한다.
#
# 사용 예:
#   export ENLIGHT_SDK=/home/topst/tc-nn-toolkit
#   ./convert_npu.sh yolov8s_target_extracted.onnx ~/calib_npu 1
#
# 인자: 1) 추출된 onnx  2) 캘리브레이션 이미지 폴더(make_calib_set.py 결과)  3) 클래스 수
#
# !! 주의: EnlightSDK는 TOPST 전용 배포라 공개 문서에 모든 인자가 나와 있지 않다.
#    아래 CHECK 표시가 붙은 인자는 설치된 SDK의 `--help`로 한 번 확인하고 맞춰야 한다.

set -euo pipefail

ONNX="${1:?추출된 onnx 경로를 주세요 (export_onnx_npu.py 결과)}"
CALIB="${2:?캘리브레이션 이미지 폴더를 주세요 (make_calib_set.py 결과)}"
NUM_CLASS="${3:-1}"

SDK="${ENLIGHT_SDK:?ENLIGHT_SDK 환경변수에 tc-nn-toolkit 경로를 설정하세요}"
NAME="$(basename "${ONNX%.onnx}")"
WORK="${WORK_DIR:-./npu_build}"

# 탐지 임계값: 보드에서 쓰는 값과 맞춘다 (config의 conf_threshold 0.4 기준)
TH_CONF="${TH_CONF:-0.4}"
TH_IOU="${TH_IOU:-0.5}"

# 입력 정규화: ultralytics는 픽셀을 255로만 나눈다 (채널별 mean/std 보정 없음).
#   x = (pixel - 0) / 255
# 이 값을 틀리게 주면 int8 양자화 스케일이 어긋나서 검출이 통째로 망가진다.
MEAN="${MEAN:-0 0 0}"
STD="${STD:-255 255 255}"

echo "=== 0. 입력 확인 ==="
[ -f "$ONNX" ] || { echo "onnx가 없습니다: $ONNX" >&2; exit 1; }
[ -d "$CALIB" ] || { echo "캘리브레이션 폴더가 없습니다: $CALIB" >&2; exit 1; }
echo "onnx       : $ONNX"
echo "calib      : $CALIB ($(find "$CALIB" -type f | wc -l)장)"
echo "num-class  : $NUM_CLASS"
echo "mean / std : [$MEAN] / [$STD]"
mkdir -p "$WORK"

echo
echo "=== 1. converter: onnx -> .enlight (+ 활성값 통계 수집) ==="
# --type obj 로 Enlight detector 후처리가 붙으면, 보드에서 결과가 enlight_objs_t
#   (int cnt + enlight_obj_t obj[256]: x_min/x_max/y_min/y_max/cls/score)로 바로 나온다
#   -> post_process.c를 직접 쓸 필요가 없다. 가능하면 이 쪽이 낫다. (libtc-ndnpu/npu_api.h 확인)
# --type unknown: 후처리(DFL 디코드/NMS)를 그래프에 넣지 않고 우리가 보드 CPU에서 직접 한다.
#                 (export_onnx_npu.py가 헤드 conv 출력까지만 남겨둔 그래프라서)
#   -> 만약 Enlight 쪽 detector 후처리를 그대로 쓰고 싶다면 --type obj 와
#      --add-detection-post-process <anchor파일> 조합을 확인해야 한다 (CHECK)
# --enable-track: 캘리브레이션 데이터로 활성값 범위를 수집 (양자화 정확도에 필수)
python "$SDK/EnlightSDK/converter.py" "$ONNX" \
    --type unknown \
    --dataset Custom --dataset-root "$CALIB" \
    --enable-track \
    --mean $MEAN --std $STD \
    --num-class "$NUM_CLASS" \
    --output "$WORK/$NAME.enlight"            # CHECK: --force-output 필요 여부

echo
echo "=== 2. quantizer: float -> int8 ==="
python "$SDK/EnlightSDK/quantizer.py" "$WORK/$NAME.enlight" \
    --output "$WORK/${NAME}_quantized.enlight"

echo
echo "=== 3. compiler: int8 .enlight -> NPU 코드 ==="
# 결과: output_code/<network_name>/ 에 npu_cmd.bin, quantized_network.bin, network.h, post_process.c
python "$SDK/EnlightSDK/compiler.py" "$WORK/${NAME}_quantized.enlight" \
    --th-conf "$TH_CONF" --th-iou "$TH_IOU"

OUT_DIR="$(find . -type d -path "*output_code/*" -name "*${NAME}*" | head -1)"
OUT_DIR="${OUT_DIR:-output_code/$NAME}"
echo "컴파일 결과 폴더: $OUT_DIR"
ls -l "$OUT_DIR" || true

echo
echo "=== 4. build_network: net.so 빌드 ==="
echo "문서 절차대로 수동 확인이 필요한 단계입니다:"
cat <<'STEP'
  1) $OUT_DIR 의 내용을 SDK의 build_network/ 로 복사
  2) build_network/network.h, build_network/post_process.c 를 복사해온 파일로의
     심볼릭 링크로 교체
  3) build_network 에서 make  (aarch64-none-linux-gnu-gcc 필요)
  4) 생성된 net.so 를 네트워크 폴더(npu_cmd.bin / quantized_network.bin 옆)로 복사

  * post_process.c 는 compiler가 만들어준 기본 후처리다. 우리는 헤드를 잘라낸
    그래프(box/cls conv 출력 6개)를 쓰므로, DFL 디코드 + sigmoid + NMS를
    tools/export_onnx_npu.py 의 decode_raw_outputs()/nms() 와 동일하게
    구현해 넣어야 한다 (그 numpy 구현이 ultralytics 결과와 0.0001px 이내로 일치함을
    확인한 기준 구현이다).
STEP

echo
echo "=== 5. 보드로 전송 ==="
echo "  scp -r $OUT_DIR root@<보드IP>:/home/root/"
echo "  보드에서 확인:  tcnnapp -n /home/root/$NAME/ -p /dev/video0"
