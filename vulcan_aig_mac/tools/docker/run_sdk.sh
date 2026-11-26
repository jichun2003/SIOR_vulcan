#!/usr/bin/env bash
# 20.04 SDK 컨테이너를 띄운다.
#
# 사용:  ./run_sdk.sh <tc-nn-toolkit 폴더> <작업 폴더>
#   작업 폴더 안에 넣어둘 것:
#     yolov8s_target_extracted.onnx
#     calib_npu/            (make_calib_set.py 결과, 300장)
#     convert_npu.sh
#
# 컨테이너 안에서:
#   /sdk  = tc-nn-toolkit (ENLIGHT_SDK=/sdk 로 이미 설정됨)
#   /work = 작업 폴더 (결과물이 그대로 호스트 폴더에 남는다)
#
# Apple Silicon Mac에서 시도할 때: PLATFORM=linux/amd64 ./run_sdk.sh ...
#   (Docker Desktop 설정에서 Rosetta 에뮬레이션을 켤 것. 공식 지원 환경은 아님)

set -euo pipefail

SDK="$(cd "${1:?tc-nn-toolkit 폴더 경로를 주세요}" && pwd)"
WORK="$(cd "${2:?작업 폴더 경로를 주세요 (onnx, calib_npu, convert_npu.sh)}" && pwd)"
IMG="aig-npu-sdk:20.04"
HERE="$(cd "$(dirname "$0")" && pwd)"
PLAT_ARGS=()
[ -n "${PLATFORM:-}" ] && PLAT_ARGS=(--platform "$PLATFORM")

[ -d "$SDK/EnlightSDK" ] || echo "경고: $SDK/EnlightSDK 가 없습니다. SDK 폴더가 맞는지 확인하세요" >&2

if ! docker image inspect "$IMG" >/dev/null 2>&1; then
    if [ ! -f "$HERE/requirements.txt" ]; then
        if [ -f "$SDK/requirements.txt" ]; then
            cp "$SDK/requirements.txt" "$HERE/requirements.txt"
        else
            echo "툴킷의 requirements.txt를 $HERE 에 복사한 뒤 다시 실행하세요" >&2
            exit 1
        fi
    fi
    echo "=== 이미지가 없어 처음 한 번 빌드합니다 (몇 분 걸림) ==="
    docker build ${PLAT_ARGS[@]+"${PLAT_ARGS[@]}"} -t "$IMG" "$HERE"
fi

# 내 uid/gid로 실행 -> 결과 파일이 root 소유로 생기지 않는다
docker run --rm -it ${PLAT_ARGS[@]+"${PLAT_ARGS[@]}"} \
    -u "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$SDK":/sdk \
    -v "$WORK":/work \
    "$IMG" bash
