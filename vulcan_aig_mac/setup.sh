#!/bin/bash
# [보드에서 실행] 빌드 스크립트. 사용법: bash setup.sh
# 이 폴더(vulcan_aig_bundle) 안에서 모든 것이 끝난다. 기존 ~/board_app 등은 건드리지 않는다.
set -e
cd "$(dirname "$0")"
D="$(pwd)"

echo "== 1/4 빌드 도구 확인 =="
if ! command -v cmake >/dev/null || ! command -v gcc >/dev/null || ! dpkg -s libjpeg-dev >/dev/null 2>&1 || ! command -v i2ctransfer >/dev/null; then
    echo "빌드 도구가 없어 설치합니다 (인터넷 필요)"
    sudo apt-get update && sudo apt-get install -y build-essential cmake pkg-config libjpeg-dev v4l-utils i2c-tools
else
    echo "이미 설치돼 있음"
fi

echo "== 2/4 NPU 확인 (우리 모델 20회 실행) =="
sudo tcnputestapp -n "$D/vulcan_s16_quantized" -t 20 | tail -n 6

echo "== 3/4 카메라 확인 =="
if [ -e /dev/video2 ]; then
    echo "/dev/video2 있음"
else
    echo "경고: /dev/video2 가 없음. 전원을 끄고 CSI 케이블을 확인한 뒤 다시 부팅할 것"
fi

echo "== 4/4 aig_turret 빌드 =="
rm -rf board_app/build && mkdir -p board_app/build && cd board_app/build
cmake -DAIG_TCNNAPP_DIR="$D/tc-nn-app" -DAIG_SW_RESIZE=ON -DAIG_NPU_INCLUDE_DIR="$D/npu_include" .. > cmake.log
make -j4 aig_turret 2>&1 | tail -n 5
ls -l aig_turret
echo
echo "완료. 실행은: bash $D/run_i2c.sh  (우노 I2C 연결)  /  bash $D/run.sh  (인지부만)"
