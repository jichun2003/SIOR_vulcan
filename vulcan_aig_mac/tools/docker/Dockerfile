# AI-G NPU 변환용 Ubuntu 20.04 환경 (Ubuntu 24 호스트, 또는 Intel/AMD PC에서 Docker로 실행)
# 툴킷 문서 2장 기준: Ubuntu 20.04 64bit / Python 3.8 / torch 1.12.0+cpu / GPU 불필요
#                     크로스 컴파일러 gcc-arm-9.2-2019.12 (aarch64-none-linux-gnu)
#
# 빌드:  docker build -t aig-npu-sdk:20.04 .
# 실행:  ./run_sdk.sh <tc-nn-toolkit 폴더> <작업 폴더>

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive TZ=Asia/Seoul

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3.8 python3.8-dev python3.8-venv python3.8-tk python3-pip \
        build-essential make cmake git wget unzip xz-utils ca-certificates file \
        libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/bin/python3.8 /usr/bin/python \
    && python -m pip install --no-cache-dir --upgrade "pip<24"

# 툴킷 문서 2.2와 같은 버전
RUN pip install --no-cache-dir \
        torch==1.12.0+cpu torchvision==0.13.0+cpu torchaudio==0.12.0 \
        --extra-index-url https://download.pytorch.org/whl/cpu

# 툴킷 압축을 푼 폴더의 requirements.txt를 이 Dockerfile 옆에 복사한 뒤 빌드할 것 (문서 2.2)
COPY requirements.txt /tmp/sdk_requirements.txt
RUN pip install --no-cache-dir -r /tmp/sdk_requirements.txt

# build_network(net.so) 단계용 aarch64 크로스 컴파일러
# 툴킷 문서(3.4.1)는 ARM 배포판 gcc-arm-9.2-2019.12를 쓰지만 developer.arm.com 다운로드가
# 수십 KB/s로 느려서(1시간 이상), Ubuntu 20.04 저장소의 gcc-aarch64-linux-gnu(gcc 9.3,
# glibc 2.31)로 대신한다. 보드(Yocto Kirkstone, glibc 2.35)는 이보다 새 glibc라 호환된다.
# build_network/Makefile이 aarch64-none-linux-gnu-gcc 이름을 쓰므로 같은 이름으로 연결해 둔다.
# 실패하면 빌드가 멈추도록 set -eux, 국내 미러(카카오) + 재시도
# libsm6/libxext6/libxrender1: 툴킷의 opencv-python 4.3이 import 때 요구 (없으면 libSM.so.6 에러)
RUN set -eux; \
    sed -i 's|http://archive.ubuntu.com|http://mirror.kakao.com|g; s|http://security.ubuntu.com|http://mirror.kakao.com|g' /etc/apt/sources.list; \
    apt-get -o Acquire::Retries=5 update; \
    apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
        gcc-aarch64-linux-gnu libc6-dev-arm64-cross binutils-aarch64-linux-gnu \
        libsm6 libxext6 libxrender1 libice6; \
    rm -rf /var/lib/apt/lists/*; \
    for t in gcc ld ar as nm objdump strip; do \
        ln -sf "/usr/bin/aarch64-linux-gnu-$t" "/usr/local/bin/aarch64-none-linux-gnu-$t"; \
    done; \
    ln -sf /usr/bin/aarch64-linux-gnu-gcc /usr/local/bin/aarch64-none-linux-gnu-cc; \
    aarch64-none-linux-gnu-gcc --version; \
    python -c "import cv2; print('cv2', cv2.__version__)"

ENV ENLIGHT_SDK=/sdk
WORKDIR /work
