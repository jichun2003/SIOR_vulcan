#!/bin/sh
# AI-G 보드에서 실행하는 환경 조사 스크립트.
# 읽기만 한다 — 포탑을 움직이거나 사격하지 않는다 (UART 쓰기 테스트는 일부러 뺐다).
#
# 쓰는 법: 보드 콘솔에 이 파일 내용을 그대로 붙여넣거나,
#          scp로 올린 뒤  sh board_check.sh 2>&1 | tee board_check.txt
# 그리고 출력 전체를 복사해서 주면 된다.

echo "===== 1. 시스템 ====="
uname -a
cat /etc/os-release 2>/dev/null | head -4
echo "CPU 코어: $(nproc 2>/dev/null)"
free -h 2>/dev/null | head -2
df -h / 2>/dev/null | tail -1

echo
echo "===== 2. 파이썬 ====="
for p in python3 python pip3; do
    w=$(command -v $p 2>/dev/null)
    echo "$p: ${w:-없음}"
done
python3 -V 2>&1
python3 -c "import cv2; print('cv2', cv2.__version__)" 2>&1 | tail -1
python3 -c "import numpy; print('numpy', numpy.__version__)" 2>&1 | tail -1

echo
echo "===== 3. NPU ====="
ls -1 /dev/npu* 2>&1
echo "-- 설치된 네트워크 --"
ls -1 /usr/share/ 2>/dev/null | grep -i quantized
echo "-- 라이브러리 --"
ls -1 /usr/lib/libtcndnpu* /usr/lib/libvisionprotocol* 2>&1
ls -1 /usr/include/npu_api.h 2>&1
echo "-- 앱 --"
for a in tcnnapp tcnputestapp tcnncameraapp; do
    w=$(command -v $a 2>/dev/null)
    echo "$a: ${w:-없음}"
done

echo
echo "===== 4. 카메라 ====="
ls -1 /dev/video* 2>&1
ls -1 /dev/overlay 2>&1
if command -v v4l2-ctl >/dev/null 2>&1; then
    for d in /dev/video0 /dev/video1 /dev/video2; do
        [ -e "$d" ] || continue
        echo "-- $d --"
        v4l2-ctl -d "$d" --list-formats-ext 2>&1 | head -20
    done
else
    echo "v4l2-ctl 없음"
fi

echo
echo "===== 5. 시리얼 (아두이노 연결용) ====="
echo "-- tty 노드 --"
ls -1 /dev/ttyUSB* /dev/ttyACM* /dev/ttyAMA* /dev/ttyS* 2>&1
echo "-- USB 장치 --"
if command -v lsusb >/dev/null 2>&1; then lsusb 2>&1; else echo "lsusb 없음"; fi
echo "-- USB 시리얼 드라이버 로그 --"
dmesg 2>/dev/null | grep -iE "ch34|cp210|ftdi|cdc_acm|usbserial|ttyUSB|ttyACM" | tail -10

echo
echo "===== 6. 네트워크 ====="
ip -4 addr show 2>/dev/null | grep -E "inet |^[0-9]+:" || ifconfig 2>&1 | head -12

echo
echo "===== 7. 빌드 도구 (보드에서 직접 컴파일 가능한지) ====="
for t in gcc g++ make cmake pkg-config; do
    w=$(command -v $t 2>/dev/null)
    echo "$t: ${w:-없음}"
done
pkg-config --libs tcndnpu 2>&1
pkg-config --libs visionprotocol 2>&1

echo
echo "===== 끝. 위 출력을 전부 복사해서 전달 ====="
