cmake_minimum_required(VERSION 3.13)
project(aig_turret C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

# 제어 로직만 (카메라/NPU와 무관) — 테스트와 공유
set(CONTROL_SRC pid.c turret.c tracking_filter.c fire.c)

# -----------------------------------------------------------------------------
# 1) PC 시험 빌드:  cmake -DAIG_STUB_NPU=ON ..
#    카메라/NPU가 없는 PC에서 가짜 API로 빌드해 배관(JSON/CSV/UART)을 확인한다.
# 2) 보드 빌드:     cmake -DAIG_TCNNAPP_DIR=<tc-nn-app 소스 경로> ..
#    camera_api.c / scaler_api.c는 라이브러리가 아니라 tc-nn-app의 소스라서 같이 컴파일한다.
#    (github.com/topst-development/tc-nn-app, 브랜치 develop)
#    NPU 라이브러리는 -ltcndnpu (libtc-ndnpu/lib/tcndnpu.pc.in 참고)
# -----------------------------------------------------------------------------
option(AIG_STUB_NPU "Build with fake Telechips APIs for PC testing" OFF)
set(AIG_TCNNAPP_DIR "" CACHE PATH "Path to tc-nn-app source tree (board build)")
# Ubuntu 22.04 보드 이미지 대응 (2026-11-20):
#  - AIG_SW_RESIZE: tcc_scaler_ioctrl.h가 없어 scaler_api.c를 빼고 CPU 리사이즈 사용
#  - AIG_NPU_INCLUDE_DIR: npu_api.h 위치 (libtc-ndnpu/lib/include)
option(AIG_SW_RESIZE "Use CPU nearest resize instead of Telechips HW scaler" OFF)
set(AIG_NPU_INCLUDE_DIR "" CACHE PATH "Directory containing npu_api.h")

add_executable(aig_turret
    main.c
    npu_vision.c
    uart.c
    json_out.c
    preview_out.c
    ${CONTROL_SRC}
)

# PC 미리보기 영상 JPEG 압축 (--preview-port). libjpeg(-turbo)가 없으면 원시 RGB로 보낸다.
find_package(JPEG QUIET)
if(JPEG_FOUND)
    target_include_directories(aig_turret PRIVATE ${JPEG_INCLUDE_DIRS})
    target_link_libraries(aig_turret PRIVATE ${JPEG_LIBRARIES})
    target_compile_definitions(aig_turret PRIVATE AIG_HAVE_JPEG=1)
    message(STATUS "AI-G: 미리보기 JPEG 압축 사용 (${JPEG_LIBRARIES})")
else()
    message(STATUS "AI-G: libjpeg 없음 -> 미리보기는 원시 RGB (--preview-scale 4 권장)")
endif()

if(AIG_STUB_NPU)
    target_sources(aig_turret PRIVATE stub/stub.c)
    target_include_directories(aig_turret PRIVATE stub)
    target_compile_definitions(aig_turret PRIVATE AIG_STUB_NPU=1)
    message(STATUS "AI-G: 스텁 빌드 (PC 시험용). 카메라/NPU는 가짜입니다")
else()
    if(NOT AIG_TCNNAPP_DIR)
        message(FATAL_ERROR
            "보드 빌드에는 -DAIG_TCNNAPP_DIR=<tc-nn-app 소스 경로>가 필요합니다 "
            "(camera_api.c/scaler_api.c가 거기 있습니다). "
            "PC에서 로직만 시험하려면 -DAIG_STUB_NPU=ON")
    endif()
    target_sources(aig_turret PRIVATE ${AIG_TCNNAPP_DIR}/common/camera/camera_api.c)
    if(AIG_SW_RESIZE)
        target_compile_definitions(aig_turret PRIVATE AIG_SW_RESIZE=1)
        message(STATUS "AI-G: CPU 리사이즈 (하드웨어 스케일러 미사용)")
    else()
        target_sources(aig_turret PRIVATE ${AIG_TCNNAPP_DIR}/common/display/scaler_api.c)
    endif()
    if(AIG_NPU_INCLUDE_DIR)
        target_include_directories(aig_turret PRIVATE ${AIG_NPU_INCLUDE_DIR})
    endif()
    target_include_directories(aig_turret PRIVATE
        ${AIG_TCNNAPP_DIR}/common/camera
        ${AIG_TCNNAPP_DIR}/common/display
        ${AIG_TCNNAPP_DIR}/include
    )
    # npu_api.h / libtcndnpu: SDK 기본 설치 위치. pkg-config가 있으면 그쪽을 쓴다.
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(TCNDNPU QUIET tcndnpu)
    endif()
    if(TCNDNPU_FOUND)
        target_include_directories(aig_turret PRIVATE ${TCNDNPU_INCLUDE_DIRS})
        target_link_libraries(aig_turret PRIVATE ${TCNDNPU_LIBRARIES})
    else()
        # 런타임 패키지에는 libtcndnpu.so(개발용 링크)가 없고 libtcndnpu.so.1만 있을 수 있다
        find_library(TCNDNPU_LIB NAMES tcndnpu libtcndnpu.so.1 PATHS /usr/lib /usr/lib/aarch64-linux-gnu)
        if(NOT TCNDNPU_LIB)
            message(FATAL_ERROR "libtcndnpu를 찾을 수 없습니다 (/usr/lib/libtcndnpu.so.1 확인)")
        endif()
        message(STATUS "AI-G: NPU 라이브러리 ${TCNDNPU_LIB}")
        target_link_libraries(aig_turret PRIVATE ${TCNDNPU_LIB} dl)
    endif()
endif()

target_link_libraries(aig_turret PRIVATE pthread m)

# 제어 로직 닫힌 루프 시뮬레이션 (항상 빌드 가능 — 보드 API 불필요)
add_executable(test_control test_control.c ${CONTROL_SRC})
target_link_libraries(test_control PRIVATE m)

enable_testing()
add_test(NAME control_sim COMMAND test_control)
