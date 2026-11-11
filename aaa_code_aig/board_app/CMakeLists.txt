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

add_executable(aig_turret
    main.c
    npu_vision.c
    uart.c
    json_out.c
    ${CONTROL_SRC}
)

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
    target_sources(aig_turret PRIVATE
        ${AIG_TCNNAPP_DIR}/common/camera/camera_api.c
        ${AIG_TCNNAPP_DIR}/common/display/scaler_api.c
    )
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
        message(STATUS "AI-G: pkg-config에 tcndnpu가 없어 -ltcndnpu로 직접 링크합니다")
        target_link_libraries(aig_turret PRIVATE tcndnpu)
    endif()
endif()

target_link_libraries(aig_turret PRIVATE pthread m)

# 제어 로직 닫힌 루프 시뮬레이션 (항상 빌드 가능 — 보드 API 불필요)
add_executable(test_control test_control.c ${CONTROL_SRC})
target_link_libraries(test_control PRIVATE m)

enable_testing()
add_test(NAME control_sim COMMAND test_control)
