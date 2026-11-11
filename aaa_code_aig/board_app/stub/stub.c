/*
 * PC 시험용 가짜 Telechips API (-DAIG_STUB_NPU).
 *
 * 카메라/스케일러/NPU가 없는 PC에서 보드 앱(main.c, npu_vision.c)이 그대로 빌드되고
 * 돌아가는지, JSON 출력/CSV 로그/UART 쓰기 경로가 동작하는지 확인하기 위한 것이다.
 * 추론 결과는 좌우로 왕복하는 가짜 표적 하나를 만들어 돌려준다 (도르래 표적 모사).
 *
 * 제어 로직(PID/α-β/지연보상)의 수렴 성능은 여기서 보는 게 아니라 test_control.c의
 * 닫힌 루프 시뮬레이션에서 본다. 이 스텁은 "연결과 배관"만 확인한다.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "camera_api.h"
#include "npu_api.h"
#include "scaler_api.h"

#define STUB_NN_W 640
#define STUB_NN_H 640

static double stub_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

/* ---- 카메라 ---- */
static int    g_cam_dummy;
static double g_t0;
static unsigned char *g_frame;

int32_t CameraCreate(CameraHandle *pHandle)
{
    *pHandle = &g_cam_dummy;
    g_t0 = stub_now();
    return 0;
}

int32_t CameraDestroy(CameraHandle handle) { (void)handle; free(g_frame); g_frame = NULL; return 0; }

int32_t CameraOpenDevice(CameraHandle handle, char *inputDevName)
{
    (void)handle;
    printf("[stub] 카메라 열기(가짜): %s\n", inputDevName);
    return 0;
}

int32_t CameraCloseDevice(CameraHandle handle) { (void)handle; return 0; }

int32_t CameraSetConfig(CameraHandle handle, uint32_t width, uint32_t height)
{
    (void)handle;
    free(g_frame);
    g_frame = (unsigned char *)calloc((size_t)width * height * 3u, 1u);
    printf("[stub] 카메라 설정(가짜): %ux%u\n", width, height);
    return 0;
}

int32_t CameraGetBuffer(CameraHandle handle, uint8_t **virtualAddr, uint64_t *baseOffset)
{
    (void)handle;
    /* 실제 카메라의 프레임 간격(30fps)을 흉내내서 루프가 너무 빨리 돌지 않게 */
    usleep(33 * 1000);
    *virtualAddr = g_frame;
    *baseOffset = (uint64_t)(uintptr_t)g_frame;
    return 0;
}

int32_t CameraReleaseBuffer(CameraHandle handle) { (void)handle; return 0; }
cam_status_t CameraGetStatus(CameraHandle handle) { (void)handle; return CAMERA_STATUS_STREAMING; }

/* ---- 스케일러 (아무것도 하지 않음) ---- */
static int g_scaler_dummy;
int32_t ScalerCreate(ScalerHandle *pHandle) { *pHandle = &g_scaler_dummy; return 0; }
int32_t ScalerDestroy(ScalerHandle h) { (void)h; return 0; }
int32_t ScalerOpenDevice(ScalerHandle h, const char *n, scaler_index_t i) { (void)h; (void)n; (void)i; return 0; }
int32_t ScalerCloseDevice(ScalerHandle h, scaler_index_t i) { (void)h; (void)i; return 0; }
int32_t ScalerResize(ScalerHandle h, scaler_index_t i, scaler_size_info_t s, scaler_size_info_t d)
{ (void)h; (void)i; (void)s; (void)d; return 0; }
int32_t ScalerCropResize(ScalerHandle h, scaler_index_t i, scaler_size_info_t s, scaler_size_info_t d,
                         uint32_t a, uint32_t b, uint32_t c, uint32_t e)
{ (void)h; (void)i; (void)s; (void)d; (void)a; (void)b; (void)c; (void)e; return 0; }
int32_t ScalerPoll(ScalerHandle h, scaler_index_t i) { (void)h; (void)i; return 0; }
int32_t ScalerGetStatus(ScalerHandle h, scaler_index_t i) { (void)h; (void)i; return 0; }

/* ---- NPU ---- */
static npu_t     g_npu = { 7 };
static npu_net_t g_net = { 7, NULL, NULL };

npu_t *npu_open(int minor) { (void)minor; printf("[stub] npu_open(가짜)\n"); return &g_npu; }
int    npu_close(npu_t *npu) { (void)npu; return 0; }

npu_net_t *network_load_from_file(npu_t *npu, char *sofile, char *cmdfile, char *paramfile)
{
    (void)npu;
    printf("[stub] 네트워크 로드(가짜): %s / %s / %s\n", sofile, cmdfile, paramfile);
    return &g_net;
}

int network_close(npu_net_t *net) { (void)net; return 0; }
int network_set_color_format(npu_net_t *net, int fmt) { (void)net; (void)fmt; return 0; }
int network_get_input_width(npu_net_t *net) { (void)net; return STUB_NN_W; }
int network_get_input_height(npu_net_t *net) { (void)net; return STUB_NN_H; }
int network_get_output_size(npu_net_t *net) { (void)net; return 1024; }
int network_get_type(npu_net_t *net) { (void)net; return TELECHIPS_NPU_POST_DETECTOR; }

npu_buf_t *buffer_alloc(npu_t *npu, int size)
{
    npu_buf_t *b = (npu_buf_t *)calloc(1, sizeof(*b));
    (void)npu;
    if (b == NULL) {
        return NULL;
    }
    b->size = size;
    b->caddr = (char *)calloc((size_t)size, 1u);
    b->paddr = (unsigned long)(uintptr_t)b->caddr;
    return b;
}

char *buffer_get_addr(npu_buf_t *buf) { return buf->caddr; }

int buffer_close(npu_buf_t *buf)
{
    if (buf != NULL) {
        free(buf->caddr);
        free(buf);
    }
    return 0;
}

int network_run(npu_net_t *net, npu_buf_t *in, npu_buf_t *out, npu_err_bits_t *status,
                npu_run_mode_t mode, npu_perf_t *perf, int timeout_in_ms)
{
    (void)net; (void)in; (void)out; (void)mode; (void)timeout_in_ms;
    if (status != NULL) {
        status->bits = 0u;
    }
    if (perf != NULL) {
        perf->elapsed_in_us = 15490u;   /* 모델 주의 yolov8s 15.49ms */
        perf->dma = 0u;
        perf->comp = 0u;
        perf->all = 0u;
    }
    return 0;
}

int network_run_postprocess(npu_net_t *net, npu_buf_t *out, void *result)
{
    enlight_objs_t *objs = (enlight_objs_t *)result;
    double t = stub_now() - g_t0;
    /* 좌우 왕복 (주기 4초, NN 좌표 중앙 ±200px) + 세로는 약간만 흔들림 */
    double cx = STUB_NN_W / 2.0 + 200.0 * sin(2.0 * 3.14159265 * t / 4.0);
    double cy = STUB_NN_H / 2.0 + 15.0 * sin(2.0 * 3.14159265 * t / 7.0);
    double w = 40.0, h = 30.0;

    (void)net; (void)out;
    memset(objs, 0, sizeof(*objs));
    objs->cnt = 1;
    objs->obj[0].x_min = (float)(cx - w / 2.0);
    objs->obj[0].x_max = (float)(cx + w / 2.0);
    objs->obj[0].y_min = (float)(cy - h / 2.0);
    objs->obj[0].y_max = (float)(cy + h / 2.0);
    objs->obj[0].cls = 0;
    objs->obj[0].score = 0.87f;
    objs->obj[0].img_w = STUB_NN_W;
    objs->obj[0].img_h = STUB_NN_H;
    return 0;
}
