#include "npu_vision.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Telechips SDK 헤더 (보드/SDK 호스트에서만 존재).
 * AIG_STUB_NPU로 빌드하면 stub/ 안의 가짜 구현을 써서 PC에서 제어 로직만 시험할 수 있다. */
#include <camera_api.h>
#include <npu_api.h>
#ifndef AIG_SW_RESIZE
#include <scaler_api.h>
#endif

/* AIG_SW_RESIZE: 하드웨어 스케일러(tcc_scaler_ioctrl.h 필요) 대신 CPU nearest-neighbor 리사이즈.
 * Ubuntu 22.04 보드 이미지에 Telechips 스케일러 헤더가 없어서 추가 (2026-11-20). */

#define AIG_NPU_TIMEOUT_MS 1000
#define SO_FILE    "net.so"
#define CMD_FILE   "npu_cmd.bin"
#define PARAM_FILE "quantized_network.bin"

struct aig_vision {
    const aig_config_t *cfg;

    CameraHandle camera;
#ifndef AIG_SW_RESIZE
    ScalerHandle scaler;
    scaler_index_t scaler_idx;
#else
    uint32_t *xoff;     /* 출력 x -> 카메라 행 안 바이트 오프셋 (미리 계산) */
    uint32_t xoff_sx0, xoff_sw;
#endif
    /* 카메라 픽셀 형식: camera_api.c가 V4L2_PIX_FMT_RGB32(4바이트/픽셀)를 요청한다.
     * 바이트 순서는 환경변수 AIG_CAM_ORDER(BGRX/RGBX/XRGB/XBGR, 기본 BGRX)로 바꿀 수 있다.
     * 2026-11-24 프레임 색 통계: BGRX에서 세 채널이 자연스럽고, XRGB는 파랑이 255로 고정(X 바이트) -> BGRX */
    uint32_t cam_bpp;
    int      off_r, off_g, off_b;
    /* NPU 입력 값 변환표 (AIG_IN_MODE: half=v*128/255 [기본], raw=그대로, s128=v^0x80).
     * 2026-11-24: NPU는 입력 바이트를 int8로 그대로 읽는다. SDK 시뮬레이터와 같은 0~128 범위(half)가 맞다. */
    uint8_t  in_lut[256];

    npu_t      *npu;
    npu_net_t  *net;
    npu_buf_t  *in_buf;
    npu_buf_t  *out_buf;

    uint32_t nn_w;      /* 스케일러 목적지 폭 (16의 배수로 정렬된 값) */
    uint32_t nn_h;
    uint32_t net_w;     /* 네트워크가 요구한 원래 입력 폭 (좌표 복원에 사용) */
    uint32_t net_h;

    enlight_objs_t objs;

    /* PC 미리보기 (aig_vision_request_preview) */
    int      preview_step;      /* 0 = 이번 프레임은 복사 안 함 */
    bool     preview_ready;
    uint8_t *preview;
    size_t   preview_cap;
    uint32_t preview_w, preview_h;
};

/* 카메라 RGB888 프레임을 step 간격으로 솎아 복사 (보간 없음, 1280x720 -> 640x360이면
 * 약 0.7MB 읽기라 보드 CPU로도 1ms 안팎) */
static void copy_preview(aig_vision_t *v, const uint8_t *src)
{
    const aig_config_t *cfg = v->cfg;
    uint32_t step = (uint32_t)v->preview_step;
    uint32_t w = cfg->frame_width / step;
    uint32_t h = cfg->frame_height / step;
    size_t need = (size_t)w * h * 3u;
    size_t src_stride = (size_t)cfg->frame_width * v->cam_bpp;

    if (need > v->preview_cap) {
        uint8_t *nb = (uint8_t *)realloc(v->preview, need);
        if (nb == NULL) {
            return;
        }
        v->preview = nb;
        v->preview_cap = need;
    }
    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *row = src + (size_t)y * step * src_stride;
        uint8_t *dst = v->preview + (size_t)y * w * 3u;
        for (uint32_t x = 0; x < w; x++) {
            const uint8_t *p = row + (size_t)x * step * v->cam_bpp;
            dst[0] = p[v->off_r];
            dst[1] = p[v->off_g];
            dst[2] = p[v->off_b];
            dst += 3;
        }
    }
    v->preview_w = w;
    v->preview_h = h;
    v->preview_ready = true;
}

/* 카메라 바이트 순서 설정 (AIG_CAM_ORDER, AIG_CAM_BPP 환경변수) */
static void set_cam_order(aig_vision_t *v)
{
    const char *ord = getenv("AIG_CAM_ORDER");
    const char *bpp = getenv("AIG_CAM_BPP");
    v->cam_bpp = 4u;
    if (bpp != NULL && atoi(bpp) == 3) {
        v->cam_bpp = 3u;
    }
    if (ord == NULL) {
        ord = v->cam_bpp == 3u ? "RGB" : "BGRX";
    }
    if (strcmp(ord, "RGBX") == 0 || strcmp(ord, "RGB") == 0) {
        v->off_r = 0; v->off_g = 1; v->off_b = 2;
    } else if (strcmp(ord, "XRGB") == 0) {
        v->off_r = 1; v->off_g = 2; v->off_b = 3;
    } else if (strcmp(ord, "XBGR") == 0) {
        v->off_r = 3; v->off_g = 2; v->off_b = 1;
    } else { /* BGRX, BGR */
        ord = v->cam_bpp == 3u ? "BGR" : "BGRX";
        v->off_r = 2; v->off_g = 1; v->off_b = 0;
    }
    printf("[vision]   카메라 픽셀: %u바이트, 순서 %s (AIG_CAM_ORDER로 변경)\n", v->cam_bpp, ord);

    {
        const char *m = getenv("AIG_IN_MODE");
        if (m == NULL) {
            m = "half";
        }
        for (int i = 0; i < 256; i++) {
            if (strcmp(m, "half") == 0) {
                int q = (i * 128 + 127) / 255;                      /* SDK: round(v/255*128) */
                v->in_lut[i] = (uint8_t)(q > 127 ? 127 : q);        /* int8 상한 127로 clamp (128은 -128이 됨) */
            } else if (strcmp(m, "s128") == 0) {
                v->in_lut[i] = (uint8_t)(i ^ 0x80);                /* v-128 (int8) */
            } else {
                m = "raw";
                v->in_lut[i] = (uint8_t)i;
            }
        }
        printf("[vision]   NPU 입력 값: %s (AIG_IN_MODE=raw/half/s128)\n", m);
    }
}

#ifdef AIG_SW_RESIZE
/* 카메라 프레임의 (sx0, sy0, sw, sh) 영역을 NPU 입력 버퍼(RGB24, 폭 nn_w)로 nearest 리사이즈 */
static int sw_resize(aig_vision_t *v, const uint8_t *src,
                     uint32_t sx0, uint32_t sy0, uint32_t sw, uint32_t sh)
{
    const aig_config_t *cfg = v->cfg;
    size_t stride = (size_t)cfg->frame_width * v->cam_bpp;
    uint8_t *dst = (uint8_t *)v->in_buf->caddr;
    const int ro = v->off_r, go = v->off_g, bo = v->off_b;
    const uint8_t *lut = v->in_lut;

    if (dst == NULL || src == NULL) {
        return -1;
    }
    if (v->xoff == NULL || v->xoff_sx0 != sx0 || v->xoff_sw != sw) {
        uint32_t *t = (uint32_t *)realloc(v->xoff, sizeof(uint32_t) * v->net_w);
        if (t == NULL) {
            return -1;
        }
        v->xoff = t;
        for (uint32_t x = 0; x < v->net_w; x++) {
            uint32_t xs = sx0 + (uint32_t)(((uint64_t)x * sw) / v->net_w);
            v->xoff[x] = xs * v->cam_bpp;
        }
        v->xoff_sx0 = sx0;
        v->xoff_sw = sw;
    }
    for (uint32_t y = 0; y < v->net_h; y++) {
        uint32_t ys = sy0 + (uint32_t)(((uint64_t)y * sh) / v->net_h);
        const uint8_t *row = src + (size_t)ys * stride;
        uint8_t *d = dst + (size_t)y * v->nn_w * 3u;
        for (uint32_t x = 0; x < v->net_w; x++) {
            const uint8_t *p = row + v->xoff[x];
            d[0] = lut[p[ro]];
            d[1] = lut[p[go]];
            d[2] = lut[p[bo]];
            d += 3;
        }
    }
    return 0;
}
#endif

void aig_vision_request_preview(aig_vision_t *v, int step)
{
    v->preview_step = step < 1 ? 1 : step;
}

const uint8_t *aig_vision_preview(const aig_vision_t *v, uint32_t *w, uint32_t *h)
{
    if (!v->preview_ready) {
        return NULL;
    }
    *w = v->preview_w;
    *h = v->preview_h;
    return v->preview;
}

/* RGB24 입력은 폭이 16의 배수여야 한다 (tc-nn-app의 alignInputDataWidth와 동일) */
static uint32_t align_up(uint32_t v, uint32_t multiple)
{
    if (multiple == 0u) {
        return v;
    }
    return ((v + multiple - 1u) / multiple) * multiple;
}

aig_vision_t *aig_vision_open(const aig_config_t *cfg)
{
    aig_vision_t *v;
    char sofile[512], cmdfile[512], paramfile[512];
    int in_w, in_h, in_size, out_size;

    v = (aig_vision_t *)calloc(1, sizeof(*v));
    if (v == NULL) {
        return NULL;
    }
    v->cfg = cfg;
#ifndef AIG_SW_RESIZE
    v->scaler_idx = SCALER_INDEX_0;
#endif
    set_cam_order(v);

    /* --- 카메라 --- */
    if (CameraCreate(&v->camera) != 0) {
        fprintf(stderr, "[vision] CameraCreate 실패\n");
        goto fail;
    }
    if (CameraOpenDevice(v->camera, (char *)cfg->camera_dev) != 0) {
        fprintf(stderr, "[vision] 카메라를 열 수 없습니다: %s\n", cfg->camera_dev);
        goto fail;
    }
    if (CameraSetConfig(v->camera, cfg->frame_width, cfg->frame_height) != 0) {
        fprintf(stderr, "[vision] CameraSetConfig(%ux%u) 실패\n",
                cfg->frame_width, cfg->frame_height);
        goto fail;
    }

#ifndef AIG_SW_RESIZE
    /* --- 스케일러 --- */
    if (ScalerCreate(&v->scaler) != 0) {
        fprintf(stderr, "[vision] ScalerCreate 실패\n");
        goto fail;
    }
    if (ScalerOpenDevice(v->scaler, SCALER_DEV_NAME_0, v->scaler_idx) != 0) {
        fprintf(stderr, "[vision] ScalerOpenDevice(%s) 실패\n", SCALER_DEV_NAME_0);
        goto fail;
    }
#endif

    /* --- NPU --- */
    v->npu = npu_open(cfg->npu_cluster);
    if (v->npu == NULL) {
        fprintf(stderr, "[vision] npu_open(%d) 실패 (/dev/npu* 권한/존재 확인)\n", cfg->npu_cluster);
        goto fail;
    }
    snprintf(sofile, sizeof(sofile), "%s/%s", cfg->network_dir, SO_FILE);
    snprintf(cmdfile, sizeof(cmdfile), "%s/%s", cfg->network_dir, CMD_FILE);
    snprintf(paramfile, sizeof(paramfile), "%s/%s", cfg->network_dir, PARAM_FILE);
    v->net = network_load_from_file(v->npu, sofile, cmdfile, paramfile);
    if (v->net == NULL) {
        fprintf(stderr, "[vision] 네트워크 로드 실패: %s\n", cfg->network_dir);
        goto fail;
    }
    /* 입력은 RGB (기본값이지만 명시한다) */
    network_set_color_format(v->net, NPU_COLOR_RGB);

    in_w = network_get_input_width(v->net);
    in_h = network_get_input_height(v->net);
    if (in_w <= 0 || in_h <= 0) {
        fprintf(stderr, "[vision] 입력 크기를 알 수 없습니다 (%d x %d)\n", in_w, in_h);
        goto fail;
    }
    v->net_w = (uint32_t)in_w;
    v->net_h = (uint32_t)in_h;
    v->nn_w = align_up((uint32_t)in_w, 16u);   /* RGB24: 폭 16 정렬 */
    v->nn_h = (uint32_t)in_h;

    in_size = (int)(3u * v->nn_w * v->nn_h);
    v->in_buf = buffer_alloc(v->npu, in_size);
    out_size = network_get_output_size(v->net);
    v->out_buf = buffer_alloc(v->npu, out_size);
    if (v->in_buf == NULL || v->out_buf == NULL) {
        fprintf(stderr, "[vision] buffer_alloc 실패 (in %d, out %d)\n", in_size, out_size);
        goto fail;
    }

#ifdef AIG_SW_RESIZE
    if (v->in_buf->caddr != NULL) {
        memset(v->in_buf->caddr, 0, (size_t)in_size);
    }
    printf("[vision]   리사이즈: CPU nearest (AIG_SW_RESIZE)\n");
#endif
    printf("[vision] NPU 네트워크 로드 완료: %s\n", cfg->network_dir);
    printf("[vision]   입력 %ux%u (정렬 폭 %u), 출력 버퍼 %d바이트, 후처리 타입 %d\n",
           v->net_w, v->net_h, v->nn_w, out_size, network_get_type(v->net));
    printf("[vision]   카메라 %s %ux%u -> %s\n", cfg->camera_dev,
           cfg->frame_width, cfg->frame_height,
           cfg->fit_mode == AIG_FIT_CROP ? "가운데 정사각형 crop 후 리사이즈" : "그대로 리사이즈(stretch)");
    return v;

fail:
    aig_vision_close(v);
    return NULL;
}

void aig_vision_close(aig_vision_t *v)
{
    if (v == NULL) {
        return;
    }
    if (v->net != NULL) {
        network_close(v->net);
    }
    if (v->in_buf != NULL) {
        buffer_close(v->in_buf);
    }
    if (v->out_buf != NULL) {
        buffer_close(v->out_buf);
    }
    if (v->npu != NULL) {
        npu_close(v->npu);
    }
#ifndef AIG_SW_RESIZE
    if (v->scaler != NULL) {
        ScalerCloseDevice(v->scaler, v->scaler_idx);
        ScalerDestroy(v->scaler);
    }
#else
    free(v->xoff);
#endif
    if (v->camera != NULL) {
        CameraCloseDevice(v->camera);
        CameraDestroy(v->camera);
    }
    free(v->preview);
    free(v);
}

void aig_vision_input_size(const aig_vision_t *v, uint32_t *w, uint32_t *h)
{
    if (w != NULL) {
        *w = v->net_w;
    }
    if (h != NULL) {
        *h = v->net_h;
    }
}

int aig_vision_capture_detect(aig_vision_t *v, aig_det_t *dets, int max_dets,
                              int *n_dets, float *infer_ms)
{
    const aig_config_t *cfg = v->cfg;
    uint8_t *virt = NULL;
    uint64_t phys = 0;
#ifndef AIG_SW_RESIZE
    scaler_size_info_t src, dst;
#endif
    npu_err_bits_t err_status;
    npu_perf_t perf;
    int n = 0;
    /* NPU 입력 좌표 -> 원본 프레임 좌표 변환 계수 */
    float sx, sy, offx, offy;

    *n_dets = 0;
    v->preview_ready = false;

    if (CameraGetBuffer(v->camera, &virt, &phys) != 0) {
        fprintf(stderr, "[vision] CameraGetBuffer 실패\n");
        return -1;
    }
    if (v->preview_step > 0 && virt != NULL) {
        copy_preview(v, virt);
        v->preview_step = 0;
    }

#ifdef AIG_SW_RESIZE
    (void)phys;
    if (cfg->fit_mode == AIG_FIT_CROP) {
        uint32_t side = cfg->frame_width < cfg->frame_height ? cfg->frame_width : cfg->frame_height;
        uint32_t cx0 = (cfg->frame_width - side) / 2u;
        uint32_t cy0 = (cfg->frame_height - side) / 2u;
        if (sw_resize(v, virt, cx0, cy0, side, side) != 0) {
            fprintf(stderr, "[vision] sw_resize 실패\n");
            CameraReleaseBuffer(v->camera);
            return -1;
        }
        sx = (float)side / (float)v->net_w;
        sy = (float)side / (float)v->net_h;
        offx = (float)cx0;
        offy = (float)cy0;
    } else {
        if (sw_resize(v, virt, 0u, 0u, cfg->frame_width, cfg->frame_height) != 0) {
            fprintf(stderr, "[vision] sw_resize 실패\n");
            CameraReleaseBuffer(v->camera);
            return -1;
        }
        sx = (float)cfg->frame_width / (float)v->net_w;
        sy = (float)cfg->frame_height / (float)v->net_h;
        offx = 0.0f;
        offy = 0.0f;
    }
    CameraReleaseBuffer(v->camera);
#else
    src.pmap = phys;
    src.width = cfg->frame_width;
    src.height = cfg->frame_height;
    src.format = SCALER_FORMAT_RGB888;

    dst.pmap = v->in_buf->paddr;
    dst.width = v->nn_w;
    dst.height = v->nn_h;
    dst.format = SCALER_FORMAT_RGB888;

    if (cfg->fit_mode == AIG_FIT_CROP) {
        /* 가운데 정사각형만 잘라서 넣는다 (종횡비 유지 = 학습 때 letterbox와 비슷한 그림) */
        uint32_t side = cfg->frame_width < cfg->frame_height ? cfg->frame_width : cfg->frame_height;
        uint32_t cx0 = (cfg->frame_width - side) / 2u;
        uint32_t cy0 = (cfg->frame_height - side) / 2u;
        if (ScalerCropResize(v->scaler, v->scaler_idx, src, dst,
                             cx0, cy0, cx0 + side, cy0 + side) != 0) {
            fprintf(stderr, "[vision] ScalerCropResize 실패\n");
            CameraReleaseBuffer(v->camera);
            return -1;
        }
        sx = (float)side / (float)v->net_w;
        sy = (float)side / (float)v->net_h;
        offx = (float)cx0;
        offy = (float)cy0;
    } else {
        if (ScalerResize(v->scaler, v->scaler_idx, src, dst) != 0) {
            fprintf(stderr, "[vision] ScalerResize 실패\n");
            CameraReleaseBuffer(v->camera);
            return -1;
        }
        sx = (float)cfg->frame_width / (float)v->net_w;
        sy = (float)cfg->frame_height / (float)v->net_h;
        offx = 0.0f;
        offy = 0.0f;
    }
    ScalerPoll(v->scaler, v->scaler_idx);
    CameraReleaseBuffer(v->camera);
#endif

    if (network_run(v->net, v->in_buf, v->out_buf, &err_status,
                    NPU_RUN_SYNC, &perf, AIG_NPU_TIMEOUT_MS) < 0) {
        fprintf(stderr, "[vision] network_run 실패\n");
        return -1;
    }
    if (infer_ms != NULL) {
        *infer_ms = (float)perf.elapsed_in_us / 1000.0f;
    }

    memset(&v->objs, 0, sizeof(v->objs));
    if (network_run_postprocess(v->net, v->out_buf, &v->objs) != 0) {
        fprintf(stderr, "[vision] network_run_postprocess 실패\n");
        return -1;
    }

    for (int i = 0; i < v->objs.cnt && n < max_dets; i++) {
        const enlight_obj_t *o = &v->objs.obj[i];
        aig_det_t *d;

        if (o->score < cfg->conf_threshold) {
            continue;
        }
        d = &dets[n++];
        d->x1 = o->x_min * sx + offx;
        d->y1 = o->y_min * sy + offy;
        d->x2 = o->x_max * sx + offx;
        d->y2 = o->y_max * sy + offy;
        d->cx = (d->x1 + d->x2) * 0.5f;
        d->cy = (d->y1 + d->y2) * 0.5f;
        d->w = d->x2 - d->x1;
        d->h = d->y2 - d->y1;
        d->conf = o->score;
        d->cls = (int)o->cls;
    }
    *n_dets = n;
    return 0;
}

int aig_pick_target(const aig_det_t *dets, int n, int target_class,
                    const float *last_center, float max_jump_px)
{
    int best = -1;
    bool any_near = false;

    if (n <= 0) {
        return -1;
    }

    /* 가까운 후보가 하나라도 있으면 그 중에서만 고르고, 없으면 전체에서 고른다(재포착) */
    if (last_center != NULL && max_jump_px > 0.0f) {
        for (int i = 0; i < n; i++) {
            if (target_class >= 0 && dets[i].cls != target_class) {
                continue;
            }
            if (hypotf(dets[i].cx - last_center[0], dets[i].cy - last_center[1]) <= max_jump_px) {
                any_near = true;
                break;
            }
        }
    }

    for (int i = 0; i < n; i++) {
        if (target_class >= 0 && dets[i].cls != target_class) {
            continue;
        }
        if (any_near &&
            hypotf(dets[i].cx - last_center[0], dets[i].cy - last_center[1]) > max_jump_px) {
            continue;
        }
        if (best < 0 || dets[i].conf > dets[best].conf) {
            best = i;
        }
    }
    return best;
}
