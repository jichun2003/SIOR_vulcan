#include "npu_vision.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Telechips SDK 헤더 (보드/SDK 호스트에서만 존재).
 * AIG_STUB_NPU로 빌드하면 stub/ 안의 가짜 구현을 써서 PC에서 제어 로직만 시험할 수 있다. */
#include <camera_api.h>
#include <npu_api.h>
#include <scaler_api.h>

#define AIG_NPU_TIMEOUT_MS 1000
#define SO_FILE    "net.so"
#define CMD_FILE   "npu_cmd.bin"
#define PARAM_FILE "quantized_network.bin"

struct aig_vision {
    const aig_config_t *cfg;

    CameraHandle camera;
    ScalerHandle scaler;
    scaler_index_t scaler_idx;

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
    size_t src_stride = (size_t)cfg->frame_width * 3u;

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
        if (step == 1u) {
            memcpy(dst, row, (size_t)w * 3u);
            continue;
        }
        for (uint32_t x = 0; x < w; x++) {
            const uint8_t *p = row + (size_t)x * step * 3u;
            dst[0] = p[0];
            dst[1] = p[1];
            dst[2] = p[2];
            dst += 3;
        }
    }
    v->preview_w = w;
    v->preview_h = h;
    v->preview_ready = true;
}

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
    v->scaler_idx = SCALER_INDEX_0;

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

    /* --- 스케일러 --- */
    if (ScalerCreate(&v->scaler) != 0) {
        fprintf(stderr, "[vision] ScalerCreate 실패\n");
        goto fail;
    }
    if (ScalerOpenDevice(v->scaler, SCALER_DEV_NAME_0, v->scaler_idx) != 0) {
        fprintf(stderr, "[vision] ScalerOpenDevice(%s) 실패\n", SCALER_DEV_NAME_0);
        goto fail;
    }

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
    if (v->scaler != NULL) {
        ScalerCloseDevice(v->scaler, v->scaler_idx);
        ScalerDestroy(v->scaler);
    }
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
    scaler_size_info_t src, dst;
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
