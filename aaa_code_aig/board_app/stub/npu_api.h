/*
 * Telechips npu_api.h의 최소 가짜 선언 (PC에서 제어 로직만 시험하기 위한 스텁).
 * 실제 보드/SDK에서는 SDK가 제공하는 npu_api.h가 쓰인다 — 이 폴더는 -DAIG_STUB_NPU일 때만
 * include 경로에 들어간다.
 *
 * 구조체/시그니처는 공개 저장소(topst-development/libtc-ndnpu, develop 브랜치)의
 * lib/include/npu_api.h를 그대로 따랐다. 바꾸면 보드에서 안 맞게 되므로 주의.
 */

#ifndef AIG_STUB_NPU_API_H
#define AIG_STUB_NPU_API_H

typedef struct { int fd; } npu_t;

typedef struct {
    int fd;
    int size;
    char *caddr;
    unsigned long paddr;
} npu_buf_t;

struct enlight_net;
typedef struct {
    int fd;
    struct enlight_net *methods;
    void *dl;
} npu_net_t;

typedef struct {
    unsigned int elapsed_in_us;
    unsigned int dma;
    unsigned int comp;
    unsigned int all;
} npu_perf_t;

typedef union { unsigned int bits; } npu_err_bits_t;

typedef enum { NPU_RUN_SYNC = 0x0, NPU_RUN_ASYNC = 0x1 } npu_run_mode_t;

enum { NPU_COLOR_RGB = 0, NPU_COLOR_YUV = 1 };

typedef struct {
    float   x_min;
    float   x_max;
    float   y_min;
    float   y_max;
    short   cls;
    float   score;
    short   img_w;
    short   img_h;
} enlight_obj_t;

typedef struct {
    int cnt;
    enlight_obj_t obj[256];
} enlight_objs_t;

typedef enum {
    TELECHIPS_NPU_POST_NONE,
    TELECHIPS_NPU_POST_CLASSIFIER,
    TELECHIPS_NPU_POST_DETECTOR,
    TELECHIPS_NPU_POST_CUSTOM
} enlight_postproc_t;

npu_t     *npu_open(int minor);
int        npu_close(npu_t *npu);
npu_net_t *network_load_from_file(npu_t *npu, char *sofile, char *cmdfile, char *paramfile);
int        network_close(npu_net_t *net);
int        network_set_color_format(npu_net_t *net, int color_format);
int        network_get_input_width(npu_net_t *net);
int        network_get_input_height(npu_net_t *net);
int        network_get_output_size(npu_net_t *net);
int        network_get_type(npu_net_t *net);
npu_buf_t *buffer_alloc(npu_t *npu, int size);
char      *buffer_get_addr(npu_buf_t *buf);
int        buffer_close(npu_buf_t *buf);
int        network_run(npu_net_t *net, npu_buf_t *in, npu_buf_t *out,
                       npu_err_bits_t *status, npu_run_mode_t mode,
                       npu_perf_t *perf, int timeout_in_ms);
int        network_run_postprocess(npu_net_t *net, npu_buf_t *out, void *result);

#endif /* AIG_STUB_NPU_API_H */
