/*
 * 카메라 -> 하드웨어 스케일러 -> NPU 추론 -> 표적 하나 선택.
 * aaa_code_1111/vision.py (Camera + Detector + pick_target)에 대응.
 *
 * Telechips API 사용 흐름 (tc-nn-app/src/NnNeuralNetwork.c, NnAppMain.c 기준):
 *   npu_open -> network_load_from_file(net.so, npu_cmd.bin, quantized_network.bin)
 *   -> network_get_input_width/height -> buffer_alloc(입력/출력)
 *   -> [매 프레임] CameraGetBuffer -> ScalerResize(카메라 물리주소 -> inputBuf->paddr)
 *      -> network_run(SYNC) -> network_run_postprocess(-> enlight_objs_t)
 *
 * 스케일러가 색변환+리사이즈를 하드웨어로 처리하므로 CPU memcpy가 없다.
 */

#ifndef AIG_NPU_VISION_H
#define AIG_NPU_VISION_H

#include <stdbool.h>
#include <stdint.h>

#include "aig_config.h"

#define AIG_MAX_DETS 64

/* 원본 프레임 픽셀 좌표계의 탐지 결과 */
typedef struct {
    float x1, y1, x2, y2;
    float cx, cy;
    float w, h;
    float conf;
    int   cls;
} aig_det_t;

typedef struct aig_vision aig_vision_t;

aig_vision_t *aig_vision_open(const aig_config_t *cfg);
void          aig_vision_close(aig_vision_t *v);

/* 한 프레임 캡처 + 추론.
 * dets에 conf 임계값을 넘은 탐지를 최대 max_dets개 담고, 개수를 *n_dets에 넣는다.
 * infer_ms가 NULL이 아니면 NPU 추론 시간(ms)을 넣는다.
 * 반환: 0 성공, -1 실패 */
int aig_vision_capture_detect(aig_vision_t *v, aig_det_t *dets, int max_dets,
                              int *n_dets, float *infer_ms);

/* NPU 입력 해상도 (로그/디버그용) */
void aig_vision_input_size(const aig_vision_t *v, uint32_t *w, uint32_t *h);

/* 프레임에서 표적 하나를 고른다 (1111 vision.py pick_target과 동일 규칙).
 *   - target_class >= 0이면 그 클래스만 후보
 *   - last_center가 NULL이 아니고 max_jump_px > 0이면, 직전 위치에서 max_jump_px보다
 *     멀리 떨어진 후보는 (가까운 후보가 하나라도 있으면) 제외
 *   - 남은 후보 중 최고 신뢰도
 * 반환: dets 안의 인덱스, 없으면 -1 */
int aig_pick_target(const aig_det_t *dets, int n, int target_class,
                    const float *last_center, float max_jump_px);

#endif /* AIG_NPU_VISION_H */
