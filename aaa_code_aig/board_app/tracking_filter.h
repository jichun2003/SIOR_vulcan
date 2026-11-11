/*
 * 지연 보상 + α-β 속도 추정 + 2자유도 제어.
 * aaa_code_1111/tracking_filter.py (PredictiveTracker)를 그대로 옮긴 것.
 *
 * 모든 각도는 "서보각 공간"에서 다룬다:
 *   표적 서보각 = (그 프레임이 찍힌 시점의 서보각) + sign * 화면 각도 오차
 *
 * 흐름 (축마다 독립):
 *   1) 시간 정렬   : 캡처 시점 = 처리 완료 시각 - L, 그 시점의 서보 명령각을 이력에서 보간
 *   2) α-β 필터    : 측정 표적각 -> [표적각 θ, 각속도 ω]
 *   3) 현재 예측   : θ̂(지금) = θ(캡처) + ω·L  (스미스 예측기)
 *   4) 2자유도 제어: 회전속도 = ω̂ (피드포워드) + PID(θ̂ - 서보각)
 *   5) 놓침 처리   : 예측 속도로 계속 회전, 시간이 지나면 서서히 정지
 */

#ifndef AIG_TRACKING_FILTER_H
#define AIG_TRACKING_FILTER_H

#include <stdbool.h>

#include "aig_config.h"
#include "turret.h"

#define AIG_SERVO_HISTORY_LEN 600

/* 1축 등속 모델 α-β 필터. 상태 = [각도 θ, 각속도 ω], 기준 시각 t */
typedef struct {
    float  alpha;
    float  beta;
    bool   ready;
    float  theta;
    float  omega;
    double t;
} aig_ab_filter_t;

/* 서보 명령각 이력 (시각, pan, tilt). 과거 시점 값을 선형 보간으로 꺼낸다 */
typedef struct {
    double t[AIG_SERVO_HISTORY_LEN];
    float  pan[AIG_SERVO_HISTORY_LEN];
    float  tilt[AIG_SERVO_HISTORY_LEN];
    int    head;  /* 가장 오래된 항목 위치 */
    int    count;
} aig_servo_history_t;

/* 한 프레임 처리 결과 (로그용) */
typedef struct {
    bool  has_meas;
    float meas_pan, meas_tilt;
    float est_pan, est_tilt;
    float omega_pan, omega_tilt;
    float pred_pan, pred_tilt;
    float resid_pan, resid_tilt;
    int   filter_reset;
    int   outlier;
    float ff_scale;
} aig_track_info_t;

typedef struct {
    const aig_config_t *cfg;
    aig_turret_t       *turret;
    aig_servo_history_t history;
    aig_ab_filter_t     f_pan;
    aig_ab_filter_t     f_tilt;
    bool   has_last_meas;
    double last_meas_time;
    bool   has_last_time;
    double last_time;
    bool   coast_done;
    int    outliers;
} aig_predictive_tracker_t;

void aig_tracker_init(aig_predictive_tracker_t *pt, const aig_config_t *cfg, aig_turret_t *turret);
void aig_tracker_reset(aig_predictive_tracker_t *pt);
/* 현재 서보각을 이력에 기록 (매 프레임 호출됨) */
void aig_tracker_record(aig_predictive_tracker_t *pt, double now);

/* 표적을 찾은 프레임: 화면 각도 오차(deg)를 넣으면 turret의 각도를 갱신한다 */
void aig_tracker_on_measurement(aig_predictive_tracker_t *pt, double now,
                                float err_pan_deg, float err_tilt_deg,
                                aig_track_info_t *out);

/* 표적이 없는 프레임: 예측만으로 회전하다가 서서히 멈춘다 */
void aig_tracker_on_lost(aig_predictive_tracker_t *pt, double now, aig_track_info_t *out);

#endif /* AIG_TRACKING_FILTER_H */
