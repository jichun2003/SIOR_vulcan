/*
 * 포탑 각도 상태 + 2축 추적 제어기. aaa_code_1111/turret_control.py의 TurretControl 포팅.
 *
 * pan_sign/tilt_sign("화면 오차 -> 서보각 증가 방향" 하드웨어 보정)은 여기 한 곳에서만
 * 적용한다 (update). update_2dof는 이미 서보각 공간의 값을 받으므로 부호 보정이 없다.
 */

#ifndef AIG_TURRET_H
#define AIG_TURRET_H

#include <stdbool.h>

#include "aig_config.h"
#include "pid.h"

typedef struct {
    const aig_config_t *cfg;
    aig_pid_t pid_pan;
    aig_pid_t pid_tilt;
    float pan_angle;
    float tilt_angle;
    int   lost_frames;
} aig_turret_t;

void aig_turret_init(aig_turret_t *t, const aig_config_t *cfg);

/* legacy: 화면 오차(표적-중심, deg)를 받아 서보각 갱신 (부호 보정 적용) */
void aig_turret_update(aig_turret_t *t, float pan_err_deg, float tilt_err_deg, double now);

/* 2dof: 오차와 피드포워드가 이미 서보각 공간. 회전속도 = ff(표적 각속도) + PID(예측각 - 서보각) */
void aig_turret_update_2dof(aig_turret_t *t, float pan_err_deg, float tilt_err_deg,
                            float pan_ff_deg_s, float tilt_ff_deg_s, double now);

/* 표적을 놓친 동안 예측 각속도로 계속 회전 (피드백 없음, 속도 상한 적용) */
void aig_turret_coast(aig_turret_t *t, float pan_rate_deg_s, float tilt_rate_deg_s, double dt);

/* 놓친 프레임 처리. lost_target_hold_frames에 도달해 제어기를 리셋했으면 true */
bool aig_turret_handle_lost(aig_turret_t *t);

void aig_turret_reset(aig_turret_t *t);

#endif /* AIG_TURRET_H */
