/*
 * 사격 통제: 2단계 신호 상태머신. aaa_code_1111/fire_control.py 포팅.
 *
 *   TRACK  --신호1--> LEAD --신호2--> FIRING --시간종료--> TRACK
 *   취소(cancel)는 어느 상태에서든 TRACK으로.
 *
 * 선도각(리드샷) 계산은 포팅하지 않았다. 1111도 --ballistic-json이 없으면 선도각 0으로
 * 동작하고, 탄도 모델(ballistic/lead_solver.py)은 궤적표를 만드는 수치해석이라 보드로
 * 옮기려면 별도 작업이 필요하다. 지금은 1111에서 ballistic_json=None일 때와 동일하게
 * "신호 2번 -> n초 사격" 흐름만 수행한다.
 */

#ifndef AIG_FIRE_H
#define AIG_FIRE_H

#include <stdbool.h>

#include "aig_config.h"

typedef enum {
    AIG_FIRE_TRACK = 0,
    AIG_FIRE_LEAD,
    AIG_FIRE_FIRING
} aig_fire_state_t;

typedef enum {
    AIG_FIRE_EVENT_NONE = 0,
    AIG_FIRE_EVENT_LEAD,
    AIG_FIRE_EVENT_FIRE
} aig_fire_event_t;

typedef struct {
    const aig_config_t *cfg;
    aig_fire_state_t    state;
    double              fire_end;
} aig_fire_t;

void aig_fire_init(aig_fire_t *f, const aig_config_t *cfg);

/* 신호 1회 처리. LEAD 장입 / 사격 시작 / 무시 중 하나를 반환 */
aig_fire_event_t aig_fire_signal(aig_fire_t *f, double now);

/* 선도각 해제 + 사격 중지. 사격 중이었으면 true */
bool aig_fire_cancel(aig_fire_t *f);

/* 사격 시간이 끝났으면 TRACK으로 돌아가며 true */
bool aig_fire_update(aig_fire_t *f, double now);

bool  aig_fire_is_firing(const aig_fire_t *f);
float aig_fire_remaining(const aig_fire_t *f, double now);
const char *aig_fire_state_name(const aig_fire_t *f);

#endif /* AIG_FIRE_H */
