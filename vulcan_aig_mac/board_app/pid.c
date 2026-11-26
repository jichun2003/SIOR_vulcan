#include "pid.h"

#include <math.h>

float aig_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void aig_pid_init(aig_pid_t *p, float kp, float ki, float kd, float deadband_deg,
                  float max_rate_deg_s, float err_alpha, float d_alpha,
                  float integral_limit_deg_s)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->deadband = deadband_deg;
    p->max_rate = max_rate_deg_s;
    p->err_alpha = err_alpha;
    p->d_alpha = d_alpha;
    p->integral_limit = integral_limit_deg_s;
    aig_pid_reset(p);
}

void aig_pid_reset(aig_pid_t *p)
{
    p->has_err_f = false;
    p->err_f = 0.0f;
    p->has_prev_err_f = false;
    p->prev_err_f = 0.0f;
    p->deriv_f = 0.0f;
    p->integral = 0.0f;
    p->has_prev_time = false;
    p->prev_time = 0.0;
}

void aig_pid_clear_time(aig_pid_t *p)
{
    p->has_prev_time = false;
}

float aig_pid_update(aig_pid_t *p, float error, double now, float ff_rate)
{
    float dt;
    float e;
    float e_ctrl;
    float raw_d;
    float rate;
    bool  saturated;

    if (!p->has_prev_time) {
        dt = 1.0f / 30.0f;          /* 첫 프레임: 보통 프레임 간격으로 가정 */
    } else {
        dt = (float)(now - p->prev_time);
        /* 렉으로 dt가 튀어도 한 번에 크게 안 움직이게 */
        dt = aig_clampf(dt, 1e-3f, 0.2f);
    }
    p->prev_time = now;
    p->has_prev_time = true;

    /* 1) 오차 저역통과 필터 */
    if (!p->has_err_f) {
        p->err_f = error;
        p->has_err_f = true;
    } else {
        p->err_f = p->err_alpha * error + (1.0f - p->err_alpha) * p->err_f;
    }
    e = p->err_f;

    /* 2) 불감대 (경계에서 튀지 않게 불감대만큼 빼서 연속으로) */
    if (fabsf(e) <= p->deadband) {
        e_ctrl = 0.0f;
    } else {
        e_ctrl = e - copysignf(p->deadband, e);
    }

    /* 3) 미분 (필터된 오차 기준 + 한 번 더 필터) */
    if (!p->has_prev_err_f) {
        raw_d = 0.0f;
    } else {
        raw_d = (e - p->prev_err_f) / dt;
    }
    p->prev_err_f = e;
    p->has_prev_err_f = true;
    p->deriv_f = p->d_alpha * raw_d + (1.0f - p->d_alpha) * p->deriv_f;

    /* 4) 변화율 + 포화 */
    rate = p->kp * e_ctrl + p->ki * p->integral + p->kd * p->deriv_f + ff_rate;
    saturated = fabsf(rate) >= p->max_rate;
    rate = aig_clampf(rate, -p->max_rate, p->max_rate);

    /* 5) 적분 (불감대 안이거나 포화 중이면 쌓지 않음 = 안티 와인드업) */
    if (p->ki > 0.0f && e_ctrl != 0.0f && !saturated) {
        float lim;
        p->integral += e_ctrl * dt;
        lim = p->integral_limit / p->ki;
        p->integral = aig_clampf(p->integral, -lim, lim);
    }

    return rate * dt;
}
