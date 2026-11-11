/*
 * 속도형 PID. aaa_code_1111/turret_control.py의 RatePID를 그대로 옮긴 것.
 *
 * 화면 각도 오차(deg) -> 서보각 "변화량"(deg)을 낸다.
 *   회전속도[deg/s] = Kp*e + Ki*∫e + Kd*de/dt + ff_rate,  반환값 = 회전속도 * dt
 * 게인이 "프레임당"이 아니라 "초당"이라 FPS가 바뀌어도 반응 속도가 같다.
 */

#ifndef AIG_PID_H
#define AIG_PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float deadband;
    float max_rate;
    float err_alpha;        /* 1이면 필터 없음 */
    float d_alpha;
    float integral_limit;   /* deg/s 단위 상한 */

    /* 상태 */
    bool  has_err_f;
    float err_f;
    bool  has_prev_err_f;
    float prev_err_f;
    float deriv_f;
    float integral;
    bool  has_prev_time;
    double prev_time;
} aig_pid_t;

void  aig_pid_init(aig_pid_t *p, float kp, float ki, float kd, float deadband_deg,
                   float max_rate_deg_s, float err_alpha, float d_alpha,
                   float integral_limit_deg_s);
void  aig_pid_reset(aig_pid_t *p);
/* 시간 기준만 끊는다 (한 프레임 놓쳤을 때 dt가 커지지 않게) */
void  aig_pid_clear_time(aig_pid_t *p);
/* error: 각도 오차(deg), now: 단조 시계(초), ff_rate: 피드포워드 회전속도(deg/s).
 * 반환: 이번 프레임에 더할 서보각 변화량(deg). */
float aig_pid_update(aig_pid_t *p, float error, double now, float ff_rate);

float aig_clampf(float v, float lo, float hi);

#endif /* AIG_PID_H */
