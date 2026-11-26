/*
 * 제어 로직 닫힌 루프 시뮬레이션 (PC에서 실행).
 *
 * 카메라/NPU 없이 pid.c + turret.c + tracking_filter.c만 가지고,
 *   표적 각도(사인파) -> 지연 L 후 측정 -> α-β 예측 -> 2자유도 PID -> 서보(슬루 제한)
 * 루프를 돌려서 추종 오차가 수렴하는지 본다. 파이썬(1111)에서 검증된 동작이 C 포팅
 * 후에도 같은지 확인하는 용도다.
 *
 * 실패 조건(종료 코드 1):
 *   - 움직이는 표적에서 정상상태 RMS 오차가 임계값을 넘음 (= 발산/진동/과소보상)
 *   - 정지 표적에서 오차가 불감대보다 크게 남음
 *   - 표적을 놓친 뒤 coast_max_s가 지나도 서보가 계속 움직임
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aig_config.h"
#include "tracking_filter.h"
#include "turret.h"

#define SIM_HZ     60.0
#define SIM_DT     (1.0 / SIM_HZ)
#define HIST_LEN   4096
#define SERVO_SLEW 66.0f   /* 펌웨어 슬루레이트 (deg/s) */

typedef struct {
    double target[HIST_LEN];   /* 표적의 "서보각 공간" 각도 */
    double servo[HIST_LEN];    /* 서보 실제 각도 */
    int    n;
} sim_hist_t;

static double hist_get(const double *buf, int n, int steps_back)
{
    int idx = n - 1 - steps_back;
    if (idx < 0) {
        idx = 0;
    }
    return buf[idx];
}

/* 표적 각도: 중앙 ± amp 사인파 (도르래 표적이 좌우로 왕복하는 상황) */
static double target_angle(const aig_config_t *cfg, double t, double amp, double period)
{
    if (period <= 0.0) {
        return cfg->pan_center_deg + amp;   /* 정지 표적 */
    }
    return cfg->pan_center_deg + amp * sin(2.0 * M_PI * t / period);
}

typedef struct {
    double rms;
    double max;
    double final_servo_rate;   /* 마지막 구간의 서보 각속도 크기 (coast 확인용) */
} sim_result_t;

/*
 * amp/period : 표적 움직임
 * true_lag_s : 실제 지연 (측정이 이만큼 과거 상태를 반영)
 * lose_at_s  : 이 시각 이후 표적을 놓친 것으로 처리 (<0이면 계속 보임)
 * settle_s   : 이 시각 이후부터 오차를 집계
 */
static sim_result_t run_sim(const aig_config_t *cfg, double amp, double period,
                            double true_lag_s, double lose_at_s, double settle_s,
                            double duration_s)
{
    aig_turret_t turret;
    aig_predictive_tracker_t tracker;
    sim_hist_t h;
    sim_result_t res;
    int lag_steps = (int)(true_lag_s * SIM_HZ + 0.5);
    double sum_sq = 0.0;
    int    n_err = 0;
    double max_abs = 0.0;
    double prev_servo = 0.0, last_rate_sum = 0.0;
    int    last_rate_n = 0;

    memset(&h, 0, sizeof(h));
    memset(&res, 0, sizeof(res));
    aig_turret_init(&turret, cfg);
    aig_tracker_init(&tracker, cfg, &turret);

    /* 이력 초기화: 시작 시 서보는 중앙, 표적은 t=0 위치 */
    h.target[0] = target_angle(cfg, 0.0, amp, period);
    h.servo[0] = turret.pan_angle;
    h.n = 1;
    prev_servo = turret.pan_angle;

    for (int step = 1; step < (int)(duration_s * SIM_HZ) && h.n < HIST_LEN; step++) {
        double t = step * SIM_DT;
        double tgt_now = target_angle(cfg, t, amp, period);
        double meas_target = hist_get(h.target, h.n, lag_steps);
        double meas_servo = hist_get(h.servo, h.n, lag_steps);
        aig_track_info_t info;
        bool visible = (lose_at_s < 0.0) || (t < lose_at_s);
        double cmd, servo_now, delta, err;

        /* 화면 각도 오차 = (표적 각도 - 그때의 서보각) / pan_sign
         * (tracking_filter가 meas = servo_cap + pan_sign*err로 복원하므로 역산) */
        if (visible) {
            double err_deg = (meas_target - meas_servo) / cfg->pan_sign;
            aig_tracker_on_measurement(&tracker, t, (float)err_deg, 0.0f, &info);
        } else {
            aig_tracker_on_lost(&tracker, t, &info);
        }

        /* 서보: 명령각을 향해 슬루레이트 제한으로 이동 */
        cmd = turret.pan_angle;
        servo_now = hist_get(h.servo, h.n, 0);
        delta = cmd - servo_now;
        if (delta > SERVO_SLEW * SIM_DT) {
            delta = SERVO_SLEW * SIM_DT;
        }
        if (delta < -SERVO_SLEW * SIM_DT) {
            delta = -SERVO_SLEW * SIM_DT;
        }
        servo_now += delta;

        h.target[h.n] = tgt_now;
        h.servo[h.n] = servo_now;
        h.n++;

        if (t >= settle_s && visible) {
            err = tgt_now - servo_now;
            sum_sq += err * err;
            n_err++;
            if (fabs(err) > max_abs) {
                max_abs = fabs(err);
            }
        }
        if (!visible && t >= lose_at_s + cfg->coast_max_s + 0.2) {
            last_rate_sum += fabs(servo_now - prev_servo) / SIM_DT;
            last_rate_n++;
        }
        prev_servo = servo_now;
    }

    res.rms = n_err > 0 ? sqrt(sum_sq / n_err) : 0.0;
    res.max = max_abs;
    res.final_servo_rate = last_rate_n > 0 ? last_rate_sum / last_rate_n : 0.0;
    return res;
}

int main(void)
{
    aig_config_t cfg = aig_config_default();
    sim_result_t r;
    int failures = 0;

    /* NPU 기준 지연으로 가정: 추론 15ms + 프레임 33ms + 서보 반응 -> 대략 0.08s.
     * 실제 값은 보드에서 --sweep으로 재측정해서 넣어야 한다. */
    cfg.latency_s = 0.08f;
    cfg.latency_tilt_s = 0.08f;

    printf("=== 1) 움직이는 표적 (±25도, 주기 4초), 지연 0.08s 일치 ===\n");
    r = run_sim(&cfg, 25.0, 4.0, 0.08, -1.0, 3.0, 20.0);
    printf("    정상상태 오차 RMS %.3f deg, 최대 %.3f deg\n", r.rms, r.max);
    if (r.rms > 3.0 || r.max > 8.0) {
        printf("    !! 실패: 추종 오차가 너무 큽니다\n");
        failures++;
    }

    printf("=== 2) 같은 표적, 지연을 0.25s로 잘못 넣은 경우 (과보상) ===\n");
    {
        aig_config_t bad = cfg;
        bad.latency_s = 0.25f;
        bad.latency_tilt_s = 0.25f;
        r = run_sim(&bad, 25.0, 4.0, 0.08, -1.0, 3.0, 20.0);
        printf("    정상상태 오차 RMS %.3f deg, 최대 %.3f deg\n", r.rms, r.max);
        printf("    (지연을 틀리게 넣으면 오차가 커지는지 확인용 - 실패 판정은 하지 않음)\n");
    }

    printf("=== 3) 정지 표적 (중앙에서 10도 떨어진 곳) ===\n");
    r = run_sim(&cfg, 10.0, 0.0, 0.08, -1.0, 3.0, 10.0);
    printf("    정상상태 오차 RMS %.3f deg, 최대 %.3f deg\n", r.rms, r.max);
    if (r.rms > cfg.deadband_deg + 0.6f) {
        printf("    !! 실패: 정지 표적에 수렴하지 않습니다 (불감대 %.2f deg)\n", cfg.deadband_deg);
        failures++;
    }

    printf("=== 4) 5초에 표적 상실 -> coast 후 정지하는지 ===\n");
    r = run_sim(&cfg, 25.0, 4.0, 0.08, 5.0, 3.0, 10.0);
    printf("    coast_max(%.1fs) 경과 후 서보 각속도 %.3f deg/s\n",
           cfg.coast_max_s, r.final_servo_rate);
    if (r.final_servo_rate > 1.0) {
        printf("    !! 실패: 표적을 놓친 뒤에도 서보가 계속 움직입니다\n");
        failures++;
    }

    printf("\n%s (실패 %d개)\n", failures == 0 ? "모두 통과" : "실패 있음", failures);
    return failures == 0 ? 0 : 1;
}
