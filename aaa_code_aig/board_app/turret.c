#include "turret.h"

static void clamp_angles(aig_turret_t *t)
{
    t->pan_angle = aig_clampf(t->pan_angle, t->cfg->pan_min_deg, t->cfg->pan_max_deg);
    t->tilt_angle = aig_clampf(t->tilt_angle, t->cfg->tilt_min_deg, t->cfg->tilt_max_deg);
}

void aig_turret_init(aig_turret_t *t, const aig_config_t *cfg)
{
    t->cfg = cfg;
    aig_pid_init(&t->pid_pan, cfg->kp, cfg->ki, cfg->kd, cfg->deadband_deg,
                 cfg->max_rate_deg_s, cfg->error_filter_alpha, cfg->d_filter_alpha,
                 cfg->integral_limit_deg_s);
    aig_pid_init(&t->pid_tilt, cfg->kp, cfg->ki, cfg->kd, cfg->deadband_deg,
                 cfg->max_rate_deg_s, cfg->error_filter_alpha, cfg->d_filter_alpha,
                 cfg->integral_limit_deg_s);
    t->pan_angle = cfg->pan_center_deg;
    t->tilt_angle = cfg->tilt_center_deg;
    t->lost_frames = 0;
}

void aig_turret_update(aig_turret_t *t, float pan_err_deg, float tilt_err_deg, double now)
{
    t->pan_angle += t->cfg->pan_sign * aig_pid_update(&t->pid_pan, pan_err_deg, now, 0.0f);
    t->tilt_angle += t->cfg->tilt_sign * aig_pid_update(&t->pid_tilt, tilt_err_deg, now, 0.0f);
    clamp_angles(t);
    t->lost_frames = 0;
}

void aig_turret_update_2dof(aig_turret_t *t, float pan_err_deg, float tilt_err_deg,
                            float pan_ff_deg_s, float tilt_ff_deg_s, double now)
{
    t->pan_angle += aig_pid_update(&t->pid_pan, pan_err_deg, now, pan_ff_deg_s);
    t->tilt_angle += aig_pid_update(&t->pid_tilt, tilt_err_deg, now, tilt_ff_deg_s);
    clamp_angles(t);
    t->lost_frames = 0;
}

void aig_turret_coast(aig_turret_t *t, float pan_rate_deg_s, float tilt_rate_deg_s, double dt)
{
    float lim = t->cfg->max_rate_deg_s;
    t->pan_angle += aig_clampf(pan_rate_deg_s, -lim, lim) * (float)dt;
    t->tilt_angle += aig_clampf(tilt_rate_deg_s, -lim, lim) * (float)dt;
    clamp_angles(t);
}

bool aig_turret_handle_lost(aig_turret_t *t)
{
    t->lost_frames++;
    /* 한 프레임이라도 놓치면 시간 간격이 끊기므로 다음 프레임 dt가 커지지 않게 시간만 초기화 */
    aig_pid_clear_time(&t->pid_pan);
    aig_pid_clear_time(&t->pid_tilt);
    if (t->lost_frames == t->cfg->lost_target_hold_frames) {
        aig_pid_reset(&t->pid_pan);
        aig_pid_reset(&t->pid_tilt);
        return true;
    }
    return false;
}

void aig_turret_reset(aig_turret_t *t)
{
    t->pan_angle = t->cfg->pan_center_deg;
    t->tilt_angle = t->cfg->tilt_center_deg;
    aig_pid_reset(&t->pid_pan);
    aig_pid_reset(&t->pid_tilt);
    t->lost_frames = 0;
}
