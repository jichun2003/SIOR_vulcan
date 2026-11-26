#include "tracking_filter.h"

#include <math.h>
#include <string.h>

/* |x| <= db면 0, 그 밖은 db만큼 빼서 연속으로
 * (정지 표적에서 속도 추정 노이즈가 서보를 흔들지 않게) */
static float soft_deadband(float x, float db)
{
    if (fabsf(x) <= db) {
        return 0.0f;
    }
    return x - copysignf(db, x);
}

/* ---------------- α-β 필터 ---------------- */

static void ab_init(aig_ab_filter_t *f, float alpha, float beta)
{
    f->alpha = alpha;
    f->beta = beta;
    f->ready = false;
    f->theta = 0.0f;
    f->omega = 0.0f;
    f->t = 0.0;
}

static void ab_reset(aig_ab_filter_t *f)
{
    f->ready = false;
    f->theta = 0.0f;
    f->omega = 0.0f;
    f->t = 0.0;
}

static float ab_value_at(const aig_ab_filter_t *f, double t)
{
    return f->theta + f->omega * (float)(t - f->t);
}

/* 측정 없이 t까지 등속으로 진행 (놓침 구간) */
static void ab_predict_to(aig_ab_filter_t *f, double t)
{
    if (!f->ready || t <= f->t) {
        return;
    }
    f->theta += f->omega * (float)(t - f->t);
    f->t = t;
}

/* 측정 - 예측 (상태는 바꾸지 않음). 이상치 판정용 */
static float ab_residual(const aig_ab_filter_t *f, float meas, double t)
{
    return meas - ab_value_at(f, t);
}

/* 측정 반영. 잔차를 반환하고, 필터가 처음부터 다시 시작했으면 *was_reset = true */
static float ab_update(aig_ab_filter_t *f, float meas, double t, bool *was_reset)
{
    float dt;
    float theta_pred;
    float r;

    *was_reset = false;
    if (!f->ready) {
        f->theta = meas;
        f->omega = 0.0f;
        f->t = t;
        f->ready = true;
        *was_reset = true;
        return 0.0f;
    }

    dt = (float)(t - f->t);
    if (dt <= 1e-4f) {
        dt = 1e-4f;
    }
    theta_pred = f->theta + f->omega * dt;
    r = meas - theta_pred;

    f->theta = theta_pred + f->alpha * r;
    f->omega = f->omega + (f->beta / dt) * r;
    f->t = t;
    return r;
}

/* ---------------- 서보 명령각 이력 ---------------- */

static void hist_init(aig_servo_history_t *h)
{
    h->head = 0;
    h->count = 0;
}

static int hist_idx(const aig_servo_history_t *h, int i)
{
    return (h->head + i) % AIG_SERVO_HISTORY_LEN;
}

static void hist_add(aig_servo_history_t *h, double t, float pan, float tilt)
{
    int last;

    if (h->count > 0) {
        last = hist_idx(h, h->count - 1);
        if (t <= h->t[last]) {
            /* 같은(또는 거꾸로 간) 시각이면 마지막 값만 갱신 */
            h->pan[last] = pan;
            h->tilt[last] = tilt;
            return;
        }
    }

    if (h->count < AIG_SERVO_HISTORY_LEN) {
        int slot = hist_idx(h, h->count);
        h->t[slot] = t;
        h->pan[slot] = pan;
        h->tilt[slot] = tilt;
        h->count++;
    } else {
        /* 가득 찬 경우: 가장 오래된 항목을 덮어쓰고 head를 민다 */
        h->t[h->head] = t;
        h->pan[h->head] = pan;
        h->tilt[h->head] = tilt;
        h->head = (h->head + 1) % AIG_SERVO_HISTORY_LEN;
    }
}

/* t 시점의 서보각을 선형 보간으로 꺼낸다. 이력이 없으면 false */
static bool hist_at(const aig_servo_history_t *h, double t, float *pan, float *tilt)
{
    int lo, hi, i0, i1;
    double t0, t1, k;

    if (h->count == 0) {
        return false;
    }
    i0 = hist_idx(h, 0);
    if (t <= h->t[i0]) {
        *pan = h->pan[i0];
        *tilt = h->tilt[i0];
        return true;
    }
    i1 = hist_idx(h, h->count - 1);
    if (t >= h->t[i1]) {
        *pan = h->pan[i1];
        *tilt = h->tilt[i1];
        return true;
    }

    /* t[lo] <= t < t[lo+1]이 되는 lo를 이분 탐색 */
    lo = 0;
    hi = h->count - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (h->t[hist_idx(h, mid)] <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    i0 = hist_idx(h, lo);
    i1 = hist_idx(h, lo + 1);
    t0 = h->t[i0];
    t1 = h->t[i1];
    k = (t1 - t0) > 0.0 ? (t - t0) / (t1 - t0) : 0.0;
    *pan = h->pan[i0] + (float)k * (h->pan[i1] - h->pan[i0]);
    *tilt = h->tilt[i0] + (float)k * (h->tilt[i1] - h->tilt[i0]);
    return true;
}

/* ---------------- 예측 추적기 ---------------- */

void aig_tracker_init(aig_predictive_tracker_t *pt, const aig_config_t *cfg, aig_turret_t *turret)
{
    pt->cfg = cfg;
    pt->turret = turret;
    hist_init(&pt->history);
    ab_init(&pt->f_pan, cfg->ab_alpha, cfg->ab_beta);
    ab_init(&pt->f_tilt, cfg->ab_alpha, cfg->ab_beta);
    pt->has_last_meas = false;
    pt->last_meas_time = 0.0;
    pt->has_last_time = false;
    pt->last_time = 0.0;
    pt->coast_done = false;
    pt->outliers = 0;
    aig_tracker_record(pt, 0.0);
}

void aig_tracker_reset(aig_predictive_tracker_t *pt)
{
    ab_reset(&pt->f_pan);
    ab_reset(&pt->f_tilt);
    pt->has_last_meas = false;
    pt->coast_done = false;
    pt->outliers = 0;
}

void aig_tracker_record(aig_predictive_tracker_t *pt, double now)
{
    hist_add(&pt->history, now, pt->turret->pan_angle, pt->turret->tilt_angle);
}

/* 축별 캡처 시점. tilt는 포신 하중 때문에 지연이 다를 수 있어 따로 둔다 */
static void cap_times(const aig_config_t *cfg, double now, double *tp, double *tt)
{
    double lp = cfg->latency_s;
    double lt = (cfg->latency_tilt_s >= 0.0f) ? cfg->latency_tilt_s : cfg->latency_s;
    *tp = now - lp;
    *tt = now - lt;
}

static void info_clear(aig_track_info_t *out)
{
    memset(out, 0, sizeof(*out));
}

/* 이상치 프레임: 측정을 버리고 예측 속도로 한 프레임 진행 */
static void coast_outlier(aig_predictive_tracker_t *pt, double now, double tp, double tt,
                          float meas_pan, float meas_tilt, float r_pan, float r_tilt,
                          aig_track_info_t *out)
{
    double dt = pt->has_last_time ? (now - pt->last_time) : 0.0;

    if (dt < 0.0) {
        dt = 0.0;
    }
    if (dt > 0.2) {
        dt = 0.2;
    }
    pt->last_time = now;
    pt->has_last_time = true;

    ab_predict_to(&pt->f_pan, tp);
    ab_predict_to(&pt->f_tilt, tt);
    aig_turret_coast(pt->turret, pt->f_pan.omega, pt->f_tilt.omega, dt);
    aig_tracker_record(pt, now);

    info_clear(out);
    out->has_meas = true;
    out->meas_pan = meas_pan;
    out->meas_tilt = meas_tilt;
    out->omega_pan = pt->f_pan.omega;
    out->omega_tilt = pt->f_tilt.omega;
    out->pred_pan = ab_value_at(&pt->f_pan, now);
    out->pred_tilt = ab_value_at(&pt->f_tilt, now);
    out->resid_pan = r_pan;
    out->resid_tilt = r_tilt;
    out->outlier = 1;
    out->ff_scale = 1.0f;
}

void aig_tracker_on_measurement(aig_predictive_tracker_t *pt, double now,
                                float err_pan_deg, float err_tilt_deg,
                                aig_track_info_t *out)
{
    const aig_config_t *cfg = pt->cfg;
    double tp, tt;
    float servo_cap_pan, servo_cap_tilt;
    float meas_pan, meas_tilt;
    float r_pan = 0.0f, r_tilt = 0.0f;
    float w_pan, w_tilt, pred_pan, pred_tilt, e_pan, e_tilt;
    bool filter_reset = false;
    bool reset_pan = false, reset_tilt = false;

    cap_times(cfg, now, &tp, &tt);
    if (!hist_at(&pt->history, tp, &servo_cap_pan, &servo_cap_tilt)) {
        servo_cap_pan = pt->turret->pan_angle;
        servo_cap_tilt = pt->turret->tilt_angle;
    }
    {   /* tilt는 다른 시점에서 보간 */
        float p, t;
        if (hist_at(&pt->history, tt, &p, &t)) {
            servo_cap_tilt = t;
        }
    }

    /* 1) 시간 정렬된 측정 표적각 (서보각 공간) */
    meas_pan = servo_cap_pan + cfg->pan_sign * err_pan_deg;
    meas_tilt = servo_cap_tilt + cfg->tilt_sign * err_tilt_deg;

    /* 이상치 게이트: 예측과 크게 다르면 한두 프레임은 무시하고 예측대로 계속 돈다.
     * reacquire_frames 프레임 연속이면 진짜로 표적이 바뀐 것으로 보고 필터를 다시 시작. */
    if (pt->f_pan.ready) {
        r_pan = ab_residual(&pt->f_pan, meas_pan, tp);
        r_tilt = ab_residual(&pt->f_tilt, meas_tilt, tt);
        if (fmaxf(fabsf(r_pan), fabsf(r_tilt)) > cfg->reacquire_gate_deg) {
            pt->outliers++;
            if (pt->outliers < cfg->reacquire_frames) {
                coast_outlier(pt, now, tp, tt, meas_pan, meas_tilt, r_pan, r_tilt, out);
                return;
            }
            ab_reset(&pt->f_pan);
            ab_reset(&pt->f_tilt);
            filter_reset = true;
        }
        pt->outliers = 0;
    }

    /* 2) α-β 필터 (필터 시간축 = 캡처 시점) */
    r_pan = ab_update(&pt->f_pan, meas_pan, tp, &reset_pan);
    r_tilt = ab_update(&pt->f_tilt, meas_tilt, tt, &reset_tilt);
    if (reset_pan && reset_tilt && pt->has_last_meas) {
        filter_reset = true;
    }

    /* 3) 현재 시점 예측 (작은 속도 추정값은 노이즈로 보고 0 처리) */
    w_pan = soft_deadband(pt->f_pan.omega, cfg->ff_deadband_deg_s);
    w_tilt = soft_deadband(pt->f_tilt.omega, cfg->ff_deadband_deg_s);
    pred_pan = pt->f_pan.theta + w_pan * (float)(now - pt->f_pan.t);
    pred_tilt = pt->f_tilt.theta + w_tilt * (float)(now - pt->f_tilt.t);

    /* 4) 2자유도 제어: FF(ω̂) + PID(예측 표적각 - 현재 서보각) */
    e_pan = pred_pan - pt->turret->pan_angle;
    e_tilt = pred_tilt - pt->turret->tilt_angle;
    aig_turret_update_2dof(pt->turret, e_pan, e_tilt, w_pan, w_tilt, now);

    pt->last_meas_time = now;
    pt->has_last_meas = true;
    pt->last_time = now;
    pt->has_last_time = true;
    pt->coast_done = false;
    aig_tracker_record(pt, now);

    info_clear(out);
    out->has_meas = true;
    out->meas_pan = meas_pan;
    out->meas_tilt = meas_tilt;
    out->est_pan = pt->f_pan.theta;
    out->est_tilt = pt->f_tilt.theta;
    out->omega_pan = w_pan;
    out->omega_tilt = w_tilt;
    out->pred_pan = pred_pan;
    out->pred_tilt = pred_tilt;
    out->resid_pan = r_pan;
    out->resid_tilt = r_tilt;
    out->filter_reset = filter_reset ? 1 : 0;
    out->ff_scale = 1.0f;
}

void aig_tracker_on_lost(aig_predictive_tracker_t *pt, double now, aig_track_info_t *out)
{
    const aig_config_t *cfg = pt->cfg;
    double dt = pt->has_last_time ? (now - pt->last_time) : 0.0;
    double lost_s;
    float scale, w_pan, w_tilt;
    double tp, tt;

    if (dt < 0.0) {
        dt = 0.0;
    }
    if (dt > 0.2) {
        dt = 0.2;
    }
    pt->last_time = now;
    pt->has_last_time = true;

    info_clear(out);
    out->ff_scale = 0.0f;

    if (!pt->has_last_meas || !pt->f_pan.ready || pt->coast_done) {
        aig_tracker_record(pt, now);
        return;
    }

    lost_s = now - pt->last_meas_time;
    if (lost_s >= cfg->coast_max_s) {
        aig_tracker_reset(pt);
        pt->coast_done = true;
        aig_tracker_record(pt, now);
        return;
    }

    if (lost_s <= cfg->coast_hold_s) {
        scale = 1.0f;
    } else {
        scale = 1.0f - (float)((lost_s - cfg->coast_hold_s) /
                               (cfg->coast_max_s - cfg->coast_hold_s));
    }

    cap_times(cfg, now, &tp, &tt);
    ab_predict_to(&pt->f_pan, tp);
    ab_predict_to(&pt->f_tilt, tt);
    w_pan = pt->f_pan.omega * scale;
    w_tilt = pt->f_tilt.omega * scale;
    aig_turret_coast(pt->turret, w_pan, w_tilt, dt);
    aig_tracker_record(pt, now);

    out->omega_pan = pt->f_pan.omega;
    out->omega_tilt = pt->f_tilt.omega;
    out->pred_pan = ab_value_at(&pt->f_pan, now);
    out->pred_tilt = ab_value_at(&pt->f_tilt, now);
    out->ff_scale = scale;
}
