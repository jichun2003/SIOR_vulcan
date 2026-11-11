#include "fire.h"

void aig_fire_init(aig_fire_t *f, const aig_config_t *cfg)
{
    f->cfg = cfg;
    f->state = AIG_FIRE_TRACK;
    f->fire_end = 0.0;
}

aig_fire_event_t aig_fire_signal(aig_fire_t *f, double now)
{
    if (f->state == AIG_FIRE_TRACK) {
        f->state = AIG_FIRE_LEAD;
        return AIG_FIRE_EVENT_LEAD;
    }
    if (f->state == AIG_FIRE_LEAD) {
        f->state = AIG_FIRE_FIRING;
        f->fire_end = now + f->cfg->fire_duration_s;
        return AIG_FIRE_EVENT_FIRE;
    }
    return AIG_FIRE_EVENT_NONE;   /* 사격 중 신호는 무시 */
}

bool aig_fire_cancel(aig_fire_t *f)
{
    bool was_firing = (f->state == AIG_FIRE_FIRING);
    f->state = AIG_FIRE_TRACK;
    return was_firing;
}

bool aig_fire_update(aig_fire_t *f, double now)
{
    if (f->state == AIG_FIRE_FIRING && now >= f->fire_end) {
        f->state = AIG_FIRE_TRACK;
        return true;
    }
    return false;
}

bool aig_fire_is_firing(const aig_fire_t *f)
{
    return f->state == AIG_FIRE_FIRING;
}

float aig_fire_remaining(const aig_fire_t *f, double now)
{
    double left;

    if (f->state != AIG_FIRE_FIRING) {
        return 0.0f;
    }
    left = f->fire_end - now;
    return left > 0.0 ? (float)left : 0.0f;
}

const char *aig_fire_state_name(const aig_fire_t *f)
{
    switch (f->state) {
    case AIG_FIRE_LEAD:   return "LEAD";
    case AIG_FIRE_FIRING: return "FIRING";
    case AIG_FIRE_TRACK:
    default:              return "TRACK";
    }
}
