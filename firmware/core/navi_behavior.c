/*
 * Behaviour: what makes it a fairy rather than a drone holding position.
 *
 *   FOLLOW     hover near the companion's shoulder, drift around them in
 *              short flits, bob gently; when they walk, lead the way.
 *   TARGET     dart to a point of interest and circle it, glowing yellow.
 *   ATTENTION  bounce in front of the companion, flash green and chirp.
 *   LAND       when told, or when the battery is nearly empty.
 */
#include "navi_internal.h"
#include <string.h>

uint32_t navi_sizeof(void)     { return (uint32_t)sizeof(navi_t); }
uint32_t navi_sizeof_cfg(void) { return (uint32_t)sizeof(navi_cfg_t); }

typedef char navi_storage_check[(sizeof(navi_t) <= NAVI_STORAGE_BYTES) ? 1 : -1];

void navi_default_cfg(navi_cfg_t *c)
{
    c->bf_angle_limit = 30.0f * NAVI_PI / 180.0f;
    c->hover_thr_init = 0.35f;
    c->thr_min = 0.08f;
    c->thr_max = 0.92f;

    c->kp_pos = 1.6f;
    c->kv_pos = 3.0f;
    c->ki_vel = 1.2f;
    c->acc_max = 3.5f;
    c->kp_z = 2.2f;
    c->kv_z = 4.0f;
    c->ki_hover = 0.12f;
    c->vz_max = 0.7f;

    c->flow_lpf = 0.45f;
    c->flow_min_quality = 40;
    c->z_alpha = 0.30f;
    c->z_beta = 0.035f;
    c->floor_step = 0.22f;

    c->uwb_enabled = 1;
    c->uwb_sigma = 0.15f;
    c->tgt_acc_sigma = 0.15f;
    c->tgt_vel_tau = 2.0f;
    c->tgt_init_x = -0.7f;  /* put it on the floor ~0.7 m in front of you, */
    c->tgt_init_y = 0.0f;   /* nose pointing away                          */
    c->tgt_init_sigma = 0.35f;
    c->beacon_z = 1.25f;

    c->hover_z = 1.50f;
    c->safe_z = 2.00f;
    c->z_max = 2.20f;
    c->orbit_radius = 0.90f;
    c->keepout_radius = 0.55f;
    c->leash = 3.0f;
    c->v_follow = 1.6f;
    c->v_dart = 2.0f;
    c->attention_secs = 3.0f;
    c->target_secs = 10.0f;
    c->idle_attention_secs = 0.0f;
    c->lead_enabled = 0;
    c->vel_ff = 0.5f;

    c->vbat_warn = 3.50f;
    c->vbat_land = 3.30f;
}

void navi_init(navi_t *n, const navi_cfg_t *cfg)
{
    memset(n, 0, sizeof(*n));
    n->cfg = *cfg;
    n->rng = 0x4E415649u;
    n->pf_rng = 0x46414952u;
    n->side = 1.0f;
    n->vbat_f = 4.2f;
    n->state = NAVI_LANDED;
    navi_est_reset(n);
    navi_ctrl_reset(n);
}

/* Shoulder height on its own; above heads when following someone. */
static float cruise_z(const navi_cfg_t *c)
{
    return c->uwb_enabled ? c->safe_z : c->hover_z;
}

static float frand(navi_t *n)
{
    n->rng = n->rng * 1664525u + 1013904223u;
    return (float)(n->rng >> 8) / 16777216.0f;
}

static void enter(navi_t *n, int state)
{
    n->state = state;
    n->state_t = 0.0f;
    n->hold_x = n->x;
    n->hold_y = n->y;
    n->z_ramp = n->z;
}

static void glow(navi_out_t *out, float r, float g, float b, float level)
{
    level = navi_clamp(level, 0.0f, 1.0f);
    out->led_r = (int32_t)(r * level);
    out->led_g = (int32_t)(g * level);
    out->led_b = (int32_t)(b * level);
}

/* Fast shimmer (wing beat) riding on a slow breath. */
static float shimmer(float t)
{
    return 0.72f + 0.18f * sinf(2.0f * NAVI_PI * 9.0f * t) +
           0.10f * sinf(2.0f * NAVI_PI * 0.7f * t);
}

static void handle_command(navi_t *n, const navi_in_t *in)
{
    const navi_cfg_t *c = &n->cfg;
    int flying = n->state != NAVI_LANDED;

    switch (in->cmd) {
    case NAVI_CMD_KILL:
        enter(n, NAVI_LANDED);
        break;
    case NAVI_CMD_TAKEOFF:
        if (!flying && n->att_age < 0.2f && n->tof_age < 0.2f &&
            (in->vbat < 0.5f || in->vbat > c->vbat_warn)) {
            navi_est_reset(n);
            navi_ctrl_reset(n);
            n->fault = 0;
            n->lowbat_t = 0.0f;
            n->vbat_f = in->vbat > 0.5f ? in->vbat : 4.2f;
            n->orbit_ang = navi_wrap_pi(
                atan2f(-c->tgt_init_y, -c->tgt_init_x)); /* where we stand */
            n->orbit_rate = 0.0f;
            n->next_flit_t = 0.0f;
            enter(n, NAVI_TAKEOFF);
            n->z_ramp = 0.0f;
        }
        break;
    case NAVI_CMD_LAND:
        if (flying && n->state != NAVI_FAILSAFE) enter(n, NAVI_LAND);
        break;
    case NAVI_CMD_ATTENTION:
        if (n->state == NAVI_FOLLOW) enter(n, NAVI_ATTENTION);
        break;
    case NAVI_CMD_TARGET:
        if (n->state == NAVI_FOLLOW || n->state == NAVI_TARGET) {
            n->poi[0] = in->cmd_x;
            n->poi[1] = in->cmd_y;
            n->poi[2] = in->cmd_z;
            enter(n, NAVI_TARGET);
        }
        break;
    case NAVI_CMD_RELEASE:
        if (n->state == NAVI_TARGET) enter(n, NAVI_FOLLOW);
        break;
    default:
        break;
    }
}

static void check_health(navi_t *n, const navi_in_t *in)
{
    const navi_cfg_t *c = &n->cfg;
    if (n->state == NAVI_LANDED) return;

    int f = 0;
    if (n->att_age > 0.25f) f |= NAVI_FAULT_ATT;
    if (n->flow_age > 0.8f) f |= NAVI_FAULT_FLOW;
    if (n->tof_age > 0.6f)  f |= NAVI_FAULT_TOF;
    if (n->uwb_age > 2.0f)  f |= NAVI_FAULT_UWB;

    if (in->vbat > 0.5f) {
        n->vbat_f += navi_clamp(in->dt, 0.0f, 1.0f) * (in->vbat - n->vbat_f);
        if (n->vbat_f < c->vbat_land) n->lowbat_t += in->dt;
        else n->lowbat_t = 0.0f;
    }
    if (n->lowbat_t > 2.0f) f |= NAVI_FAULT_VBAT;
    n->fault = f | (n->fault & NAVI_FAULT_VBAT);

    if (n->state == NAVI_FAILSAFE) return;
    if (f & (NAVI_FAULT_ATT | NAVI_FAULT_FLOW | NAVI_FAULT_TOF)) {
        enter(n, NAVI_FAILSAFE);
        return;
    }
    if (n->state == NAVI_LAND || n->state == NAVI_TAKEOFF) return;
    if ((f & NAVI_FAULT_VBAT) || n->uwb_age > 15.0f)
        enter(n, NAVI_LAND);
}

/* Where to hover relative to the companion. */
static void follow_setpoint(navi_t *n, const navi_in_t *in, navi_sp_t *sp)
{
    const navi_cfg_t *c = &n->cfg;
    float dt = in->dt;
    float tx = n->tg[0], ty = n->tg[1], tvx = n->tg[2], tvy = n->tg[3];
    float speed = sqrtf(tvx * tvx + tvy * tvy);
    float sigma = n->tgt_sigma;
    float rate;

    if (n->t >= n->next_flit_t) {
        /* Pick a new whim: pause, or drift one way or the other. */
        float u = frand(n);
        float mag = 0.25f + 0.35f * frand(n);
        n->orbit_rate = u < 0.2f ? 0.0f : (u < 0.6f ? mag : -mag);
        if (frand(n) < 0.3f) n->side = -n->side;
        n->next_flit_t = n->t + 1.5f + 3.0f * frand(n);
    }

    if (c->lead_enabled && speed > 0.35f && sigma < 0.5f) {
        /* Companion is walking: fly ahead and off to one side, weaving.
         * The weave keeps changing the geometry so range-only tracking
         * stays observable. */
        float heading = atan2f(tvy, tvx);
        float want = heading +
                     n->side * (1.0f + 0.45f * sinf(1.3f * n->t));
        rate = navi_clamp(2.0f * navi_wrap_pi(want - n->orbit_ang), -1.6f, 1.6f);
    } else {
        rate = n->orbit_rate;
        if (sigma > 0.30f) {
            /* Unsure where they are: move, that is what gives a fix. */
            float dir = rate < 0.0f ? -1.0f : 1.0f;
            float mag = fabsf(rate);
            if (mag < 0.6f) mag = 0.6f;
            rate = dir * mag;
        }
    }
    n->orbit_ang = navi_wrap_pi(n->orbit_ang + rate * dt);

    float ca = cosf(n->orbit_ang), sa = sinf(n->orbit_ang);
    float R = c->orbit_radius;
    sp->x = tx + R * ca + 0.04f * sinf(2.3f * n->t);
    sp->y = ty + R * sa + 0.04f * sinf(1.7f * n->t + 1.0f);
    sp->z = c->hover_z + 0.06f * sinf(2.0f * NAVI_PI * 0.55f * n->t);
    sp->vx = c->vel_ff * tvx - R * rate * sa;
    sp->vy = c->vel_ff * tvy + R * rate * ca;
    sp->v_max = c->v_follow;
}

/* Raw-range supervisor.
 *
 * The particle filter can be confidently wrong (a mirror-image hypothesis
 * explains the ranges for a while). The measured distance itself cannot: so
 * if it says we are too far away, stop trusting the estimate and home in the
 * way a moth -- or a bacterium -- would: fly straight while the distance is
 * shrinking, turn when it is not. Bounded by construction.
 * Returns 1 while it owns the setpoint. */
static int homing_setpoint(navi_t *n, const navi_in_t *in, navi_sp_t *sp)
{
    const navi_cfg_t *c = &n->cfg;
    if (!c->uwb_enabled) return 0;

    if (!n->homing && n->range_f > c->leash) {
        n->homing = 1;
        n->home_t = 0.0f;
        n->range_ref = n->range_f;
        n->home_hdg = atan2f(n->tg[1] - n->y, n->tg[0] - n->x); /* best guess */
    }
    if (!n->homing) return 0;
    if (n->range_f < c->orbit_radius + 0.7f) {
        n->homing = 0;
        return 0;
    }
    n->home_t += in->dt;
    if (n->home_t > 0.8f) {
        float closing = (n->range_ref - n->range_f) / n->home_t;
        if (closing < 0.3f)
            n->home_hdg = navi_wrap_pi(n->home_hdg + 1.7f + 1.2f * frand(n));
        n->home_t = 0.0f;
        n->range_ref = n->range_f;
    }
    float ch = cosf(n->home_hdg), sh = sinf(n->home_hdg);
    sp->x = n->x + 0.8f * ch;
    sp->y = n->y + 0.8f * sh;
    sp->z = c->safe_z;
    sp->vx = 0.0f;
    sp->vy = 0.0f;
    sp->v_max = c->v_dart;
    return 1;
}

/* Never crowd the companion, whatever the behaviour wants. */
static void keepout(navi_t *n, navi_sp_t *sp)
{
    const navi_cfg_t *c = &n->cfg;
    float dx = n->x - n->tg[0], dy = n->y - n->tg[1];
    float d = sqrtf(dx * dx + dy * dy);
    if (d < c->keepout_radius && d > 0.02f) {
        float push = 3.0f * (c->keepout_radius - d);
        sp->vx += push * dx / d;
        sp->vy += push * dy / d;
    }
    /* and do not ask for a setpoint inside the bubble either */
    float sx = sp->x - n->tg[0], sy = sp->y - n->tg[1];
    float sd = sqrtf(sx * sx + sy * sy);
    if (sd < c->keepout_radius && sd > 0.02f) {
        sp->x = n->tg[0] + sx / sd * c->keepout_radius;
        sp->y = n->tg[1] + sy / sd * c->keepout_radius;
    }
    /* ...nor for a straight line that cuts through it: if the path to the
     * setpoint passes too close, aim for a point beside them instead. */
    float ax = sp->x - n->x, ay = sp->y - n->y;
    float a2 = ax * ax + ay * ay;
    if (a2 > 0.01f) {
        float u = navi_clamp(((n->tg[0] - n->x) * ax + (n->tg[1] - n->y) * ay) / a2,
                             0.0f, 1.0f);
        float cx = n->x + u * ax - n->tg[0], cy = n->y + u * ay - n->tg[1];
        float cd = sqrtf(cx * cx + cy * cy);
        if (cd < c->keepout_radius && u > 0.0f && u < 1.0f) {
            if (cd < 0.05f) {           /* dead centre: pick a side */
                float an = sqrtf(a2);
                cx = -ay / an;
                cy = ax / an;
                cd = 1.0f;
            }
            float r = 1.5f * c->keepout_radius;
            sp->x = n->tg[0] + cx / cd * r;
            sp->y = n->tg[1] + cy / cd * r;
        }
    }

    /* Range-only tracking is occasionally confidently wrong (see
     * docs/DESIGN.md, "What the simulation taught us"), so when following a
     * person we never fly at a height where being wrong means hitting them:
     * everything happens above head height. */
    if (c->uwb_enabled) {
        sp->z += c->safe_z - c->hover_z;        /* keep the bob and bounce */
        if (sp->z < c->safe_z - 0.10f) sp->z = c->safe_z - 0.10f;
    }
}

static float chirp(float ts, float period)
{
    float p = fmodf(ts, period);
    if (p < 0.09f) return 2800.0f;
    if (p > 0.14f && p < 0.34f) return 3600.0f + 1500.0f * (p - 0.14f);
    return 0.0f;
}

void navi_behavior_update(navi_t *n, const navi_in_t *in, navi_sp_t *sp,
                          navi_out_t *out)
{
    const navi_cfg_t *c = &n->cfg;
    float dt = in->dt;

    memset(sp, 0, sizeof(*sp));
    handle_command(n, in);
    check_health(n, in);
    n->state_t += dt;
    float ts = n->state_t;

    switch (n->state) {
    case NAVI_LANDED:
        sp->motors_off = 1;
        glow(out, 90, 150, 255, 0.10f + 0.06f * sinf(2.0f * NAVI_PI * 0.3f * n->t));
        break;

    case NAVI_TAKEOFF:
        if (ts < 1.0f) {
            sp->idle = 1;               /* let the FC arm with throttle low */
        } else {
            n->z_ramp += 0.5f * dt;
            if (n->z_ramp > cruise_z(c)) n->z_ramp = cruise_z(c);
        }
        sp->x = 0.0f;
        sp->y = 0.0f;
        sp->z = n->z_ramp;
        sp->v_max = 0.5f;
        if (ts < 1.0f) out->buzzer_hz = 1800.0f + 1800.0f * ts;
        glow(out, 150, 200, 255, navi_clamp(ts / 2.0f, 0.1f, 1.0f) * shimmer(n->t));
        if (n->z > cruise_z(c) - 0.15f) enter(n, NAVI_FOLLOW);
        else if (ts > 7.0f && !n->airborne) enter(n, NAVI_LANDED);
        break;

    case NAVI_FOLLOW:
        if (n->fault & NAVI_FAULT_UWB) {
            /* Beacon quiet: wait where we are rather than chase a guess. */
            sp->x = n->hold_x;
            sp->y = n->hold_y;
            sp->z = cruise_z(c);
            sp->v_max = 0.5f;
            glow(out, 255, 150, 40, 0.5f + 0.4f * sinf(2.0f * NAVI_PI * 1.5f * n->t));
        } else if (homing_setpoint(n, in, sp)) {
            n->hold_x = n->x;
            n->hold_y = n->y;
            glow(out, 255, 255, 255, shimmer(n->t));   /* "where did you go?" */
        } else {
            follow_setpoint(n, in, sp);
            keepout(n, sp);
            n->hold_x = n->x;
            n->hold_y = n->y;
            if (in->vbat > 0.5f && n->vbat_f < c->vbat_warn)
                glow(out, 255, 90, 60, shimmer(n->t)); /* getting tired */
            else
                glow(out, 150, 200, 255, shimmer(n->t));
            if (c->idle_attention_secs > 0.0f && ts > c->idle_attention_secs)
                enter(n, NAVI_ATTENTION);
        }
        break;

    case NAVI_TARGET: {
        float px = n->poi[0] - n->tg[0], py = n->poi[1] - n->tg[1];
        float pd = sqrtf(px * px + py * py);
        if (pd > c->leash) {
            px *= c->leash / pd;
            py *= c->leash / pd;
        }
        float a = 1.6f * ts;
        sp->x = n->tg[0] + px + 0.25f * cosf(a);
        sp->y = n->tg[1] + py + 0.25f * sinf(a);
        sp->z = n->poi[2] + 0.05f * sinf(2.0f * NAVI_PI * 0.8f * ts);
        sp->v_max = c->v_dart;
        keepout(n, sp);
        if (ts < 0.15f) out->buzzer_hz = 3200.0f;
        glow(out, 255, 210, 40, shimmer(n->t));
        if (ts > c->target_secs) enter(n, NAVI_FOLLOW);
        break;
    }

    case NAVI_ATTENTION: {
        /* Hold station on the orbit, a little closer, and bounce. */
        float R = 0.5f * (c->orbit_radius + c->keepout_radius);
        sp->x = n->tg[0] + R * cosf(n->orbit_ang);
        sp->y = n->tg[1] + R * sinf(n->orbit_ang);
        sp->z = c->hover_z + 0.07f * sinf(2.0f * NAVI_PI * 1.2f * ts);
        sp->vx = n->tg[2];
        sp->vy = n->tg[3];
        sp->v_max = c->v_follow;
        keepout(n, sp);
        out->buzzer_hz = chirp(ts, 1.0f);
        glow(out, 90, 255, 120, fmodf(ts, 0.25f) < 0.15f ? 1.0f : 0.25f);
        if (ts > c->attention_secs) enter(n, NAVI_FOLLOW);
        break;
    }

    case NAVI_LAND:
        if (c->uwb_enabled && ts < 5.0f && n->z > 1.0f &&
            n->range_f < 1.15f && !(n->fault & NAVI_FAULT_UWB)) {
            /* Too close to come down: the radio distance says someone is
             * underneath or beside us. Back away first (5 s at most). */
            float dx = n->x - n->tg[0], dy = n->y - n->tg[1];
            float d = sqrtf(dx * dx + dy * dy);
            if (d > 0.05f) {
                n->hold_x = n->x + 0.6f * dx / d;
                n->hold_y = n->y + 0.6f * dy / d;
            }
            n->z_ramp = n->z;
        } else {
            n->z_ramp -= 0.35f * dt;
        }
        sp->x = n->hold_x;
        sp->y = n->hold_y;
        sp->z = n->z_ramp > n->z - 0.25f ? n->z_ramp : n->z - 0.25f;
        sp->v_max = 0.5f;
        if (n->fault & NAVI_FAULT_VBAT)
            glow(out, 255, 40, 30, 0.4f + 0.5f * sinf(2.0f * NAVI_PI * 2.0f * n->t));
        else
            glow(out, 150, 200, 255, 0.5f * shimmer(n->t));
        if ((n->tof_age < 0.1f && n->agl < 0.07f) || ts > 12.0f)
            enter(n, NAVI_LANDED);
        break;

    case NAVI_FAILSAFE:
    default:
        if (n->tof_age < 0.3f && n->att_age < 0.25f) {
            /* Height still known: level out and descend under control. */
            n->z_ramp -= 0.4f * dt;
            sp->level = 1;
            sp->z = n->z_ramp > n->z - 0.25f ? n->z_ramp : n->z - 0.25f;
            if (n->agl < 0.07f) enter(n, NAVI_LANDED);
        } else {
            sp->blind_descent = 1;
        }
        glow(out, 255, 0, 0, fmodf(n->t, 0.2f) < 0.1f ? 1.0f : 0.0f);
        if (ts > 6.0f) enter(n, NAVI_LANDED);
        break;
    }

    if (n->state != NAVI_LANDED && n->state != NAVI_FAILSAFE)
        sp->z = navi_clamp(sp->z, -1.0f, c->z_max);
    if (n->state == NAVI_LANDED) sp->motors_off = 1;
}

void navi_step(navi_t *n, const navi_in_t *in, navi_out_t *out)
{
    navi_sp_t sp;
    memset(out, 0, sizeof(*out));
    n->t += in->dt;

    navi_est_update(n, in);
    navi_behavior_update(n, in, &sp, out);
    navi_ctrl_update(n, in, &sp, out);

    out->state = n->state;
    out->fault = n->fault;
    out->est_x = n->x;   out->est_y = n->y;   out->est_z = n->z;
    out->est_vx = n->vx; out->est_vy = n->vy; out->est_vz = n->vz;
    out->tgt_x = n->tg[0];  out->tgt_y = n->tg[1];
    out->tgt_vx = n->tg[2]; out->tgt_vy = n->tg[3];
    out->tgt_sigma = n->tgt_sigma;
    out->sp_x = sp.x; out->sp_y = sp.y; out->sp_z = sp.z;
    out->hover_thr = n->hover_thr;
}
