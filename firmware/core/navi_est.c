/*
 * State estimation.
 *
 *  1. Own height:    alpha-beta filter on the downward range finder, with
 *                    floor-step rejection (flying over a table must not make
 *                    the fairy jump).
 *  2. Own position:  optical flow, de-rotated with the gyro and scaled by
 *                    height, integrated into a drifting "odom" frame.
 *  3. Companion:     range-only particle filter on the UWB distance to the
 *                    beacon the person wears. A single range does not give
 *                    a bearing -- the fairy's constant flitting is what
 *                    makes the person's position observable.
 */
#include "navi_internal.h"

static float urand(navi_t *n)
{
    n->pf_rng = n->pf_rng * 1664525u + 1013904223u;
    return (float)(n->pf_rng >> 8) / 16777216.0f;
}

/* Approximately N(0,1): sum of four uniforms, rescaled. */
static float gauss(navi_t *n)
{
    return (urand(n) + urand(n) + urand(n) + urand(n) - 2.0f) * 1.7320508f;
}

void navi_est_reset(navi_t *n)
{
    const navi_cfg_t *c = &n->cfg;
    n->x = n->y = n->z = 0.0f;
    n->vx = n->vy = n->vz = 0.0f;
    n->floor_off = 0.0f;
    n->agl = 0.0f;
    n->tof_dt = 0.0f;
    n->airborne = 0;
    n->tg[0] = c->tgt_init_x;
    n->tg[1] = c->tgt_init_y;
    n->tg[2] = n->tg[3] = 0.0f;
    n->tgt_sigma = c->tgt_init_sigma;
    n->pf_dt = 0.0f;
    n->pf_lost = 0;
    n->range_f = sqrtf(c->tgt_init_x * c->tgt_init_x + c->tgt_init_y * c->tgt_init_y);
    n->homing = 0;
    for (int i = 0; i < NAVI_PF_N; i++) {
        n->pf[i][0] = c->tgt_init_x + c->tgt_init_sigma * 0.707f * gauss(n);
        n->pf[i][1] = c->tgt_init_y + c->tgt_init_sigma * 0.707f * gauss(n);
        n->pf[i][2] = n->pf[i][3] = 0.0f;
        n->pf_w[i] = 1.0f / NAVI_PF_N;
    }
    n->pad_init = 0;
    n->pad_lost = 0;
    n->pad_dt = 0.0f;
    n->pad_sig = 9.0f;
    n->pad_range_f = 0.0f;
}

static void height_update(navi_t *n, const navi_in_t *in, float dt)
{
    const navi_cfg_t *c = &n->cfg;
    float tilt = cosf(in->roll) * cosf(in->pitch);

    n->z += n->vz * dt;
    n->tof_dt += dt;

    if (in->tof_fresh && in->tof_range > 0.0f) {
        n->agl = in->tof_range * tilt;
        float r = n->agl + n->floor_off - n->z;
        if (n->airborne && fabsf(r) > c->floor_step) {
            /* The ground moved, not us: absorb it into the floor offset. */
            n->floor_off -= r;
            r = 0.0f;
        }
        float sdt = navi_clamp(n->tof_dt, 0.005f, 0.2f);
        n->z  += c->z_alpha * r;
        n->vz += c->z_beta / sdt * r;
        n->tof_dt = 0.0f;
        n->tof_age = 0.0f;
    }
    n->airborne = n->z > 0.30f;
}

static void flow_update(navi_t *n, const navi_in_t *in, float dt)
{
    const navi_cfg_t *c = &n->cfg;

    if (n->agl < 0.06f) {
        /* Sitting on (or about to touch) the ground. */
        n->vx = n->vy = 0.0f;
        if (in->flow_fresh) n->flow_age = 0.0f;
    } else if (in->flow_fresh && in->flow_quality >= c->flow_min_quality) {
        float tilt = cosf(in->roll) * cosf(in->pitch);
        float d = n->agl / navi_clamp(tilt, 0.5f, 1.0f); /* slant range */
        float vbx = (in->flow_x + in->gyro_y) * d;
        float vby = (in->flow_y - in->gyro_x) * d;
        float cy = cosf(in->yaw), sy = sinf(in->yaw);
        float mx = cy * vbx - sy * vby;
        float my = sy * vbx + cy * vby;
        n->vx += c->flow_lpf * (mx - n->vx);
        n->vy += c->flow_lpf * (my - n->vy);
        n->flow_age = 0.0f;
    }
    n->x += n->vx * dt;
    n->y += n->vy * dt;
}

/* ---- companion tracker: particle filter -------------------------------
 *
 * One range puts the person on a circle around the drone, not at a point, and
 * two ranges from different places still leave a mirror-image pair. A Kalman
 * filter has to commit to one of them; particles can carry both until the
 * drone's own movement rules one out.
 *
 * Each particle is a hypothesis [x, y, vx, vy] in the odom frame.
 */
/* Motion model: people mostly stand still, sometimes set off in a straight
 * line, and sometimes stop. So velocity is persistent (not white noise), with
 * occasional jumps to "stopped" or "walking in a new direction". Persistent
 * hypotheses are what make a walking person observable from range alone. */
static void pf_predict(navi_t *n, float dt)
{
    const navi_cfg_t *c = &n->cfg;
    float sv = c->tgt_acc_sigma * sqrtf(dt);
    float p_jump = dt / c->tgt_vel_tau;      /* per-particle jump probability */
    for (int i = 0; i < NAVI_PF_N; i++) {
        float *p = n->pf[i];
        float u = urand(n);
        if (u < p_jump) {
            p[2] = p[3] = 0.0f;                         /* stops */
        } else if (u < 2.0f * p_jump) {
            float ang = 2.0f * NAVI_PI * urand(n);      /* sets off */
            float spd = 0.5f + 0.9f * urand(n);
            p[2] = spd * cosf(ang);
            p[3] = spd * sinf(ang);
        } else {
            p[2] += sv * gauss(n);
            p[3] += sv * gauss(n);
        }
        float spd2 = p[2] * p[2] + p[3] * p[3];
        if (spd2 > 1.5f * 1.5f) {       /* people indoors do not sprint */
            float k = 1.5f / sqrtf(spd2);
            p[2] *= k;
            p[3] *= k;
        }
        p[0] += p[2] * dt + 0.015f * gauss(n);
        p[1] += p[3] * dt + 0.015f * gauss(n);
    }
}

static void pf_weigh(navi_t *n, float range)
{
    const navi_cfg_t *c = &n->cfg;
    float dz = c->beacon_z - n->z;
    float inv2s2 = 0.5f / (c->uwb_sigma * c->uwb_sigma);
    float sum = 0.0f, support = 0.0f;
    for (int i = 0; i < NAVI_PF_N; i++) {
        float dx = n->pf[i][0] - n->x, dy = n->pf[i][1] - n->y;
        float e = range - sqrtf(dx * dx + dy * dy + dz * dz);
        support += n->pf_w[i] * expf(-e * e * inv2s2);
        /* Gaussian core plus a floor: a body-blocked (too long) range must
         * not be able to kill every good particle. */
        float l = expf(-e * e * inv2s2) + (e > 0.0f ? 0.03f : 0.003f);
        n->pf_w[i] *= l;
        sum += n->pf_w[i];
    }
    /* If hardly any hypothesis explains the range, several times in a row,
     * the cloud is simply wrong (one-off spikes are just body blocking).
     * Start again from the only thing we know: they are somewhere on a ring
     * of this radius around us. */
    n->pf_lost = support < 0.02f ? n->pf_lost + 1 : 0;
    if (n->pf_lost >= 4 || sum < 1e-30f) {
        float h2 = range * range - dz * dz;
        float h = h2 > 0.04f ? sqrtf(h2) : 0.2f;
        for (int i = 0; i < NAVI_PF_N; i++) {
            float ang = 2.0f * NAVI_PI * urand(n);
            float r = h + c->uwb_sigma * gauss(n);
            n->pf[i][0] = n->x + r * cosf(ang);
            n->pf[i][1] = n->y + r * sinf(ang);
            n->pf[i][2] = n->pf[i][3] = 0.0f;
            n->pf_w[i] = 1.0f / NAVI_PF_N;
        }
        n->pf_lost = 0;
        return;
    }
    float neff = 0.0f;
    for (int i = 0; i < NAVI_PF_N; i++) {
        n->pf_w[i] /= sum;
        neff += n->pf_w[i] * n->pf_w[i];
    }
    if (1.0f / neff > 0.5f * NAVI_PF_N) return;

    /* systematic resampling, in place via the scratch index table */
    float step = 1.0f / NAVI_PF_N;
    float u = step * urand(n), cum = n->pf_w[0];
    int j = 0;
    for (int i = 0; i < NAVI_PF_N; i++) {
        while (u > cum && j < NAVI_PF_N - 1) cum += n->pf_w[++j];
        n->pf_idx[i] = (uint16_t)j;
        u += step;
    }
    /* The index table is non-decreasing, so the copy can be done in place:
     * particles that take a later source go first in ascending order, the
     * ones that take an earlier source then go in descending order. Neither
     * pass overwrites a particle the other still needs to read. */
    for (int i = 0; i < NAVI_PF_N; i++)
        if (n->pf_idx[i] > i)
            for (int k = 0; k < 4; k++) n->pf[i][k] = n->pf[n->pf_idx[i]][k];
    for (int i = NAVI_PF_N - 1; i >= 0; i--)
        if (n->pf_idx[i] < i)
            for (int k = 0; k < 4; k++) n->pf[i][k] = n->pf[n->pf_idx[i]][k];
    for (int i = 0; i < NAVI_PF_N; i++) n->pf_w[i] = step;
}

static void pf_summarise(navi_t *n)
{
    float m[4] = {0, 0, 0, 0}, var = 0.0f;
    for (int i = 0; i < NAVI_PF_N; i++)
        for (int k = 0; k < 4; k++) m[k] += n->pf_w[i] * n->pf[i][k];
    for (int i = 0; i < NAVI_PF_N; i++) {
        float dx = n->pf[i][0] - m[0], dy = n->pf[i][1] - m[1];
        var += n->pf_w[i] * (dx * dx + dy * dy);
    }
    for (int k = 0; k < 4; k++) n->tg[k] = m[k];
    n->tgt_sigma = sqrtf(var);
}

/* ---- charging pad: locating a fixed beacon -----------------------------
 *
 * Same idea as the companion tracker but the easy case: the pad does not
 * move, so every range taken from a new place narrows it down for good. The
 * small jitter lets the cloud follow the slow drift of our own odometry.
 * It uses its own random stream so that adding a pad does not change how the
 * companion tracker behaves.
 */
static float pad_urand(navi_t *n)
{
    n->pad_rng = n->pad_rng * 1664525u + 1013904223u;
    return (float)(n->pad_rng >> 8) / 16777216.0f;
}

static float pad_gauss(navi_t *n)
{
    return (pad_urand(n) + pad_urand(n) + pad_urand(n) + pad_urand(n) - 2.0f) *
           1.7320508f;
}

static void pad_ring(navi_t *n, float range)
{
    const navi_cfg_t *c = &n->cfg;
    float dz = c->pad_z - n->z;
    float h2 = range * range - dz * dz;
    float h = h2 > 0.0025f ? sqrtf(h2) : 0.05f;
    for (int i = 0; i < NAVI_PF_N; i++) {
        float ang = 2.0f * NAVI_PI * pad_urand(n);
        float r = h + c->pad_sigma * pad_gauss(n);
        n->pad_pf[i][0] = n->x + r * cosf(ang);
        n->pad_pf[i][1] = n->y + r * sinf(ang);
        n->pad_w[i] = 1.0f / NAVI_PF_N;
    }
    n->pad_init = 1;
    n->pad_lost = 0;
}

static void pad_update(navi_t *n, float range, float dt)
{
    const navi_cfg_t *c = &n->cfg;
    if (!n->pad_init) {
        pad_ring(n, range);
        return;
    }
    float dz = c->pad_z - n->z;
    float jit = 0.02f * sqrtf(dt);
    float inv2s2 = 0.5f / (c->pad_sigma * c->pad_sigma);
    float sum = 0.0f, support = 0.0f;
    for (int i = 0; i < NAVI_PF_N; i++) {
        n->pad_pf[i][0] += jit * pad_gauss(n);
        n->pad_pf[i][1] += jit * pad_gauss(n);
        float dx = n->pad_pf[i][0] - n->x, dy = n->pad_pf[i][1] - n->y;
        float e = range - sqrtf(dx * dx + dy * dy + dz * dz);
        float g = expf(-e * e * inv2s2);
        support += n->pad_w[i] * g;
        n->pad_w[i] *= g + (e > 0.0f ? 0.02f : 0.002f);
        sum += n->pad_w[i];
    }
    n->pad_lost = support < 0.02f ? n->pad_lost + 1 : 0;
    if (n->pad_lost >= 4 || sum < 1e-30f) {
        pad_ring(n, range);
        return;
    }
    float neff = 0.0f;
    for (int i = 0; i < NAVI_PF_N; i++) {
        n->pad_w[i] /= sum;
        neff += n->pad_w[i] * n->pad_w[i];
    }
    if (1.0f / neff > 0.5f * NAVI_PF_N) return;

    float step = 1.0f / NAVI_PF_N;
    float u = step * pad_urand(n), cum = n->pad_w[0];
    int j = 0;
    for (int i = 0; i < NAVI_PF_N; i++) {
        while (u > cum && j < NAVI_PF_N - 1) cum += n->pad_w[++j];
        n->pf_idx[i] = (uint16_t)j;
        u += step;
    }
    /* in-place copy, same ordering argument as the companion filter */
    for (int i = 0; i < NAVI_PF_N; i++)
        if (n->pf_idx[i] > i)
            for (int k = 0; k < 2; k++) n->pad_pf[i][k] = n->pad_pf[n->pf_idx[i]][k];
    for (int i = NAVI_PF_N - 1; i >= 0; i--)
        if (n->pf_idx[i] < i)
            for (int k = 0; k < 2; k++) n->pad_pf[i][k] = n->pad_pf[n->pf_idx[i]][k];
    for (int i = 0; i < NAVI_PF_N; i++) n->pad_w[i] = step;
}

static void pad_summarise(navi_t *n)
{
    float mx = 0.0f, my = 0.0f, var = 0.0f;
    for (int i = 0; i < NAVI_PF_N; i++) {
        mx += n->pad_w[i] * n->pad_pf[i][0];
        my += n->pad_w[i] * n->pad_pf[i][1];
    }
    for (int i = 0; i < NAVI_PF_N; i++) {
        float dx = n->pad_pf[i][0] - mx, dy = n->pad_pf[i][1] - my;
        var += n->pad_w[i] * (dx * dx + dy * dy);
    }
    n->pad[0] = mx;
    n->pad[1] = my;
    n->pad_sig = sqrtf(var);
}

static void pad_est_update(navi_t *n, const navi_in_t *in)
{
    n->pad_age += in->dt;
    if (!n->cfg.pad_enabled || n->state == NAVI_LANDED) {
        if (in->pad_fresh) n->pad_age = 0.0f;
        return;
    }
    n->pad_dt += in->dt;
    if (in->pad_fresh && in->pad_range > 0.0f) {
        n->pad_age = 0.0f;
        if (!n->pad_init) n->pad_range_f = in->pad_range;
        float k = in->pad_range < n->pad_range_f ? 0.6f : 0.25f;
        n->pad_range_f += k * (in->pad_range - n->pad_range_f);
        pad_update(n, in->pad_range, n->pad_dt);
        n->pad_dt = 0.0f;
        pad_summarise(n);
    }
}

void navi_est_update(navi_t *n, const navi_in_t *in)
{
    float dt = in->dt;

    n->att_age += dt;
    n->flow_age += dt;
    n->tof_age += dt;
    n->uwb_age += dt;
    if (in->att_fresh) n->att_age = 0.0f;

    height_update(n, in, dt);
    flow_update(n, in, dt);
    pad_est_update(n, in);

    if (!n->cfg.uwb_enabled) {
        /* No beacon: the "companion" is a fixed anchor near take-off. */
        n->uwb_age = 0.0f;
        return;
    }
    if (n->state == NAVI_LANDED || n->state == NAVI_TAKEOFF) {
        /* Climbing straight up tells us nothing about bearing; keep the
         * prior ("they are standing behind me") until we start to move. */
        if (in->uwb_fresh) n->uwb_age = 0.0f;
        return;
    }
    n->pf_dt += dt;
    int got = in->uwb_fresh && in->uwb_range > 0.0f;
    if (got || n->pf_dt >= 0.1f) {
        pf_predict(n, n->pf_dt);
        n->pf_dt = 0.0f;
        if (got) {
            n->uwb_age = 0.0f;
            /* Smoothed raw range for the supervisor. Spikes are only ever
             * too long, so follow decreases quickly and increases slowly. */
            float k = in->uwb_range < n->range_f ? 0.6f : 0.25f;
            n->range_f += k * (in->uwb_range - n->range_f);
            pf_weigh(n, in->uwb_range);
        }
        pf_summarise(n);
    }
}
