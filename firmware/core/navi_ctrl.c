/*
 * Position controller. Produces angle-mode stick commands for the flight
 * controller, which closes the fast attitude/rate loops itself.
 *
 *   position error -> velocity setpoint -> acceleration -> tilt angle
 *   height error   -> climb rate        -> acceleration -> throttle
 *
 * Thrust is roughly proportional to throttle squared, so throttle is scaled
 * as hover_thr * sqrt(thrust ratio). hover_thr is learned in flight, which
 * also tracks battery sag.
 */
#include "navi_internal.h"

void navi_ctrl_reset(navi_t *n)
{
    n->ivx = n->ivy = 0.0f;
    n->hover_thr = n->cfg.hover_thr_init;
}

void navi_ctrl_update(navi_t *n, const navi_in_t *in, const navi_sp_t *sp,
                      navi_out_t *out)
{
    const navi_cfg_t *c = &n->cfg;
    float dt = in->dt;

    out->rc_roll = out->rc_pitch = out->rc_yaw = 0.0f;
    out->rc_throttle = 0.0f;
    out->arm = 0;

    if (sp->motors_off) {
        n->ivx = n->ivy = 0.0f;
        return;
    }
    out->arm = 1;
    if (sp->idle)
        return;
    if (sp->blind_descent) {
        /* No trustworthy position: stay level and sink gently. */
        out->rc_throttle = navi_clamp(0.96f * n->hover_thr, c->thr_min, c->thr_max);
        return;
    }

    /* ---- horizontal ---- */
    float vxs = c->kp_pos * (sp->x - n->x) + sp->vx;
    float vys = c->kp_pos * (sp->y - n->y) + sp->vy;
    float vn = sqrtf(vxs * vxs + vys * vys);
    if (vn > sp->v_max) {
        vxs *= sp->v_max / vn;
        vys *= sp->v_max / vn;
    }
    float evx = vxs - n->vx, evy = vys - n->vy;
    float ax = c->kv_pos * evx + n->ivx;
    float ay = c->kv_pos * evy + n->ivy;
    float an = sqrtf(ax * ax + ay * ay);
    int saturated = an > c->acc_max;
    if (saturated) {
        ax *= c->acc_max / an;
        ay *= c->acc_max / an;
    }
    if (n->airborne && !saturated) {
        float lim = 0.5f * c->acc_max;
        n->ivx = navi_clamp(n->ivx + c->ki_vel * evx * dt, -lim, lim);
        n->ivy = navi_clamp(n->ivy + c->ki_vel * evy * dt, -lim, lim);
    }

    float cy = cosf(in->yaw), sy = sinf(in->yaw);
    float a_fwd  =  cy * ax + sy * ay;
    float a_left = -sy * ax + cy * ay;
    float pitch_sp = atanf(a_fwd / NAVI_G);
    float roll_sp  = atanf(-a_left / NAVI_G);
    if (!sp->level) {
        out->rc_pitch = navi_clamp(pitch_sp / c->bf_angle_limit, -1.0f, 1.0f);
        out->rc_roll  = navi_clamp(roll_sp  / c->bf_angle_limit, -1.0f, 1.0f);
    }

    /* ---- vertical ---- */
    float vzs = navi_clamp(c->kp_z * (sp->z - n->z), -c->vz_max, c->vz_max);
    float evz = vzs - n->vz;
    float az = navi_clamp(c->kv_z * evz, -0.6f * NAVI_G, 1.0f * NAVI_G);

    if (!n->airborne && n->z < 0.05f && sp->z > 0.2f) {
        /* Still on the ground during take-off: spool up until we lift. */
        n->hover_thr += 0.20f * dt;
    } else {
        n->hover_thr += c->ki_hover * evz * dt;
    }
    n->hover_thr = navi_clamp(n->hover_thr, 0.15f, 0.90f);

    float tilt = navi_clamp(cosf(in->roll) * cosf(in->pitch), 0.7f, 1.0f);
    float ratio = navi_clamp((1.0f + az / NAVI_G) / tilt, 0.1f, 3.0f);
    out->rc_throttle =
        navi_clamp(n->hover_thr * sqrtf(ratio), c->thr_min, c->thr_max);
}
