#ifndef NAVI_INTERNAL_H
#define NAVI_INTERNAL_H

#include "navi.h"
#include <math.h>

#define NAVI_G   9.80665f
#define NAVI_PI  3.14159265f
#define NAVI_PF_N 256

/* What the behaviour layer asks the position controller to do. */
typedef struct {
    float x, y, z;       /* position setpoint, odom frame            */
    float vx, vy;        /* velocity feed-forward                    */
    float v_max;         /* horizontal speed limit                   */
    int   motors_off;    /* 1: throttle zero, disarmed               */
    int   idle;          /* 1: armed, throttle at idle (pre-takeoff) */
    int   blind_descent; /* 1: level attitude, fixed sink throttle   */
    int   level;         /* 1: level attitude, height loop still on  */
} navi_sp_t;

struct navi {
    navi_cfg_t cfg;
    float t;

    /* --- own-state estimate --- */
    float x, y, z, vx, vy, vz;
    float floor_off;     /* accumulated floor-height steps            */
    float agl;           /* last measured height above whatever is below */
    float tof_dt;        /* time since last range sample              */
    float att_age, flow_age, tof_age, uwb_age;
    int   airborne;

    /* --- companion estimate: [x, y, vx, vy], mean of a particle cloud --- */
    float    tg[4];
    float    tgt_sigma;  /* 1-sigma position spread of the particles  */
    float    pf_dt;
    int      pf_lost;    /* consecutive ranges nothing explains       */
    uint32_t pf_rng;
    float    pf[NAVI_PF_N][4];
    float    pf_w[NAVI_PF_N];
    uint16_t pf_idx[NAVI_PF_N];

    /* --- controller --- */
    float ivx, ivy;      /* velocity-loop integrators (m/s^2)         */
    float hover_thr;

    /* --- behaviour --- */
    int      state;
    float    state_t;    /* seconds in current state                  */
    float    z_ramp;     /* ramped height setpoint for takeoff/land   */
    float    orbit_ang, orbit_rate, next_flit_t, side;
    float    poi[3];
    float    lowbat_t, vbat_f;
    float    hold_x, hold_y;
    /* raw-range supervisor */
    float    range_f, range_ref, home_t, home_hdg;
    int      homing;
    uint32_t rng;
    int      fault;
};

static inline float navi_clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float navi_wrap_pi(float a)
{
    while (a >  NAVI_PI) a -= 2.0f * NAVI_PI;
    while (a < -NAVI_PI) a += 2.0f * NAVI_PI;
    return a;
}

void navi_est_reset(navi_t *n);
void navi_est_update(navi_t *n, const navi_in_t *in);
void navi_ctrl_reset(navi_t *n);
void navi_ctrl_update(navi_t *n, const navi_in_t *in, const navi_sp_t *sp,
                      navi_out_t *out);
void navi_behavior_update(navi_t *n, const navi_in_t *in, navi_sp_t *sp,
                          navi_out_t *out);

#endif
