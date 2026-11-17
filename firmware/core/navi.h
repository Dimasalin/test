/*
 * Fairy companion drone -- portable flight "brain".
 *
 * This is the outer-loop autopilot that runs on the companion MCU at 50 Hz.
 * It contains no hardware access: the ESP32 HAL (firmware/esp32) and the
 * Python simulator (sim/) both feed it the same navi_in_t and consume the
 * same navi_out_t, so the code that is simulated is the code that flies.
 *
 * Conventions
 *   World ("odom") frame: x forward, y left, z up, origin at the take-off spot.
 *   roll  > 0 : right side down  -> accelerates to the right (-y)
 *   pitch > 0 : nose down        -> accelerates forward (+x)
 *   gyro_x / gyro_y: body rates about body x (forward) / y (left), same signs.
 *   flow_x / flow_y: optical-flow angular rate [rad/s], defined so that in
 *                    level flight v_body = flow * height.
 *
 * All struct fields are 4 bytes wide so the layout can be mirrored by ctypes.
 */
#ifndef NAVI_H
#define NAVI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NAVI_LANDED = 0,   /* motors off, glowing dimly                        */
    NAVI_TAKEOFF,      /* arming, spool-up, climb to hover height          */
    NAVI_FOLLOW,       /* flit around the companion (default)              */
    NAVI_TARGET,       /* dart to a point of interest and circle it        */
    NAVI_ATTENTION,    /* bounce + flash + chirp in front of the companion */
    NAVI_LAND,         /* controlled descent, then disarm                  */
    NAVI_FAILSAFE,     /* sensors lost: level, sink, cut                   */
    NAVI_HOME          /* fly to the charging pad and land on it           */
} navi_state_t;

typedef enum {
    NAVI_CMD_NONE = 0,
    NAVI_CMD_TAKEOFF,
    NAVI_CMD_LAND,
    NAVI_CMD_ATTENTION,
    NAVI_CMD_TARGET,   /* cmd_x/y/z = point of interest in the odom frame  */
    NAVI_CMD_RELEASE,  /* leave TARGET, go back to FOLLOW                  */
    NAVI_CMD_KILL,     /* disarm immediately                               */
    NAVI_CMD_HOME      /* go to the pad (lands in place if there is none)  */
} navi_cmd_t;

enum {
    NAVI_FAULT_ATT   = 1 << 0,  /* flight-controller telemetry stale */
    NAVI_FAULT_FLOW  = 1 << 1,  /* optical flow stale / low quality  */
    NAVI_FAULT_TOF   = 1 << 2,  /* range finder stale / out of range */
    NAVI_FAULT_UWB   = 1 << 3,  /* companion beacon not heard        */
    NAVI_FAULT_VBAT  = 1 << 4,  /* battery below landing threshold   */
    NAVI_FAULT_PAD   = 1 << 5   /* pad beacon lost while going home  */
};

typedef struct {
    float   dt;              /* seconds since previous call               */
    float   roll, pitch, yaw;/* rad, from the flight controller           */
    float   gyro_x, gyro_y;  /* rad/s                                     */
    int32_t att_fresh;       /* 1 if attitude/gyro were updated this tick */
    float   flow_x, flow_y;  /* rad/s                                     */
    int32_t flow_quality;    /* 0..255                                    */
    int32_t flow_fresh;
    float   tof_range;       /* m along the sensor axis, <0 = no target   */
    int32_t tof_fresh;
    float   uwb_range;       /* m to the companion's beacon               */
    int32_t uwb_fresh;
    float   vbat;            /* V                                         */
    int32_t cmd;             /* navi_cmd_t                                */
    float   cmd_x, cmd_y, cmd_z;
    float   pad_range;       /* m to the charging pad's beacon            */
    int32_t pad_fresh;
} navi_in_t;

typedef struct {
    /* to the flight controller (angle mode) */
    float   rc_roll, rc_pitch, rc_yaw;  /* -1..1  */
    float   rc_throttle;                /*  0..1  */
    int32_t arm;
    /* to the glow + voice */
    int32_t led_r, led_g, led_b;        /* 0..255 */
    float   buzzer_hz;                  /* 0 = silent */
    /* telemetry */
    int32_t state;
    int32_t fault;
    float   est_x, est_y, est_z, est_vx, est_vy, est_vz;
    float   tgt_x, tgt_y, tgt_vx, tgt_vy, tgt_sigma;
    float   sp_x, sp_y, sp_z;
    float   hover_thr;
    float   pad_x, pad_y, pad_sigma;    /* where it thinks the pad is */
    int32_t home_phase;                 /* 0 transit, 1 survey, 2 move in, 3 settle; -1 n/a */
} navi_out_t;

typedef struct {
    /* airframe / flight controller */
    float   bf_angle_limit;   /* rad, Betaflight angle-mode limit           */
    float   hover_thr_init;   /* first guess of hover throttle              */
    float   thr_min, thr_max; /* throttle clamp while flying                */
    /* position control */
    float   kp_pos, kv_pos, ki_vel, acc_max;
    float   kp_z, kv_z, ki_hover, vz_max;
    /* estimator */
    float   flow_lpf;         /* 0..1 blend of new flow velocity            */
    int32_t flow_min_quality;
    float   z_alpha, z_beta;  /* alpha-beta height filter                   */
    float   floor_step;       /* m, range jump treated as "floor changed"   */
    /* companion tracking (range-only particle filter) */
    int32_t uwb_enabled;      /* 0: orbit the take-off spot instead         */
    float   uwb_sigma;        /* m                                          */
    float   tgt_acc_sigma;    /* m/s^1.5, slow wander of a person's velocity */
    float   tgt_vel_tau;      /* s, mean time between stop / set-off events  */
    float   tgt_init_x, tgt_init_y, tgt_init_sigma;
    float   beacon_z;         /* m, height the beacon is worn at            */
    /* behaviour */
    float   hover_z;          /* m, shoulder height: used without a beacon  */
    float   safe_z;           /* m, above head height: used when following  */
    float   z_max;            /* m, stay below the ceiling                  */
    float   orbit_radius;     /* m                                          */
    float   keepout_radius;   /* m, never closer to the companion than this */
    float   leash;            /* m, max distance from the companion         */
    float   v_follow, v_dart; /* m/s speed limits                           */
    float   attention_secs, target_secs;
    float   idle_attention_secs; /* nag after this long in FOLLOW, 0 = off  */
    int32_t lead_enabled;     /* fly ahead of a walking companion           */
    float   vel_ff;           /* 0..1 trust in the companion velocity est.  */
    /* battery (1S) */
    float   vbat_warn, vbat_land;
    /* charging pad (a second UWB beacon at a fixed spot) */
    int32_t pad_enabled;
    float   pad_z;            /* m, height of the pad's beacon above the floor */
    float   pad_sigma;        /* m, range noise assumed for the pad beacon   */
    float   pad_orbit_radius; /* m, survey circle flown before landing       */
    float   pad_approach_z;   /* m above the pad for survey and final approach */
    float   vbat_home;        /* V, head for the pad below this; 0 = never   */
} navi_cfg_t;

typedef struct navi navi_t;

/* The caller owns the storage: allocate navi_sizeof() bytes (or use
 * NAVI_STORAGE_BYTES for a static buffer). */
#define NAVI_STORAGE_BYTES 12288
uint32_t navi_sizeof(void);
uint32_t navi_sizeof_cfg(void);
void     navi_default_cfg(navi_cfg_t *cfg);
void     navi_init(navi_t *n, const navi_cfg_t *cfg);
void     navi_step(navi_t *n, const navi_in_t *in, navi_out_t *out);

#ifdef __cplusplus
}
#endif
#endif
