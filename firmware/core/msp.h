/*
 * Minimal MultiWii Serial Protocol codec (v1 and v2 framing).
 *
 * Used for two links:
 *   - flight controller (Betaflight): we send v1 requests / MSP_SET_RAW_RC
 *     and parse v1 replies;
 *   - optical-flow + range module (MicoAir MTF-01 or Matek 3901-L0X in MSP
 *     mode): it streams v2 sensor frames at us.
 */
#ifndef NAVI_MSP_H
#define NAVI_MSP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSP_RAW_IMU      102
#define MSP_ATTITUDE     108
#define MSP_ANALOG       110
#define MSP_SET_RAW_RC   200
#define MSP2_SENSOR_RANGEFINDER 0x1F01
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02

#define MSP_MAX_PAYLOAD 64

typedef struct {
    uint8_t  state;
    uint8_t  v2;
    uint8_t  crc;
    uint16_t cmd;
    uint16_t size;
    uint16_t idx;
    uint8_t  payload[MSP_MAX_PAYLOAD];
} msp_parser_t;

/* Feed one received byte. Returns 1 when a complete, checksum-valid frame is
 * available in p->cmd / p->size / p->payload. */
int msp_parse(msp_parser_t *p, uint8_t byte);

/* Build a v1 frame ("$M<"). Returns the number of bytes written to out
 * (payload_len + 6), or 0 if it does not fit. */
int msp_encode_v1(uint8_t cmd, const uint8_t *payload, uint8_t payload_len,
                  uint8_t *out, int out_cap);

/* Build a v2 frame ("$X<"), payload_len + 9 bytes. */
int msp_encode_v2(uint16_t cmd, const uint8_t *payload, uint16_t payload_len,
                  uint8_t *out, int out_cap);

/* MSP_SET_RAW_RC with n channels given in microseconds (1000..2000). */
int msp_encode_rc(const uint16_t *channels_us, int n, uint8_t *out, int out_cap);

/* -1..1 -> 1000..2000 us (centre 1500);  0..1 -> 1000..2000 us. */
uint16_t msp_stick_us(float v);
uint16_t msp_throttle_us(float v);

uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t byte);

#ifdef __cplusplus
}
#endif
#endif
