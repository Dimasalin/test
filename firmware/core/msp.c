#include "msp.h"

enum { S_IDLE, S_DOLLAR, S_PROTO, S_V1_SIZE, S_V1_CMD, S_V1_PAYLOAD, S_V1_CRC,
       S_V2_FLAG, S_V2_CMD_LO, S_V2_CMD_HI, S_V2_SIZE_LO, S_V2_SIZE_HI,
       S_V2_PAYLOAD, S_V2_CRC };

uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

int msp_parse(msp_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case S_IDLE:
        if (b == '$') p->state = S_DOLLAR;
        break;
    case S_DOLLAR:
        if (b == 'M') { p->v2 = 0; p->state = S_PROTO; }
        else if (b == 'X') { p->v2 = 1; p->state = S_PROTO; }
        else if (b != '$') p->state = S_IDLE;   /* "$$": still a frame start */
        break;
    case S_PROTO:               /* direction: '<' request, '>' reply, '!' error */
        if (b == '<' || b == '>') p->state = p->v2 ? S_V2_FLAG : S_V1_SIZE;
        else p->state = b == '$' ? S_DOLLAR : S_IDLE;
        p->crc = 0;
        p->idx = 0;
        break;

    case S_V1_SIZE:
        p->size = b;
        p->crc = b;
        p->state = b <= MSP_MAX_PAYLOAD ? S_V1_CMD : S_IDLE;
        break;
    case S_V1_CMD:
        p->cmd = b;
        p->crc ^= b;
        p->state = p->size ? S_V1_PAYLOAD : S_V1_CRC;
        break;
    case S_V1_PAYLOAD:
        p->payload[p->idx++] = b;
        p->crc ^= b;
        if (p->idx >= p->size) p->state = S_V1_CRC;
        break;
    case S_V1_CRC:
        p->state = S_IDLE;
        return p->crc == b;

    case S_V2_FLAG:
        p->crc = msp_crc8_dvb_s2(0, b);
        p->state = S_V2_CMD_LO;
        break;
    case S_V2_CMD_LO:
        p->cmd = b;
        p->crc = msp_crc8_dvb_s2(p->crc, b);
        p->state = S_V2_CMD_HI;
        break;
    case S_V2_CMD_HI:
        p->cmd |= (uint16_t)b << 8;
        p->crc = msp_crc8_dvb_s2(p->crc, b);
        p->state = S_V2_SIZE_LO;
        break;
    case S_V2_SIZE_LO:
        p->size = b;
        p->crc = msp_crc8_dvb_s2(p->crc, b);
        p->state = S_V2_SIZE_HI;
        break;
    case S_V2_SIZE_HI:
        p->size |= (uint16_t)b << 8;
        p->crc = msp_crc8_dvb_s2(p->crc, b);
        if (p->size > MSP_MAX_PAYLOAD) p->state = S_IDLE;
        else p->state = p->size ? S_V2_PAYLOAD : S_V2_CRC;
        break;
    case S_V2_PAYLOAD:
        p->payload[p->idx++] = b;
        p->crc = msp_crc8_dvb_s2(p->crc, b);
        if (p->idx >= p->size) p->state = S_V2_CRC;
        break;
    case S_V2_CRC:
        p->state = S_IDLE;
        return p->crc == b;
    default:
        p->state = S_IDLE;
        break;
    }
    return 0;
}

int msp_encode_v1(uint8_t cmd, const uint8_t *payload, uint8_t n,
                  uint8_t *out, int cap)
{
    if (cap < n + 6) return 0;
    uint8_t crc = n ^ cmd;
    out[0] = '$'; out[1] = 'M'; out[2] = '<';
    out[3] = n;   out[4] = cmd;
    for (int i = 0; i < n; i++) {
        out[5 + i] = payload[i];
        crc ^= payload[i];
    }
    out[5 + n] = crc;
    return n + 6;
}

int msp_encode_v2(uint16_t cmd, const uint8_t *payload, uint16_t n,
                  uint8_t *out, int cap)
{
    if (cap < n + 9) return 0;
    out[0] = '$'; out[1] = 'X'; out[2] = '<';
    out[3] = 0;
    out[4] = (uint8_t)(cmd & 0xFF); out[5] = (uint8_t)(cmd >> 8);
    out[6] = (uint8_t)(n & 0xFF);   out[7] = (uint8_t)(n >> 8);
    for (int i = 0; i < n; i++) out[8 + i] = payload[i];
    uint8_t crc = 0;
    for (int i = 3; i < 8 + n; i++) crc = msp_crc8_dvb_s2(crc, out[i]);
    out[8 + n] = crc;
    return n + 9;
}

int msp_encode_rc(const uint16_t *ch, int n, uint8_t *out, int cap)
{
    uint8_t pl[36];
    if (n < 1 || n > 18) return 0;
    for (int i = 0; i < n; i++) {
        pl[2 * i] = (uint8_t)(ch[i] & 0xFF);
        pl[2 * i + 1] = (uint8_t)(ch[i] >> 8);
    }
    return msp_encode_v1(MSP_SET_RAW_RC, pl, (uint8_t)(2 * n), out, cap);
}

uint16_t msp_stick_us(float v)
{
    if (v < -1.0f) v = -1.0f;
    if (v > 1.0f) v = 1.0f;
    return (uint16_t)(1500.0f + 500.0f * v + 0.5f);
}

uint16_t msp_throttle_us(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return (uint16_t)(1000.0f + 1000.0f * v + 0.5f);
}
