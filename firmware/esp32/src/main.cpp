// Fairy drone companion computer.
//
// This file is only plumbing: it moves bytes between the hardware and the
// portable flight brain in firmware/core (the same code the simulator runs).
//
//   Serial1  <-> Betaflight flight controller (MSP v1)
//   Serial2  <-  optical flow + range module (MSP v2 sensor frames)
//   SPI      <-> DW3000 UWB (stage 3, see uwb.cpp)
//   NeoPixel  -> the glow          LEDC -> the voice (piezo)
//   WiFi AP   -> phone remote at http://192.168.4.1
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "uwb.h"
extern "C" {
#include "navi.h"
#include "msp.h"
}

static uint8_t navi_mem[NAVI_STORAGE_BYTES] __attribute__((aligned(8)));
static navi_t *navi = (navi_t *)navi_mem;
static navi_in_t in;
static navi_out_t out;

static msp_parser_t fc_parser, flow_parser;
static Adafruit_NeoPixel leds(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);
static WebServer web(80);

static volatile int pending_cmd = NAVI_CMD_NONE;
static uint32_t flow_last_us;
static bool fc_seen;

static int16_t rd16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
static int32_t rd32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

// ---------------------------------------------------------------- FC link
static void fc_request(uint8_t cmd)
{
    uint8_t buf[8];
    int n = msp_encode_v1(cmd, nullptr, 0, buf, sizeof(buf));
    Serial1.write(buf, n);
}

static void fc_poll()
{
    const float d2r = 0.01745329f;
    while (Serial1.available()) {
        if (!msp_parse(&fc_parser, (uint8_t)Serial1.read())) continue;
        const uint8_t *p = fc_parser.payload;
        switch (fc_parser.cmd) {
        case MSP_ATTITUDE:          // roll, pitch [0.1 deg], yaw [deg]
            if (fc_parser.size < 6) break;
            in.roll  = SIGN_ROLL  * rd16(p)     * 0.1f * d2r;
            in.pitch = SIGN_PITCH * rd16(p + 2) * 0.1f * d2r;
            in.yaw   = SIGN_YAW   * rd16(p + 4) * d2r;
            in.att_fresh = 1;
            fc_seen = true;
            break;
        case MSP_RAW_IMU:           // acc[3], gyro[3] (deg/s), mag[3]
            if (fc_parser.size < 12) break;
            in.gyro_x = SIGN_ROLL  * rd16(p + 6) * d2r;
            in.gyro_y = SIGN_PITCH * rd16(p + 8) * d2r;
            break;
        case MSP_ANALOG:            // byte 0: battery in 0.1 V
            if (fc_parser.size < 1) break;
            in.vbat = p[0] * 0.1f;
            break;
        default:
            break;
        }
    }
}

static void fc_send_rc()
{
    // Betaflight applies its channel map (default AETR1234) to MSP frames.
    uint16_t ch[8] = {
        msp_stick_us(out.rc_roll),
        msp_stick_us(out.rc_pitch),
        msp_throttle_us(out.rc_throttle),
        msp_stick_us(out.rc_yaw),
        (uint16_t)((SOLE_RX_MSP && out.arm) ? 2000 : 1000),   // AUX1: arm
        2000,                                                 // AUX2: angle mode
        1000, 1000,
    };
    uint8_t buf[32];
    int n = msp_encode_rc(ch, 8, buf, sizeof(buf));
    Serial1.write(buf, n);
}

// ------------------------------------------------------------ flow module
static void flow_poll()
{
    while (Serial2.available()) {
        if (!msp_parse(&flow_parser, (uint8_t)Serial2.read())) continue;
        const uint8_t *p = flow_parser.payload;
        if (flow_parser.cmd == MSP2_SENSOR_RANGEFINDER && flow_parser.size >= 5) {
            int32_t mm = rd32(p + 1);
            in.tof_range = mm > 0 ? mm * 0.001f : -1.0f;
            in.tof_fresh = 1;
        } else if (flow_parser.cmd == MSP2_SENSOR_OPTIC_FLOW && flow_parser.size >= 9) {
            uint32_t now = micros();
            float dt = (now - flow_last_us) * 1e-6f;
            flow_last_us = now;
            if (dt > 0.001f && dt < 0.2f) {
                in.flow_x = FLOW_SIGN_X * rd32(p + 1) * FLOW_RAD_PER_COUNT / dt;
                in.flow_y = FLOW_SIGN_Y * rd32(p + 5) * FLOW_RAD_PER_COUNT / dt;
                in.flow_quality = p[0];
                in.flow_fresh = 1;
            }
        }
    }
}

// ------------------------------------------------------------------- voice
// The LEDC API changed between arduino-esp32 2.x and 3.x.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void buzzer_begin() { ledcAttach(PIN_BUZZER, 2000, 8); }
static void buzzer_tone(uint32_t hz) { ledcWriteTone(PIN_BUZZER, hz); }
#else
static void buzzer_begin() { ledcSetup(0, 2000, 8); ledcAttachPin(PIN_BUZZER, 0); }
static void buzzer_tone(uint32_t hz) { ledcWriteTone(0, hz); }
#endif

// ------------------------------------------------------------- phone remote
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><meta name=viewport
content="width=device-width,initial-scale=1"><title>fairy</title><style>
body{font:18px system-ui;background:#0e1420;color:#e8eefc;text-align:center}
button{font:inherit;width:44%;margin:2%;padding:22px 0;border:0;border-radius:14px;
background:#2b3a5c;color:#fff}#k{width:92%;background:#c62f3a;padding:34px 0}
pre{text-align:left;margin:4%;color:#9fb0cc}</style>
<button onclick="c('takeoff')">Take off</button><button onclick="c('land')">Land</button>
<button onclick="c('hey')">Hey!</button><button onclick="c('look')">Look there</button>
<button id=k onclick="c('kill')">KILL MOTORS</button><pre id=s></pre><script>
function c(x){fetch('/cmd?c='+x)}
setInterval(()=>fetch('/status').then(r=>r.text()).then(t=>s.textContent=t),500)
</script>)HTML";

static void web_setup()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    web.on("/", [] { web.send_P(200, "text/html", PAGE); });
    web.on("/cmd", [] {
        String c = web.arg("c");
        if (c == "takeoff")   pending_cmd = NAVI_CMD_TAKEOFF;
        else if (c == "land") pending_cmd = NAVI_CMD_LAND;
        else if (c == "hey")  pending_cmd = NAVI_CMD_ATTENTION;
        else if (c == "look") pending_cmd = NAVI_CMD_TARGET;
        else if (c == "kill") pending_cmd = NAVI_CMD_KILL;
        web.send(200, "text/plain", "ok");
    });
    web.on("/status", [] {
        static const char *names[] = {"LANDED", "TAKEOFF", "FOLLOW", "TARGET",
                                      "ATTENTION", "LAND", "FAILSAFE"};
        char b[256];
        snprintf(b, sizeof(b),
                 "state  %s\nfault  0x%02x\nfc     %s\nbatt   %.2f V\n"
                 "height %.2f m\npos    %.2f %.2f\nyou    %.2f %.2f (+-%.2f)\n"
                 "hover  %.2f",
                 names[out.state], (unsigned)out.fault, fc_seen ? "ok" : "NO LINK",
                 in.vbat, out.est_z, out.est_x, out.est_y, out.tgt_x, out.tgt_y,
                 out.tgt_sigma, out.hover_thr);
        web.send(200, "text/plain", b);
    });
    web.begin();
}

// -------------------------------------------------------------------- main
void setup()
{
    Serial.begin(115200);
    Serial1.begin(FC_BAUD, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX);
    Serial2.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, -1);
    leds.begin();
    leds.setBrightness(255);
    buzzer_begin();

    navi_cfg_t cfg;
    navi_default_cfg(&cfg);
    cfg.uwb_enabled = (BUILD_STAGE >= 3) && uwb_begin();
    navi_init(navi, &cfg);
    in.tof_range = -1.0f;

    web_setup();
}

void loop()
{
    static uint32_t next_us = micros();
    static uint32_t tick;
    static float last_buzz = -1.0f;

    fc_poll();
    flow_poll();
    web.handleClient();

    if ((int32_t)(micros() - next_us) < 0) return;
    next_us += 20000;                           // 50 Hz
    if ((int32_t)(micros() - next_us) > 100000) next_us = micros();

    float r;
    if (uwb_poll(&r)) {
        in.uwb_range = r;
        in.uwb_fresh = 1;
    }
    in.dt = 0.02f;
    in.cmd = pending_cmd;
    pending_cmd = NAVI_CMD_NONE;
    if (in.cmd == NAVI_CMD_TARGET) {            // "look there": 1.5 m ahead
        in.cmd_x = out.est_x + 1.5f * cosf(in.yaw);
        in.cmd_y = out.est_y + 1.5f * sinf(in.yaw);
        in.cmd_z = 1.2f;
    }

    navi_step(navi, &in, &out);
    in.att_fresh = in.flow_fresh = in.tof_fresh = in.uwb_fresh = 0;

    fc_send_rc();
    fc_request(MSP_ATTITUDE);
    fc_request(MSP_RAW_IMU);
    if (tick % 25 == 0) fc_request(MSP_ANALOG);

    for (int i = 0; i < LED_COUNT; i++)
        leds.setPixelColor(i, leds.Color(out.led_r, out.led_g, out.led_b));
    leds.show();
    if (out.buzzer_hz != last_buzz) {
        buzzer_tone((uint32_t)out.buzzer_hz);
        last_buzz = out.buzzer_hz;
    }

    if (tick % 10 == 0)                         // 5 Hz bench telemetry over USB
        Serial.printf("st=%d f=%02x rpy=%.1f,%.1f,%.1f gyro=%.2f,%.2f tof=%.2f "
                      "flow=%.2f,%.2f q=%d vbat=%.2f rc=%.2f,%.2f thr=%.2f\n",
                      (int)out.state, (unsigned)out.fault, in.roll * 57.3f,
                      in.pitch * 57.3f, in.yaw * 57.3f, in.gyro_x, in.gyro_y,
                      in.tof_range, in.flow_x, in.flow_y, (int)in.flow_quality,
                      in.vbat, out.rc_roll, out.rc_pitch, out.rc_throttle);
    tick++;
}
