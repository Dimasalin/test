// Everything you are likely to change when building one.
#pragma once

// ---- pins (Seeed XIAO ESP32-S3 silk-screen names -> GPIO) ----
#define PIN_FC_RX      44   // D7  <- flight controller TX (MSP)
#define PIN_FC_TX      43   // D6  -> flight controller RX
#define PIN_FLOW_RX     4   // D3  <- MTF-01 TX (the module only talks)
#define PIN_LED         2   // D1  -> SK6812 data in
#define PIN_BUZZER      3   // D2  -> piezo disc (other leg to GND)
#define PIN_UWB_CS      5   // D4     DW3000 chip select (stage 3)
#define PIN_UWB_IRQ     6   // D5     DW3000 interrupt   (stage 3)
// DW3000 SPI uses the default bus: D8 = SCK (7), D9 = MISO (8), D10 = MOSI (9)

#define LED_COUNT       2

// ---- build stage ----
// 2: no beacon. The fairy orbits its take-off spot at shoulder height.
// 3: UWB beacon worn by the companion. The fairy follows, above head height.
#define BUILD_STAGE     2
// 1: there is a charging pad with its own UWB beacon; "Home" flies to it and
//    a low battery sends the fairy there by itself. Needs the UWB driver.
#define PAD_ENABLED     0

// ---- flight controller link ----
#define FC_BAUD         115200
// 0 (recommended): Betaflight keeps its own radio receiver; this board only
//    overrides roll/pitch/yaw/throttle while the pilot holds the MSP OVERRIDE
//    switch. Arming and the kill switch stay on the radio.
// 1 (bench only, props off): Betaflight "feature RX_MSP"; this board is the
//    only receiver and arms via AUX1.
#define SOLE_RX_MSP     0
// Signs of the angles Betaflight reports. Check on the bench (see BUILD.md):
// tilting the nose DOWN must print a POSITIVE pitch, right side DOWN a
// POSITIVE roll, and turning left (anticlockwise from above) increasing yaw.
#define SIGN_ROLL       (+1.0f)
#define SIGN_PITCH      (-1.0f)
#define SIGN_YAW        (-1.0f)

// ---- optical flow (MicoAir MTF-01 in "MSP" output mode) ----
#define FLOW_BAUD       115200
// rad of ground motion per raw count; calibrate with the 1 m slide test.
#define FLOW_RAD_PER_COUNT   (1.0e-2f)
#define FLOW_SIGN_X     (+1.0f)
#define FLOW_SIGN_Y     (+1.0f)

// ---- wifi remote ----
#define WIFI_SSID       "fairy"
#define WIFI_PASS       "heylisten"   // change me (8+ characters)
