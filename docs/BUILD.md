# Building it

Build in three stages. Each one flies on its own, and each one is the test
rig for the next. Do not skip ahead: stage 1 is where you find out whether
your airframe has the thrust margin the rest depends on.

| Stage | You add | What you get |
|---|---|---|
| 1 | Airframe + flow sensor + ESP32 | Hands-off hover. Take off, hold position, land from your phone. |
| 2 | LEDs, diffuser, wings, piezo | A fairy: flits around its take-off spot, glows, chirps, darts to "look there". |
| 3 | UWB radio on the drone + beacon you wear | It follows you (above head height). **UWB driver still to be written**, see below. |
| 4 | A charging pad with its own beacon | It takes itself to bed when told or when the battery is low. Flight logic done and simulated; pad hardware not designed. |

Parts and masses are in [DESIGN.md](DESIGN.md#bill-of-materials).

## Safety first

- **Props off** for every bench step. The ducts make this small drone
  forgiving, not harmless: wear eye protection for first flights.
- Keep a radio transmitter bound and in your hand, with **arm** and **MSP
  override** on switches. Letting go of the override switch gives you back
  manual control; disarm stops the motors whatever the ESP32 is doing.
- First flights in a clear room, lit (the flow sensor needs light and a
  textured floor; glossy or plain dark floors blind it), no pets, no mirrors
  underneath, nothing above 2.4 m to get sucked into.
- 1S LiHV cells: charge to 4.35 V only on a LiHV charger, never unattended,
  and stop flying at 3.3 V under load. The firmware lands itself there.
- Under 250 g, so no registration in most places for recreational flying.
  Indoors is simplest; check local rules before flying outdoors.

## Stage 1: hover

### 1. Wire it

```
1S battery ──► AIO flight controller (motors, gyro, ELRS receiver, Betaflight)
     │              │ spare UART: TX ──► XIAO D7 (GPIO44, RX)
     │              │             RX ◄── XIAO D6 (GPIO43, TX)
     │
     └─► 5 V boost ─┬─► XIAO ESP32-S3  5V pin
                    └─► MTF-01 flow/range  VCC      MTF-01 TX ──► XIAO D3 (GPIO4)
All grounds common.
```

- The flow module wants 4.0–5.5 V and a 1S cell sags to 3.3 V, so both it and
  the XIAO run from a small 5 V boost converter fed from the battery pads.
- The stock Meteor75-class AIO board has **one** full spare UART. It goes to
  the ESP32; the flow module talks to the ESP32, not to Betaflight.
- Mount the flow module underneath, lens down, connector toward the tail.
  Mount the XIAO on top with foam tape, USB-C reachable.
- Remove the camera, VTX and canopy. You need the grams.
- **Weigh it** with battery. Budget is 56 g; over 60 g, fix that first.

### 2. Configure Betaflight (props off)

1. Ports tab: enable **MSP** on the UART wired to the ESP32, 115200 baud.
2. Modes tab: **ANGLE** always on (or on the same switch as override).
   **ARM** on a switch. **MSP OVERRIDE** on a switch.
3. CLI:
   ```
   set msp_override_channels_mask = 15     # roll, pitch, yaw, throttle
   set angle_limit = 30                    # must match bf_angle_limit in navi.h
   save
   ```
   `angle_limit` is per PID profile. The firmware assumes full stick = 30°.
4. Receiver tab: move the ESP32-driven channels and confirm the channel map
   is the default `AETR1234`.

Betaflight's documentation calls flying with MSP as the *only* receiver
"bench use only", and this design agrees: keep the real receiver. Note that
in override mode some Betaflight versions keep the last MSP values if the
companion goes silent, so the override switch in your hand is the safety, not
a timeout.

### 3. Flash the ESP32

```
cd firmware/esp32
pio run -t upload
pio device monitor
```

Edit `src/config.h` first if your pins differ. `BUILD_STAGE` stays at 2.

### 4. Bench checks (props off, USB connected)

The monitor prints one line five times a second. Go through these in order;
each one catches a mistake that would otherwise be a crash.

| Check | Do this | Must see | If not |
|---|---|---|---|
| FC link | Power the drone | `rpy=` changes when you tilt it | TX/RX swapped, or MSP not enabled on that UART |
| Roll sign | Right side down | roll **positive** | flip `SIGN_ROLL` |
| Pitch sign | Nose down | pitch **positive** | flip `SIGN_PITCH` |
| Yaw sign | Turn left (anticlockwise seen from above) | yaw **increasing** | flip `SIGN_YAW` |
| Gyro sign | Roll right quickly | `gyro=` first value positive during the motion | sign constants again |
| Range | Hold 0.5 m above the floor | `tof=0.50` ± 3 cm | module not in MSP output mode, or wrong pin |
| Flow sign X | Hold level at 0.5 m, slide it **forward** | `flow=` first value positive | flip `FLOW_SIGN_X` |
| Flow sign Y | Slide it **left** | second value positive | flip `FLOW_SIGN_Y` |
| Flow scale | See below | | |
| Battery | | `vbat=` matches a multimeter ± 0.1 V | |
| Sticks | Phone: join Wi-Fi `fairy`, open `http://192.168.4.1`, press Take off, hold the override switch | In Betaflight's Receiver tab throttle rises slowly, roll/pitch stay centred | channel map |
| Stick sign | During that, slide the drone backward by hand | Betaflight pitch channel goes **up** (it pushes forward to return) | a sign above is wrong |

**Flow scale.** Hold the drone level at exactly 0.5 m and slide it 1.0 m
forward in about 2 seconds. Add up `flow_x × 0.5 m × 0.2 s` over the printed
lines; it should come to 1.0 m. Scale `FLOW_RAD_PER_COUNT` by
`1.0 / what you got`. The simulator tolerates ±8 % error here.

### 5. First hover (props on)

1. Put it on a textured floor, 0.7 m in front of you, nose away from you.
2. Arm on the radio with throttle low. Hold the override switch.
3. Press **Take off** on the phone. It spools for a second, then climbs to
   1.5 m and holds.
4. Look at `hover` on the phone page. That is the learned hover throttle.
   **Above 0.75 you do not have enough thrust margin**: lighten it, or use a
   fresh pack, before going further. The simulation's weakest passing
   airframes hover at 0.78.
5. Press **Land**. It descends at 0.35 m/s and cuts the motors about 5 cm up.

If it drifts in a growing circle, a flow or attitude sign is wrong. If it
bobs up and down, lower `kv_z`. If it "toilet-bowls" slowly, the flow module
is rotated relative to the frame.

## Stage 2: make it a fairy

- **Glow:** two SK6812-mini LEDs back to back on a sliver of protoboard:
  5 V, GND, data from XIAO D1 through a 330 Ω resistor. Cover with half a
  ping-pong ball (1.35 g) glued over the top. At full white each LED draws
  about 36 mA.
- **Wings:** two pairs cut from iridescent cellophane or 12 µm mylar, a hair
  of 0.3 mm carbon rod along the leading edge, glued to the top of the
  diffuser clear of the ducts. Keep them under 0.5 g and out of the prop
  wash or they will flutter into the props.
- **Voice:** a 9–12 mm piezo disc between XIAO D2 and GND. The firmware plays
  its own short chirps. Record and add your own sounds if you want speech;
  none are included.
- Re-weigh, re-check hover throttle.

Now the phone buttons do something: **Hey!** makes it bounce, flash green and
chirp; **Look there** sends it 1.5 m ahead to circle a spot in yellow. With no
beacon it treats the spot 0.7 m behind its take-off point (you) as its
companion and flits around that at shoulder height, so stand still or stay
out of its orbit.

## Stage 3: follow me

The flight brain's side of this is complete and is what the simulator
exercises: a particle filter that works out where you are from distance
alone, plus a supervisor that homes in on the raw distance when the filter
is wrong.

What is **not** done is `firmware/esp32/src/uwb.cpp`, the driver that
produces those distances. It needs:

1. A Qorvo DWM3000 module on the drone (SPI: D8/D9/D10, CS D4, IRQ D5, 3.3 V)
   and a second DWM3000 on any ESP32 board in your pocket or on a lanyard at
   chest height (1.25 m; set `beacon_z` if different).
2. A **non-blocking** double-sided two-way-ranging initiator giving about 10
   ranges per second. The `Fhilb/DW3000_Arduino` examples
   (`dw3000_doublesided_ranging_ping` / `_pong`) are the starting point: the
   pong sketch runs on the beacon as is, the ping sketch has to become a
   state machine so it never stalls the 50 Hz loop.
3. Antenna-delay calibration: hold the two radios exactly 1.00 m apart and
   trim until it reads 1.00 m. The filter assumes ±15 cm.

Then set `BUILD_STAGE 3`. Expect what the simulation shows: it stays within
about a metre while you stand, falls a few metres behind while you walk, and
finds you again when you stop. It flies at 2 m because it is sometimes
confidently wrong about where you are. Ceilings under 2.4 m are too low for
this mode.

## Stage 4: a pad to go home to

The flight logic is done: **Go home** on the phone page, a battery below
3.40 V, or losing your beacon all send it to the pad, where it circles once
at knee height and settles. In simulation it lands within 15 cm of the pad
centre every time (median 4 cm), in about 20 seconds.

To build it you need:

1. A third DWM3000 + ESP32 at the centre of the pad, antenna about 5 cm above
   the floor (`pad_z`), running the same responder as your beacon.
2. The UWB driver from stage 3, extended to range the two beacons in turn
   (`uwb_poll_pad()` in `uwb.cpp`).
3. A shallow funnel about 40 cm across for the skids to slide down.
   Cardboard is fine for finding out whether it works.
4. `#define PAD_ENABLED 1` in `config.h`.

Put the pad on the floor with 1.5 m clear around it, and stand back: it
will not come down while your beacon is within 1.15 m. Do not put the pad on
a table yet; the survey circle would fly off the edge and the height logic
has only been simulated for a pad at floor level.

Charging contacts, the charger and any solar panel are yours to design;
nothing here covers them. Never charge a LiPo unattended on a home-made pad.

## What has and has not been tested

| | Status |
|---|---|
| Flight brain (`firmware/core`) | Simulated: 42 automated tests, Monte Carlo over build tolerances |
| MSP encode/parse | Unit tested against reference bytes and CRC check value |
| ESP32 firmware | Syntax-checked on a PC against stub Arduino headers only. **Not compiled for the ESP32** (toolchain download was blocked where this was written) and **never run on hardware** |
| Betaflight settings above | From Betaflight source and docs, not flown |
| Pin assignments, sign constants | Best guesses; the bench checks exist to correct them |
| UWB driver | Not written (needed for follow-me and for the pad) |
| Charging pad hardware | Not designed |
| A physical drone | Not built. This was designed and simulated in software only |
