# Design

A palm-sized glowing drone that keeps you company the way a video-game
fairy does: it hovers near you, flits about, darts off to look at things,
bounces and chirps when it wants your attention, and goes home when it is
tired. Inspired by Navi; the behaviours, sounds and code here are original.

## What it does

| Behaviour | Trigger | Motion | Glow | Voice |
|---|---|---|---|---|
| **Follow** | default | Hovers ~0.9 m from you, drifting around you in short flits with pauses, bobbing ±6 cm | pale blue, shimmering at 9 Hz | silent |
| **Look there** | phone button / command | Darts to a point (up to 3 m away), circles it for 10 s | yellow | one blip |
| **Hey!** | phone button, or after N idle seconds | Holds station, bounces ±7 cm at 1.2 Hz for 3 s | flashing green | two-note chirp each second |
| **Where did you go?** | radio says you are > 3 m away | Homes in on distance alone | white | silent |
| **Waiting** | beacon silent > 2 s | Holds position; lands after 15 s | amber pulse | silent |
| **Tired** | battery < 3.5 V | carries on | red-tinted | silent |
| **Going home** | battery < 3.3 V for 2 s, or Land | Backs away from you, descends at 0.35 m/s | red pulse / dim blue | silent |
| **Bedtime** | Go home button, battery < 3.40 V with a pad in range, or beacon lost with a pad in range | Flies to the charging pad, circles it once at knee height, settles onto it | lavender, breathing | two falling notes |
| **Failsafe** | flow, range or FC data lost | Levels out, descends, cuts motors | fast red strobe | silent |

## Architecture

```
            you
   ┌──────────────────────┐          radio transmitter (ELRS)
   │ UWB beacon (stage 3) │          arm / kill / "override" switch
   └──────────┬───────────┘                    │
        distance only                          ▼
              │                    ┌──────────────────────────┐
   ┌──────────▼───────────┐  MSP   │  AIO flight controller   │
   │  XIAO ESP32-S3       │◄──────►│  Betaflight, angle mode  │──► 4 motors
   │  "flight brain" 50 Hz│ sticks │  gyro, ESCs, ELRS RX     │
   │  ─ estimator         │ att.   └──────────────────────────┘
   │  ─ position control  │
   │  ─ behaviours        │◄────── optical flow + laser range (looks down)
   │  Wi-Fi phone remote  │──────► 2 RGB LEDs in a diffuser, piezo
   └──────────────────────┘
```

Two decisions shape everything else:

**Betaflight keeps the fast loops.** Motor control, gyro filtering and
attitude stabilisation are the parts of a multirotor that are hardest to get
right and most dangerous to get wrong, and Betaflight on a whoop board
already does them well. The ESP32 only moves the sticks, 50 times a second,
the way a pilot would in angle mode. If the ESP32 crashes, the human on the
radio still has a normal, flyable drone.

**The brain is portable C with no hardware in it.** `firmware/core` takes a
struct of sensor readings and returns a struct of stick positions, LED colour
and buzzer pitch. The ESP32 firmware and the simulator both call the same
`navi_step()`, so what was simulated is what gets flashed.

### How it knows where it is

- **Height:** a downward laser range finder, tilt-corrected, through an
  alpha-beta filter. A sudden jump in range (> 22 cm in one sample) is
  treated as *the floor changed*, not *I moved*, so flying over a table does
  not make it leap.
- **Position:** optical flow (a tiny downward camera measuring how fast the
  floor slides past), with the drone's own rotation subtracted using the
  gyro, scaled by height, integrated. This drifts slowly, which is fine:
  nothing here needs a map, only "where am I relative to a moment ago".

### How it knows where you are

This is the hard part, and the part the simulation changed most.

You wear an ultra-wideband beacon; the drone measures its distance to it ten
times a second, to about ±10 cm. A distance is not a direction. One
measurement says you are somewhere on a circle around the drone. The drone
has to move and see how the distance changes to work out where on the circle.

- A **particle filter** carries 256 guesses about where you are and how you
  are moving. Each range reading kills the guesses it contradicts. The
  motion model is people-shaped: mostly standing, sometimes setting off in a
  straight line, sometimes stopping.
- The fairy's **flitting is functional**: every sideways drift changes the
  geometry and collapses the circle of guesses to a point. A fairy that held
  perfectly still would go blind.
- A **supervisor that ignores the filter**: if the raw distance exceeds 3 m,
  it stops trusting the estimate and homes in like a moth: fly straight while
  the distance shrinks, turn when it does not. This cannot run away.
- If no guess explains the last four readings, the filter starts over from
  the ring of positions the current reading allows.

## Bill of materials

Masses marked ✓ are from manufacturer or retailer listings; `est.` are my
estimates and need weighing. Prices are approximate (USD, late 2026) and
several could not be confirmed. Research notes with sources are at the end.

| Part | Choice | Mass | ~Price |
|---|---|---|---|
| Airframe | BetaFPV Meteor75 Pro class: 75 mm ducted frame, 1102 22000 KV motors, 45 mm 3-blade props, F4 1S 5A AIO (STM32F411, BMI270, onboard ELRS) | 31.3 g ✓ ready-to-fly; **~27 g est.** with camera, VTX and canopy removed | $100 |
| Battery | 1S LiHV 550 mAh, BT2.0 connector | 14.2 g ✓ | $6 |
| Companion | Seeed XIAO ESP32-S3 (21 × 17.8 mm) | 3–4 g (sources disagree) | $8 |
| Flow + range | MicoAir MTF-01 (PMW3901 + laser, 0.02–8 m, 100 Hz, MSP output) | 4.5 g ✓ | $25 |
| 5 V supply | Micro boost converter, 1S → 5 V, ≥ 300 mA | 0.8 g est. | $4 |
| Glow | 2 × SK6812-mini-3535 + half a 40 mm ping-pong ball | 1.5 g (ball half 1.35 g ✓) | $2 |
| Voice | 9–12 mm piezo disc | 0.3 g est. | $1 |
| Wings | iridescent film + 0.3 mm carbon rod | 0.5 g est. | $2 |
| Wire, tape, glue | 30 AWG silicone | 1.5 g est. | — |
| UWB (stage 3) | Qorvo DWM3000 module (23 × 13 × 2.9 mm, 3.3 V) | 1.5 g est. + 0.5 g carrier | $18 |
| Beacon (stage 3) | Second DWM3000 + any ESP32 board + small LiPo | not flown | $30 |
| Pad beacon (stage 4) | Third DWM3000 + any ESP32 board, USB powered | not flown | $30 |
| Safety pilot | Any ELRS 2.4 GHz radio transmitter | — | $60+ |
| **All-up, stage 3** | | **≈ 56 g** | **≈ $260** |

Why these and not smaller:

- **75 mm, not 65 mm.** A 65 mm whoop gives about 18–20 g of thrust per
  motor against a 31 g stock weight. There is no room for 15 g of payload.
- **MTF-01, not the lighter Matek 3901-L0X** (2 g). The Matek's laser is
  usable to about 1.2 m in practice; this drone flies at 1.5–2 m.
- **Keep the ELRS receiver.** It is already on the board and it is the kill
  switch.

### Mass, thrust, endurance

| | Value | Basis |
|---|---|---|
| All-up mass | 56 g | table above |
| Static thrust | 4 × ~41 g = 163 g at full charge | vendor figure for 1102 + 40 mm props (40–46 g); **not independently tested**, and vendor numbers run high |
| Thrust-to-weight | 2.9 fresh, about 2.2 at 3.7 V under load | thrust ∝ V² |
| Hover throttle | 0.60–0.78 across simulated build tolerances | learned in flight |
| Hover current | ~5.4 A | scaled from claimed flight times; **estimate** |
| Flight time | **~4½ min** to the 3.3 V landing threshold | simulated endurance run: 273 s airborne |

The thrust margin is the tightest number in the design and rests on the
least certain data. Stage 1 exists to measure it on your airframe before you
invest in the rest.

## What the simulation taught us

`sim/` is a 6-DOF rigid-body model with motor lag and saturation, an emulated
Betaflight angle mode, battery sag, draughts, and noisy, delayed, sometimes
missing sensors, including body-blocked radio spikes. It runs the real C
core. See the README for how to run it.

40 simulated "story" flights (stand, walk a loop of the room, get nagged,
inspect something, walk back, land), each on a differently mis-built
airframe: mass −7/+10 %, thrust −18/+15 %, drag, flow-sensor scale ±8 %,
draught strength and battery resistance all varied.

| Metric | Median | 5th–95th pct | Worst |
|---|---|---|---|
| Landed under control | 40 / 40 | | |
| Error in "where you are", whole flight (RMS) | 0.96 m | 0.44–2.27 m | 3.52 m |
| Farthest it got from you | 4.5 m | 2.5–7.0 m | 8.0 m |
| Closest approach while below head height | 0.69 m | 0.66–0.70 m | 0.63 m |
| Altitude error (RMS) | 6 cm | 5–7 cm | 8 cm |
| Peak tilt | 20° | | 20° |
| Peak altitude | 2.09 m | | 2.12 m |
| Hover throttle | 0.68 | 0.61–0.76 | 0.79 |
| Motor saturation | 0 s | | 0 s |
| Touchdown speed | 0.83 m/s | | 0.97 m/s (motors cut ~5 cm up) |

Standing still, the same tracker holds about 15–20 cm error and the fairy
stays 0.7–1.3 m from you.

Three things came out of it that changed the design:

1. **A Kalman filter is the wrong tool for distance-only tracking.** The
   first version used one. It converged on a mirror image of the truth, the
   drone chased its own wrong estimate, and it flew away at 1.3 m/s in every
   run. The particle filter, the people-shaped motion model, the restart
   rule and the homing supervisor each fixed a specific failure seen in
   simulation.

2. **Distance-only tracking is good when you stand still and mediocre when
   you walk.** Standing: about 15 cm error. Walking at 1.1 m/s: it trails by
   1–2 m and can fall 6–8 m behind before homing pulls it back. It is back
   near you within about fifteen seconds of your stopping in most flights, not
   all. This is a property of the sensor, not a tuning problem: while you
   both move in parallel, a mirror-image track on the drone's other side
   explains the distances equally well.

3. **So it must not fly at head height near you.** With the filter
   occasionally confident and wrong, shoulder-height following produced a
   near-collision (< 10 cm) in roughly 1 flight in 25. The fix is a rule
   rather than more tuning: when following a beacon, everything happens at
   2 m, and it backs away before descending to land. Shoulder-height flight
   is kept for the no-beacon mode, where it orbits a fixed spot.

To get a fairy that sits by your shoulder while you walk, add a sensor that
gives **direction**: a UWB chip with angle-of-arrival, or a small camera
tracking an IR beacon. The core's interface would take a bearing as one more
input; the particle filter already supports any likelihood.

## Going home to a charging pad

Flying on sunlight is out by a factor of about 50 (hover needs ~20 W; the
usable top surface would collect ~0.4 W in direct sun, milliwatts indoors).
A pad that charges the drone between flights, solar-fed or not, is the
workable version, and for that the drone has to land on it.

**How it finds the pad.** The pad carries its own UWB beacon. A fixed beacon
is the easy case for distance-only tracking: every range taken from a new
place narrows it down for good. A second, simpler particle filter does this
all flight long, so the fix is already there when it is needed.

**How it lands on it.**

1. *Transit* at cruise height to the near side of the pad (homing on raw
   distance if the fix is poor).
2. *Survey:* drop to 45 cm above the pad and fly one full circle of 0.7 m
   radius. Down low the distance is almost purely horizontal, so it locates
   the pad well, and going all the way round makes a constant ranging bias
   cancel instead of pushing the answer to one side.
3. *Move in:* freeze the fix and slide over it. Directly overhead a range
   says nothing about sideways error, so it stops listening.
4. *Settle:* straight down at 0.25 m/s.

It will not descend while the person's beacon is within 1.15 m, and climbs
back up if they walk in during the survey: the circle is at knee height and
the drone cannot see anyone, only measure how far away they are.

**Simulated result**, 40 flights with the pad 1.6–4 m away in a random
direction and airframe and sensors varied:

| | |
|---|---|
| Completed the approach | 40 / 40 |
| Miss distance | median 4 cm, 95th percentile 12 cm, worst 15 cm |
| Inside a 20 / 30 / 40 cm pad | 37 / 39 / 40 |
| Command to touchdown | median 21 s, worst 31 s |

So build the pad as a **shallow funnel about 40 cm across** that the skids
slide down, with charging contacts at the bottom.

**What it costs.** Heading home at 3.40 V instead of landing at 3.30 V
shortens a flight from about 4½ minutes to about 3¾ (229 s in simulation,
ending on the pad). If the reserve runs
out on the way it lands where it is.

**Not designed yet:** the pad itself (funnel, contacts on the skids, a
LiHV-safe 1S charger, the solar panel and its buffer battery), and taking
off again by itself once charged. The pad also needs the unwritten UWB
driver, ranging two beacons in turn. As a guide, the battery holds about
2 Wh, so a 5–10 W panel in good sun could refill it in roughly 20–40
minutes.

### What the simulation does not cover

- Walls, furniture, ceilings and other people. **The drone has no obstacle
  sensing.** The "falls 6–8 m behind" figure assumes an empty hall.
- Prop wash near surfaces, ground effect, ceiling suction.
- Real optical-flow failure modes beyond drop-outs: low light, glossy or
  repeating floors, the drone's own LED lighting the floor.
- Gyro/flow timing mismatch finer than one 20 ms tick.
- Wi-Fi and UWB radios sharing a board and a 5 A-pulsing power rail.
- The airframe numbers are estimates; the Monte Carlo spreads them but
  cannot tell you they are centred correctly.

## Safety design

- Ducted props, under 60 g.
- Human kill switch on a separate radio, independent of the ESP32.
- Never below 1.85 m within reach of the tracked person in follow mode;
  0.55 m keep-out bubble and path routing around the person; backs off
  before landing.
- Take-off refused on a flat battery or with stale sensors; gives up after
  7 s if it has not lifted.
- Lands itself on low battery, lost beacon, lost flow, lost range, lost
  flight-controller link. Tilt limited to 30°, climb to 2.2 m.
- Will not descend onto the pad while the tracked person is within 1.15 m.
- Every one of those is an automated test in `tests/test_flight.py`.

## Research notes

Collected by web search on 2026-11-17. Many manufacturer pages could not be
opened directly from the build environment, so several figures are from
search-result snippets of those pages rather than the pages themselves.

- Meteor75 Pro: 31.33 g without battery, 1102 22000 KV, 45 mm props, F4 1S
  5A AIO, claimed 6:40 gentle flight on 550 mAh (betafpv.com,
  oscarliang.com). Bare weight without FPV gear: not found.
- F4 1S 5A AIO: STM32F411, BMI270 (V3), 2.96 g; reviews of the original
  report one full UART plus one TX-only pad. **Check the pad layout of the
  revision you buy.**
- 1102 + 40 mm prop thrust: 40.6 g and 46.3 g for two unlabelled KV versions
  (BetaFPV product page). No independent test found.
- 0802 + 31 mm: 18.0–19.5 g per motor in an older independent test
  (oscarliang.com/tiny-whoop-prop).
- LAVA 550 mAh 1S: 14.0–14.3 g, 68 × 15.5 × 6.7 mm.
- MTF-01: read from the manufacturer's manual
  (github.com/micoair/MTF-01_USER_MANUAL): 4.5 g, 0.02–8 m, 100 Hz,
  4.0–5.5 V, MSP output option.
- Matek 3901-L0X: ArduPilot's wiki recommends capping its range at 1.2 m.
- MSP sensor frames, read from INAV source: `MSP2_SENSOR_RANGEFINDER`
  0x1F01 `{u8 quality; i32 distance_mm}`, `MSP2_SENSOR_OPTIC_FLOW` 0x1F02
  `{u8 quality; i32 motionX; i32 motionY}`.
- Betaflight, read from source and docs: `MSP_SET_RAW_RC` is N × uint16 µs
  and follows `rcmap`; `MSP_ATTITUDE` is roll/pitch in 0.1°, yaw in whole
  degrees; `MSP_RAW_IMU` gyro is in deg/s; `MSP_ANALOG` byte 0 is battery in
  0.1 V; `angle_limit` defaults to 60°; MSP OVERRIDE with
  `msp_override_channels_mask` is the recommended way to fly with a
  companion computer.
- DWM3000: 23 × 13 × 2.9 mm, 2.5–3.6 V, tens of mA while ranging; ~10 cm
  accuracy after antenna-delay calibration. Mass not found.
- Ping-pong ball: 2.7 g (ITTF rules).
