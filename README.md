# RC Heli Field

A 600-class flybarless RC helicopter on a flying field, seen from the pilot station. You fly it with two
on-screen gimbals (Mode 2), the keyboard or a USB transmitter / gamepad, or let the autopilot take off,
hold, fly racetrack laps, land or demonstrate an autorotation. Crash protection, a low-battery failsafe
and a REAL sensor mode (the flight controller flies on noisy, delayed sensors) are modelled.

This is a port of the single-file JavaScript page (`rc-heli-field.html`, three.js r128) to the same
architecture as the circuit autopilot project: a Rust core compiled to WebAssembly owns all simulation,
control, autopilot, sensor and rule logic, and a TypeScript page only renders, gathers input and calls
the exports. The built page is one self-contained HTML file of about 114 KB.

## Layout

| Path | What |
|---|---|
| `rust/` | crate `helisim`: lib (rlib + cdylib) and a native `helisim` bin for quick runs |
| `rust/tests/suites.rs` | native ports of the JS harnesses test.js .. test5.js, with asserts |
| `web/page.html` | all CSS and DOM of the original page, three.js r128 from cdnjs, `/*WASM*/` and `/*BUNDLE*/` placeholders |
| `web/src/` | TypeScript UI (below) |
| `web/build.sh` | Rust -> wasm32, wasm-opt, tsc, esbuild, embed -> `rc-heli-field.html` |
| `web/test/smoke.mjs` | Node smoke test of the built page (wasm via the page loader + bundle in a DOM/THREE stub) |
| `web/test/browser.mjs` | headless Chromium test of the built page, desktop and phone width |
| `rc-heli-field.html` | the built page |

### Rust modules

| Module | Role |
|---|---|
| `math` | Vec3 / quaternion with three.js r128 semantics (rotation, multiply order, slerp), heading helpers |
| `units` | G, DEG, governed headspeed 2350 rpm |
| `rng` | seeded xoshiro256** (replaces Math.random), Box-Muller `randn` |
| `wind` | CALM / BREEZE / GUSTY presets and the deterministic gust pattern |
| `heli` | rigid body, rotor aerodynamics, powertrain, battery drain, skid contact, crash detection |
| `fbl` | rotor head (flapping disc, servos, hub moment, flapback, head bob) and the flybarless gyro: rate loop, self-level, heading-hold tail |
| `stunt` | STUNTS: pirouette, loop, roll, flip, inverted hover; profiles, scoring, cue text |
| `sensors` | gyro, accelerometer, magnetometer, rangefinder, barometer, GPS, ESC rpm and battery telemetry, each on its own clock |
| `estimator` | BASIC complementary filter; `Est` is what every controller sees |
| `ekf` | error-state extended Kalman filter (default in REAL mode), fixed-size 16-state matrix code, gating, GPS reset, health |
| `autopilot` | takeoff, hold, circuit path follower, land, rescue, autorotation, battery failsafe |
| `sim` | world step, commands, landing grading, toasts as an event list, HUD, cue and sensor-panel text |
| `jsonw` | tiny JSON writer and JS-style `toFixed` (no serde_json, no core::fmt float code) |
| `wasm` | C-ABI exports, one `Sim` in a thread_local RefCell |

### TypeScript

| File | Role |
|---|---|
| `engine.ts` | base64 -> DecompressionStream('gzip') -> WebAssembly.instantiate; wraps the exports; `Frame` type; command codes |
| `input.ts` | touch gimbals, keys, gamepad axes (raw values only) |
| `view3d.ts` | three.js scene, heli model, ghost, windsock, pilot / chase / pad-B cameras |
| `hud.ts` | readouts, cue, sensor panel, toasts, button labels and states, all from the frame |
| `main.ts` | wiring and the frame loop |
| `three.ts` | the global `THREE` from the script tag, typed with @types/three 0.128 |

### wasm interface

Only plain numbers cross the boundary.

| Export | |
|---|---|
| `sim_init(seed)` | new sim with a seed |
| `sim_input(yaw, pit, rol, thrDelta, thrSet, gp, g0, g1, g2, g3)` | this frame's input; `thrSet` < 0 means the touch throttle did not move; `gp` = 1 with gamepad axes 0..3 |
| `sim_cmd(code)` | 0 arm, 1 hold, 2 protect, 3 sensors, 4 level, 5 pitch, 6 wind, 7 cam, 8 reset, 9 takeoff, 10 circuit, 11 land, 12 autorotate, 13 takeover (a stick was touched), 14 estimator EKF / BASIC, 15 stunt picker toggle, 16..20 fly pirouette / loop / roll / flip / inverted hover, 21 close the picker (codes 0..14 unchanged) |
| `sim_step(n)` | n fixed steps of 1/240 s |
| `sim_frame()` | builds the JSON frame, returns its length; read it at `sim_out_ptr()` |

Per animation frame the page sends input, runs the elapsed fixed steps (at most 24, the remainder is
dropped as before), reads one frame and renders it. The frame carries true and estimated pose, rotor
rpm, wind, settings (including `ekf` and `sig2`, the 2-sigma horizontal uncertainty radius drawn as a ring around the ghost), the autopilot mode code, the gimbal positions to draw, the HUD strings with their
colour classes, the cue line, the sensor panel and the toasts raised since the last frame. All text is
decided in Rust; the page writes it to the DOM.

## Models

### Rotor and powertrain

| | |
|---|---|
| Collective thrust | `coll * TMAX * rpm^2 * ground effect * ETL * (1 - 0.45 * VRS)`, TMAX = 2.2 g per unit mass |
| Ground effect | up to +18% with the disc on the ground, gone at 1.4 m of disc height |
| Translational lift (ETL) | up to +15%, smoothstep from 1.5 to 6 m/s of edgewise airspeed |
| Settling with power (VRS) | severity from inflow 2..6.5 m/s and inplane speed under 2.5 m/s, only with collective * rpm^2 > 0.25; builds with a 1.2 s lag, washes out in 0.35 s |
| Disc damping | `0.55 * rpm * (1 - VRS) * (1 + 0.1 * inplane)` along the shaft; this is what holds it up in autorotation |
| Body drag | `(0.22 + 0.035 * airspeed)` per unit mass, 30% of that on the ground |
| Rotor drag | `0.4 rpm^2 (0.22 + 2 coll^2) + 0.04 rpm` |
| Autorotative drive | `(0.02 * inflow + 0.004 * inplane * min(1, 2 rpm)) * (1 - 0.6 VRS)` |
| Rotor inertia | 2.0 (divides the rpm rate) |
| Governor | `3 * I * (1 - rpm_measured) + drag(rpm_measured) - autodrive`, limited to 0.85 * (0.72 + 0.28 * min(1, charge / 0.3)) |
| Soft start | spool-up power limited to `0.33 * I + drag` until 97% headspeed |
| Headspeed sag | from the power limit: about 96% at full collective on a full pack, 90% at 15% |
| Battery drain | `0.0003 + 0.0079 * motor * rpm` per second |

### Rotor head: blade flapping (`fbl::rotor`)

The cyclic no longer sets body rates directly. The swashplate moves the disc, and the disc moves the body.

| | |
|---|---|
| Servos | slew 3.2 rad of disc tilt per s (0.08 s / 60 deg class: full 0.2 rad cyclic in about 60 ms), 0.0015 rad slop / deadband |
| Flapping | first order, tip-path plane relative to the shaft: `tau a' = sw + bb - tau (w + c x w) - a`, `tau = 16 / (gamma Omega) / rpm`, gamma 2.0, Omega 246 rad/s: **32.5 ms at full headspeed**, 54 ms at 60% |
| Disc lag cross-coupling | `c = gamma / 16 * 0.65 = 8%` of the on-axis lag (residual after the head's 90 deg phase lag) |
| Flapback | `0.0011 (coll + 0.15) rpm` rad per m/s of edgewise airspeed, tilts the disc away from the relative wind (replaces the old ad-hoc blowback rate) |
| Lateral flapping | `0.00035 * thrust_g` rad per m/s, from coning / inflow dissymmetry: pitch-roll coupling in forward flight |
| Hub moment | `(420 rpm^2 + 130 thrust_g) * a`, at least `84 rpm^2`, angular acceleration per rad of disc tilt (hub spring + thrust offset), body rate damping 0.5 /s, 70% less with the weight on the skids |
| Thrust | along the disc normal, not the shaft |
| Head bob | 7 Hz mode, damping ratio 0.25, driven by swashplate rate x thrust; a small wobble when cyclic is stirred fast |

### Attitude (flybarless gyro)

Self-level (and all autopilot modes) command up to 32 degrees of tilt from the right stick and turn the
tilt error into body rates (gain 7, 12 in a rescue, inverted recovery). Rate mode commands up to 4.2 rad/s
of pitch and roll and 5.2 rad/s of yaw. Stunts send body-rate demands straight to the rate loop.

The rate loop is a real controller now (`fbl::gains`). A reference model `w'' + w'/tau + L w = L w_des`
(L = hub stiffness at the estimated rpm and thrust) gives the rate the head can deliver. Cyclic is
feed-forward `(tau + 0.5 / L) w_des` plus P 0.02 and I 0.30 (clamped at 0.06 rad) on the reference
minus the measured / estimated rate. The integrator bleeds off on the skids.

Step response, `cargo test --release --test step -- --nocapture` (IDEAL, calm air, 20 m hover):

| | before (direct rate law) | after (flapping + FBL) |
|---|---|---|
| 30 deg/s roll-rate step, rate mode | rise 0.125 s, overshoot 0%, settle 0.167 s | rise 0.104 s, overshoot 2.9%, settle 0.138 s |
| 20 deg self-level roll step | rise 0.221 s, overshoot 1.8%, settle 0.296 s | rise 0.167 s, overshoot 1.9%, settle 0.250 s |

The first try (no reference model, I-term on the raw demand) overshot 29% with a 1 s tail, so the reference model is there for a reason.
Flips and loops now lag slightly at entry and exit and show flapback and cross-coupling, but the attitude loop holds them within a few degrees.
All the old suites pass unchanged.

The tail is a heading-hold gyro, P 22 + I 30 (I clamped to 0.6), limited to `45 rpm^2`,
against motor torque reaction (`8 * motor`, the yaw kick when power changes), weathervaning
(`0.5 * sideways airflow`) and fin damping. Wind
buffet, transition shudder at 2..4.5 m/s and vortex-ring shake add random rate noise in the air.

### Ground contact and crashes

Four skid points. On the ground, static friction 1.0 and kinetic 0.8 against the weight still on the
skids; the yaw integrator bleeds off and the skids stop pirouettes once 60% of the weight is on them.
A crash is a touchdown faster than 3 m/s, more than 35 degrees of tilt with the rotor turning, or more
than 3.2 m/s of sideways slide. Landings after more than 0.8 s in the air are graded Butter-soft
(< 0.5 m/s), Soft (< 1), Firm (< 2) or Hard, and the best on-pad landing is kept.

### Wind

CALM 0, BREEZE 2.2 m/s, GUSTY 4.5 m/s from the pilot's right, with a gust pattern of
`base * (0.35 sin(0.7t) sin(0.23t + 2) + 0.25 sin(1.9t + 1))` and a 30% crosswind component.

### Sensors (REAL mode)

| Sensor | Rate | Model |
|---|---|---|
| Gyro | 240 Hz | 0.6 deg/s noise, bias random walk from +-0.5 deg/s, rotor vibration, 40 Hz low-pass |
| Accelerometer | 240 Hz | specific force, 0.05 m/s2 noise, 1.5 m/s2 vibration, 20 Hz low-pass |
| Magnetometer | 50 Hz | 0.9 deg noise plus up to 4 deg from motor current |
| Rangefinder | 20 Hz | 0.05..4 m, 2 cm noise, drops out above 25 degrees of tilt |
| Barometer | 25 Hz | 12 cm noise, slow drift, ground-effect downwash error, gust error |
| GPS | 10 Hz, 150 ms latency | 0.4 m horizontal and 0.8 m vertical noise, wandering bias up to 1.2 m, 2..4 m glitches every 2..3 min, 11..17 sats |
| ESC rpm | 10 Hz, 100 ms latency | 10 rpm steps, 0.6% noise, 2% dropped frames |
| Battery | 10 Hz | 12S, OCV curve 3.2..4.2 V per cell, 0.025 ohm pack, 0.6 s low-pass, charge from load-compensated voltage with a slightly low 0.02 ohm estimate |

Two estimators run side by side on the same samples; ESTIMATOR (key E) picks the one the controllers
read. Both are ignored in IDEAL mode, which hands the controllers the true state. Physics always runs
on the truth.

BASIC (the original complementary filter) integrates the gyro and corrects it toward gravity from the
accelerometer (minus the manoeuvre acceleration derived from GPS velocity), with a gyro-bias
integrator, fuses compass heading slowly, dead-reckons velocity and position from the accelerometer
and corrects with the rangefinder (preferred below 3 m), the barometer and the delayed GPS fix,
matched against a 0.5 s history of past estimates with a soft innovation gate against jumps.

### EKF (default)

`rust/src/ekf.rs`, an error-state (indirect) extended Kalman filter in the style of a small ArduPilot
EKF / PX4 ECL. No dependencies: 16x16 arrays, a sparse F P F' product and a 3x3 elimination for the
innovation tests. It adds about 2.6 microseconds per 240 Hz step in wasm (6.0 against 3.4).

| | |
|---|---|
| Nominal state | position, velocity (world), attitude quaternion, gyro bias, accelerometer bias, baro offset |
| Error state (16) | dp, dv, dtheta (world-frame rotation error), dbg, dba, dbaro |
| Predict, 240 Hz | bias-corrected gyro and specific force rotated to world plus gravity; F = I + A dt with the [f]x and -R couplings; Q from the sensor models |
| GPS, 10 Hz | position and velocity measured 150 ms earlier: innovations are formed against a 0.5 s buffer of past nominal states at the measurement time; the correction applies to the current state and is also applied to the buffer |
| Barometer, 25 Hz | height + offset state; noise inflated while the disc is in ground effect (< 1.4 m) |
| Rangefinder, 20 Hz | slant range converted with the filter's own attitude, tilt < 25 deg, 0.05..4 m, flat terrain y = 0 |
| Magnetometer, 50 Hz | yaw-only fusion (no gain into roll / pitch), compass-motor compensation 3.6 deg per unit motor load (10% under the real 4 deg), residual noise inflated with load |
| Gravity aiding, 10 Hz | 0.1 s mean specific force against R' g, only when the magnitude is within 0.25 m/s2 of g and rates are under 0.35 rad/s; updates tilt and accel bias, not heading |
| Updates | sequential scalar updates, Joseph form `(I-KH) P (I-KH)' + K R K'`, error state injected into the nominal state after each, symmetry enforced and variance limits applied after every predict and update |
| Gating | normalised innovation squared against a chi-square gate on every update; test ratio = NIS / gate, > 1 rejected |
| GPS reset | horizontal position rejected for more than 2 s: position and velocity jump by the mean of the last 5 rejected innovations, covariance reset to 0.7 m / 0.5 m/s 1-sigma. The autopilot shifts its anchor and autorotation entry point by the same jump (like ArduCopter's EKF xy-reset handling), so the heli does not fly to the new estimate |
| Initialisation | on the ground once accelerometer, compass, baro and the first GPS fix are in: tilt from the averaged accelerometer, heading from the compass, height 0.29 m, baro offset from the first baro sample. A non-finite state reinitialises it |
| Health | status OK / RANGEFINDER / GPS REJECTED / GPS RESET / BARO ONLY, test ratios per sensor, rejected counts, 1-sigma position / height / tilt / heading, shown in the sensor panel (status, four test ratio bars that fill to the gate, 1-sigma position and heading, rejected GPS samples) |

Tuning (`ekf::tune`), all 1-sigma:

| | value | from the sensor model |
|---|---|---|
| gyro noise | 0.6 deg/s + 1.73 deg/s x rpm^2 vibration, x2 | 0.6 deg/s white, uniform +-3 deg/s x rpm^2 vibration (40 Hz low-pass not modelled, hence x2) |
| accel noise | 0.05 m/s2 + 0.87 m/s2 x rpm^2, x2 | 0.05 white, uniform +-1.5 x rpm^2 vibration |
| gyro bias walk | 0.03 deg/s/sqrt(s) | 0.02 |
| accel bias walk | 0.003 m/s2/sqrt(s) | none modelled |
| baro offset walk | 0.04 m/sqrt(s) | 0.03 |
| position walk | 0.02 m/sqrt(s) | |
| GPS horizontal position | 0.6 m | 0.4 m noise plus the wandering bias |
| GPS vertical position | 19 m | the vertical bias is a 40 s Gauss-Markov error (~0.67 m); fused at 10 Hz as white noise it would be averaged into the height, so its white-equivalent sigma_b sqrt(2 tau / dt) is used and GPS height is only a weak long-term reference |
| GPS velocity | 0.15 m/s horizontal, 0.3 m/s vertical | 0.12 / 0.25 |
| GPS bias consider term | 0.45 m per axis added to the reported horizontal sigma | the bias is not observable without another absolute reference |
| baro | 0.2 m, + 0.5 m in ground effect | 0.12 m noise, gust error, 0.4 m downwash error |
| rangefinder | 0.05 m | 0.02 m |
| compass | 1.2 deg + 1.5 deg x load | 0.9 deg + 4 deg x motor |
| gravity aiding | 1.0 m/s2 | |
| gates (NIS) | GPS position 11.8 (2 DOF, 3-sigma), velocity 21.1 (3 DOF), height 25, baro 25, range 25, compass 16, gravity 11.3 (3 DOF) | |

### EKF vs BASIC

`cargo test --release --test suites t7 -- --nocapture`, REAL sensors, breeze, seeds 700..704 for both
estimators (means over all samples of all 5 runs, max in brackets).

| Scenario | EKF | BASIC |
|---|---|---|
| Hover 60 s: est pos error | 0.44 (0.96) m | 0.46 (1.34) m |
| Hover: altitude / tilt / heading error | 0.009 (0.04) m / 0.34 (1.22) deg / 0.15 (0.66) deg | 0.016 (0.11) m / 0.56 (2.17) deg / 1.55 (2.24) deg |
| Hover: true wander from anchor | 0.45 (0.98) m | 0.48 (1.34) m |
| Circuit 3 laps: est pos error | 0.40 (0.79) m | 0.63 (1.44) m |
| Circuit: altitude / tilt / heading error | 0.10 (0.38) m / 0.38 (1.25) deg / 0.19 (0.71) deg | 0.27 (0.65) m / 3.79 (6.89) deg / 1.45 (2.94) deg |
| Circuit: tilt error in corners | 0.40 (1.25) deg | 4.30 (6.89) deg |
| Circuit: true speed / altitude | 4.25-5.21 m/s / 4.57-5.27 m | 3.54-5.06 m/s / 4.28-5.23 m |
| GPS glitch 3.5 m / 2.5 s: true heli moved | 0.65 (1.16) m, back to 0.50 m | 2.70 (3.04) m, back to 0.34 m |
| ... EKF rejections | 109 of 125 glitched samples; the 2 s reset fires once into the glitch and once back out (8 resets in 5 runs), each shifting the hold target with the estimate | |
| GPS glitch 6 m / 8 s (reset path) | moved 0.67 (1.20) m, back to 0.37 m; 2 resets per run, no crash | moved 6.13 (6.47) m, back to 0.37 m |
| LAND x5: touchdown / from pad centre | 0.28-0.36 m/s / 0.23-0.64 m | 0.23-0.39 m/s / 0.04-0.70 m |
| AUTOROTATE x5: touchdown / from pad | 0.57-0.63 m/s / 0.94-2.15 m, 0 rescues | 0.59-0.83 m/s / 1.37-3.24 m, 0 rescues |
| Crash protection, full roll at 8 m x5 | trigger +0.28-0.30 s, min alt 7.99-8.24 m, hold after 1.20-1.36 s | trigger +0.30-0.31 s, min alt 7.80-8.07 m, hold after 1.16-1.26 s |

10 minutes (60 s hover then circuit laps, 5 seeds, EKF): finite throughout, covariance diagonal
positive, below 1e4 and symmetric to 1e-9 (checked every second). |error| < 3 sigma: x 99.4%,
z 99.4%, height 100%, tilt 99.9%, heading 99.2%; horizontal NEES mean 1.84 (2 expected for 2 DOF).
1-sigma ranges: position 0.46-0.83 m, height 0.02-0.47 m (rangefinder near the ground, baro + GPS
at 5 m), tilt 0.34-0.61 deg, heading 0.19-0.21 deg. The largest position error, 4.3 m, is a natural
2..4 m GPS glitch (every 2..3 min) on its way through the reset.

The 3 m glitch reset is a judgement call: a real EKF cannot tell a 2 s glitch from a true position
jump, so after 2 s it believes GPS. The autopilot target moves with the reset, so the heli barely
moves, but the estimate carries the glitch offset until the glitch ends and the second reset takes
it back.

## Autopilot

| Mode | Behaviour |
|---|---|
| TAKEOFF / hold | spools up (soft start), climbs to 3 m, switches to hover hold 0.3 m below target |
| CIRCUIT | climbs to 5 m, joins a rounded-rectangle racetrack (x -10..14, z -20..-4, 5 m corners, ~70 m) at 5 m/s with path curvature and turn rate fed forward; lap toasts; runs until LAND or a stick touch |
| LAND | lines up over the nearest pad to 0.3 m and 0.35 m/s, then descends at `0.45 * alt` m/s clamped to 0.35..1.1 |
| AUTOROTATE | climbs to 25 m at an entry point 38 m upwind of the pad, runs in at 7 m/s, cuts the motor (throttle hold), glides at 6..8 m/s with collective governing rpm to 99%, flares at 6 m, cushions at a steady 5.5 m/s2 deceleration, sets down near the pad |
| Rescue (PROTECT) | takes over on more than 55 degrees of bank (or inverted on a normal pitch curve), a sink that would hit within 1.4 s below 8 m, settling with power, or headspeed under 72% (65% / 45 degrees in an autorotation glide); rolls upright, stops the sink at full power, climbs to a hover and hands back control; hands-off for 1.5 s |
| Battery failsafe | at 15% (2 s debounce) lands on the nearest pad, a stick touch takes over; at 8% forced landing that ignores sticks and keys |

| STUNTS (J) | picker of five manoeuvres, below |

All of it reads only the estimated state, from whichever estimator is selected.

### Stunts (`stunt`)

★ STUNTS (key J) opens a picker that TS renders from the list in the frame. Pick with a click, keys 1..5,
or arrows and Enter (Esc closes). Each stunt goes through four phases, and the cue line names each one:
*climbing* to its safe height and settling (0.5 s), *building speed* (loop and roll only: 8 m/s eastward or
westward along z = -12 m, starting 22 m out, well inside the tree ring), *in manoeuvre* (3D pitch on, user setting
remembered), *recovering* to hover hold. Then the pitch curve is restored and a result toast and panel appear. Scored items are
height lost (entry minus min altitude), max deviation (laterally from the line for loop and roll, in xz for the others),
heading error at exit, recovery time, and a 0..100 score. A stick touch hands back control. If the heli is inverted at that
moment, 3D pitch stays on.

| Stunt | Safe height | Profile |
|---|---|---|
| Pirouette | 15 m | 360 deg yaw at 180 deg/s (0.35 s sine ramps), thrust-vector position / altitude hold |
| Loop | 18 m, 8 m/s | 360 deg nose-up at 2.0 rad/s; collective holds `az = 12 cos(theta)` plus feedback to the scheduled height bump: positive at the bottom, negative over the top |
| Roll | 18 m, 8 m/s | 360 deg roll at 4.0 rad/s; collective holds altitude (vertical thrust saturated at 2.3 g, faded when the disc is near vertical), negative through inverted |
| Flip | 18 m | stationary 360 deg forward pitch flip at 4.0 rad/s |
| Inverted hover | 20 m | half flip (3.5 rad/s), 5 s inverted hover (thrust-vector controller with negated disc normal, negative collective, position and heading held), half flip back |

Rotations follow `q_ref = q_base * rot(axis, theta(t))`, with a sine-ramped rate profile. The rate demand is
the profile rate plus 7 x the attitude error, limited to 5 rad/s. A segment ends when its reference is done
and the error is under 20 deg.

Guards: no stunt below 25% battery (toast), none during a forced landing or a rescue. The battery failsafe never starts
mid-manoeuvre. Its timers keep running and it fires as soon as the stunt has recovered.

### Stunt-aware crash protection

From manoeuvre start until upright again (tilt < 30 deg) in the recovery, the bank > 55 deg and inverted
triggers are off. Rescue fires only on altitude < 6 m, headspeed < 75%, or the manoeuvre overrunning its expected
duration by more than 1.5 s. Everything else (manual flight, 3D inverted after a takeover) keeps the existing
rules. A rescue from beyond 60 deg of tilt rolls upright the shortest way: it picks the body pitch or roll axis,
whichever is closer to upright, latches the axis and direction, and commands rates directly. Then it stops the sink
and climbs as before.

### Sensor effects during stunts (REAL)

| | |
|---|---|
| Rangefinder | no reading above 25 deg of tilt or inverted (unchanged, verified in `t8_stunt_sensors`) |
| GPS sky view | antenna on the canopy: one satellite lost per 15 deg of tilt beyond 30 deg, no fix beyond 110 deg ("GPS LOST (inverted)"); on reacquire 2..4 s with 5..8 satellites and 3x position / velocity noise ("GPS REACQUIRING n sats"), reported to the EKF as 3x measurement sigma |
| Vibration | `rpm^2 (1 + 0.35 (|thrust_g| - 1)+) + 0.15 (disc tilt rate - 0.5)+ rpm`, max 2.5x; drives gyro / accel vibration and the EKF's IMU process noise; "VIB n.nx" in the panel above 1.25x |
| EKF | gravity aiding already gated (`||f| - g| < 0.25` and rates < 0.35 rad/s): no fusion at > 1 rad/s in the stunt tests; coasts on the IMU while GPS is lost (status GPS LOST, sigma grows) and reconverges through the normal gates |

The noise draw order of the GPS model is unchanged, so runs without stunts keep their exact random realisation.

### Stunt results

`cargo test --release --test suites t8 -- --nocapture` (1 run per stunt and mode, breeze):

| Stunt | IDEAL: min alt / lost / score | REAL EKF: min alt / lost / score | REAL EKF max est error att / pos |
|---|---|---|---|
| Pirouette | 15.1 m / 0.0 m / 100 | 15.2 m / 0.0 m / 100 | 0.7 deg / 0.44 m |
| Loop | 16.8 m / 1.2 m / 92 | 16.5 m / 1.2 m / 92 | 1.3 deg / 0.23 m |
| Roll | 17.3 m / 0.7 m / 97 | 17.2 m / 0.7 m / 94 | 1.8 deg / 1.06 m |
| Flip | 17.3 m / 0.8 m / 86 | 17.2 m / 0.8 m / 87 | 1.2 deg / 0.58 m |
| Inverted hover | 19.6 m / 0.4 m / 94 | 19.7 m / 0.4 m / 66 | 1.4 deg / 3.66 m (6 s GPS outage; back under 1 m 0.4 s after reacquire) |

None of these runs crashed or triggered a rescue. Broken stunts: a loop at 7 m with the collective cut over the top
triggers "Too low" 2.6 s in and recovers (min 2.3..2.5 m), and a collective cut mid-roll triggers "Sinking" 2.3 s in and
recovers (min 6.4..6.7 m), in both IDEAL and REAL. With PROTECT off, both crash. Shortest-way rescue: rolled 150 deg ->
recovers about roll (-155 deg), pitched 150 / -160 deg -> about pitch.

## Tests: Rust vs JS

`cargo test --release -- --nocapture --test-threads=1` in `rust/` (30 suites: the 26 below plus 4 `t8_stunt*`; plus `tests/step.rs`, about 3.5 s; the 7 `t7_ekf_*`
suites are the EKF vs BASIC table above) against the JS harnesses `node test*.js rc-heli-field.html` on the original page. The JS uses Math.random, the Rust
a seeded RNG, so noisy numbers agree within noise rather than to the digit. The REAL rows now run on
the EKF (the default); the BASIC column of the table above is the original estimator.

| Scenario | JS | Rust |
|---|---|---|
| Takeoff -> hold | 3.0 m, vs -0.0 | 3.00 m, vs -0.00 |
| Circuit, 6 laps | 14.9, 14.2, 14.2, 14.2, 14.2, 14.2 s; 4.44-5.02 m/s; alt 5.00-5.00 m; bat 73%; 6 lap toasts | 14.9, 14.1, 14.1, 14.1, 14.2, 14.2 s; 4.44-5.01 m/s; alt 5.00-5.00 m; bat 73%; 6 lap toasts |
| Land after circuit | Butter-soft on Pad A, 0.35 m/s | Butter-soft on Pad A, 0.35 m/s |
| 15% failsafe in circuit | after 200 s, 13 laps, true 14.5%; lands 0.35 m/s | after 200 s, 13 laps, true 14.5%; lands 0.35 m/s |
| Ground, breeze: disarmed / idle / 30% | x 0 z 0 each, hdg 0.0 / 0.0 / 0.7 | x 0 z 0 each, hdg 0.0 / 0.0 / 0.7 |
| Ground, breeze: 40% (light on skids) | x -0.905 z 0.043 | x -0.902 z 0.043 |
| Ground, gusty: disarmed / idle / 30% | (not in test2) | x 0 z 0 each |
| Sag, full pack: hover / full collective | 1.00 / 0.96 | 1.00 / 0.96 |
| Sag, 15%: hover / full collective | 1.00 / 0.90 | 1.00 / 0.90 |
| Steady autorotation coll 0.08 / 0 / 0.12 / 0.18 | 7.66 / 8.74 / 7.17 / 6.58 m/s, rpm 1.00 / 1.05 / 0.97 / 0.92 | 7.64 / 8.73 / 7.16 / 6.56 m/s, rpm 1.00 / 1.05 / 0.97 / 0.92 |
| Scripted flare at 3 m (x2) | landed 1.87, 1.84 m/s | landed 1.89, 1.83 m/s |
| Late flare at 2 m | crash 3.72 m/s | crash 3.71 m/s |
| No flare | crash 7.63 m/s | crash 7.61 m/s |
| Hover collective held after cut | crash 5.33 m/s, rpm 0.64 | crash 5.31 m/s, rpm 0.64 |
| Cut yaw kick / bailout | -3.7 deg; rpm 1.00, 17.4 -> 15.5 m, softStart false | -3.7 deg; rpm 1.00, 17.4 -> 15.7 m, softStart false |
| Protect (1) full roll at 8 m | trigger +0.29 s Over-banked 55, min 7.95 m, hold +1.64 s | trigger +0.29 s Over-banked 56, min 7.95 m, hold +1.64 s |
| Protect (2) THR HOLD at 20 m | trigger +3.41 s Headspeed 72%, min 11.02 m, hold +5.51 s | trigger +3.42 s Headspeed 72%, min 11.03 m, hold +5.52 s |
| Protect (3) collective 0 at 4 m | +0.30 s Sinking 2.6 m/s, min 3.29 m, hold +1.81 s | +0.30 s Sinking 2.6 m/s, min 3.29 m, hold +1.81 s |
| Protect (4a/4b) autopilot / manual land | no trigger, 0.35 / 0.08 m/s | no trigger, 0.35 / 0.00 m/s |
| Protect (4c) manual auto with flare | trigger 7.97 m, min 6.10 m, ends in hold | trigger 7.94 m, min 6.08 m, ends in hold |
| Protect (5) PROTECT off | no trigger, CRASHED | no trigger, CRASHED |
| Protect REAL (1) / (2) / (3) | +0.30 s, hold +1.49 / +3.18 s, min 11.98 m / +0.30 s | +0.29 s, hold +1.47 / +3.15 s, min 12.93 m / +0.29 s |
| Protect REAL (4a/4b/4c/5) | 0.37 / 0.14 m/s / trigger 8.03 m / CRASHED | 0.37 / 0.00 m/s / trigger 8.10 m / CRASHED |
| Battery IDEAL: 15% / stick touch | true 14.5% est 14.5% / off | true 14.5% est 14.5% / off |
| Battery IDEAL: 8% forced | true 7.5%, 20/20 inputs ignored, touchdown 1.04 m/s (1.04-1.97, median about 1.55, over 24 runs) | true 7.5%, 20/20 ignored, touchdown 1.73 m/s (0.93-2.50, median about 2.0, over 22 seeds) |
| Battery REAL: 15% / 8% forced | true 15.7% est 14.5% / true 8.2% est 7.5%, 20/20, 0.36 m/s | true 15.6% est 14.4% / true 8.2% est 7.4%, 20/20, 0.29 m/s |
| REAL hover 30 s | pos std 0.18 / 0.15 m; max 0.65 m from anchor; alt 2.91-3.08; est err pos 0.26 (max 0.63) m, alt -0.00, tilt 0.58 (1.45), hdg 1.42 (1.82) deg; rpm tlm 0.97-1.02 | pos std 0.15 / 0.20 m; max 0.89 m; alt 2.97-3.07; est err pos 0.30 (max 0.82) m, alt 0.00, tilt 0.33 (0.86), hdg 0.14 (0.45) deg; rpm tlm 0.97-1.02 |
| REAL circuit 3 laps | 14.3, 13.8, 14.1 s; pos err 0.50 (1.07) m; corner tilt err 4.45 (6.91) deg; 3.62-4.93 m/s; alt 4.55-5.08 | 14.7, 14.2, 14.3 s; pos err 0.86 (1.23) m; corner tilt err 0.34 (0.81) deg; 4.39-5.14 m/s; alt 4.75-5.20 |
| REAL land x5 | 0.29-0.41 m/s, 0.31-0.69 m from centre | 0.30-0.37 m/s, 0.24-0.70 m from centre |
| REAL GPS glitch 3.5 m / 2.5 s | moved 2.91 m, back to 0.24 m, hold | moved 0.58 m, back to 0.29 m, hold |
| REAL battery telemetry at 20% | est 18.2%, dips to 14.7%, recovers 15.7%, no failsafe, 41.4 V; hover failsafe true 15.7% est 14.5% | est 18.3%, dips to 14.6%, recovers 15.8%, no failsafe, 41.4 V; hover failsafe true 15.6% est 14.6% |
| AUTOROTATE IDEAL x5 | cut 25.0 m; sink 7.07-7.08, fwd 5.34-5.35 m/s; rpm 0.61-1.00; touchdown 0.44-0.45 m/s; 0.92-1.09 m from pad; 0 rescues, 0 crashes | cut 25.0 m; sink 7.08-7.10, fwd 5.35-5.39 m/s; rpm 0.61-1.00; touchdown 0.44-0.46 m/s; 1.02-1.11 m from pad; 0 rescues, 0 crashes |
| AUTOROTATE REAL x5 | cut 24.9-25.0 m; sink 6.89-6.98 m/s; rpm 0.72-1.01; touchdown 0.61-0.80 m/s; 1.82-3.06 m from pad; 0 rescues, 0 crashes | cut 25.0 m; sink 6.84-7.03 m/s; rpm 0.71-1.01; touchdown 0.55-0.62 m/s; 1.15-2.05 m from pad; 0 rescues, 0 crashes |

Notes on differences, all within run-to-run noise of the JS version:

- The IDEAL 8% forced landing is noisy in both versions. In this scripted test the pack goes completely
  flat about 31 s after the 8% trigger, partway down the descent, and the heli drops the rest of the way
  on a coasting rotor. The touchdown speed depends on the height where that happens. In Rust the trigger
  catches the heli about 0.2 m higher on average, so the median touchdown is about 2.0 m/s, against
  about 1.55 m/s in JS. Both versions stay under the 3 m/s crash limit in every run. The test asserts
  no crash rather than a soft landing.
- REAL protect (2): when the headspeed trigger fires depends on sensor noise during the throttle-hold
  sink, so the minimum altitude moves between runs.
- REAL-mode estimate errors depend on the seed (GPS bias and gyro bias are drawn at reset).

Deliberate differences from the JS page:

- Physics wind uses sim time. The JS used `performance.now()`, which equals sim time in the harnesses but
  not in the browser after dropped frames. Runs are reproducible from the seed; the page picks a random
  seed at load.
- The help overlay CSS gains `.overlay[hidden]{display:none}`. Without it the author `display:grid`
  overrides the `hidden` attribute, so "Go to the flight line" never hid the card in a plain browser.

Also tested: identical results for the same seed and different estimates for different seeds, and the
wasm frame JSON (mode code, toasts drained after each frame).

### Page tests

- `node web/test/smoke.mjs rc-heli-field.html`: pulls the base64 wasm out of the built page, loads it
  through `engine.ts` exactly as the page does (atob, Blob stream, DecompressionStream gzip,
  WebAssembly.instantiate), flies takeoff -> 2 circuit laps -> land -> REAL sensors -> EKF panel and ESTIMATOR toggle through the exports,
  then runs the page's bundle in a vm with a DOM / THREE stub, pumps requestAnimationFrame and exercises
  a Pirouette through the exports, buttons, keys (K, M, E, C, J / Esc, G, R) and a stick takeover with no runtime errors.
- `PW_DIR=<dir with puppeteer-core and @sparticuz/chromium> node web/test/browser.mjs rc-heli-field.html`: headless Chromium (puppeteer-core with
  @sparticuz/chromium from npm, software WebGL). Answers the cdnjs three.js request with the identical
  three@0.128.0 build from npm because cdnjs is not reachable from the build machine. It clicks TAKEOFF,
  presses K, M, E (and the ESTIMATOR button) and G and checks hover hold, circuit, the sensor panel
  with the EKF status and test ratio bars, and a Butter-soft landing on Pad A. It opens ★ STUNTS, checks the five picker entries, flies a Pirouette from the picker to recovery
  (cue, result panel, pitch curve restored), then flies manually (Space, W) and checks a 390 px phone viewport with the picker open: no horizontal scroll,
  sticks and picker in view, 3-column readouts. There are no page errors.

The sim costs about 6.0 microseconds per 240 Hz step in wasm (Node) in REAL mode with both estimators
running (3.4 before the EKF), and building a frame with the sensor panel takes about 30 microseconds.

## Build and run

```
cd web && npm install && ./build.sh        # writes ../rc-heli-field.html
cd rust && cargo test --release            # native test suites
cd rust && cargo run --release -- [seed] [real]   # takeoff, 3 laps, land
```

Open `rc-heli-field.html` in a browser. It needs the network only for three.js (cdnjs) and the fonts.

The wasm32 target can't be downloaded here (static.rust-lang.org is blocked), so `build.sh` builds the
wasm32 std from source: `RUSTC_BOOTSTRAP=1 cargo build --profile wasm --lib --target wasm32-unknown-unknown -Zbuild-std=std,panic_abort`.
That needs rust-src for the exact toolchain (rustc 1.97.0, commit 2d8144b7). It was fetched from GitHub
with a sparse, blob-filtered git fetch of `library/` at that commit, plus the `library/backtrace`
submodule (rust-lang/backtrace-rs at 28ec93b5), and copied to
`$(rustc --print sysroot)/lib/rustlib/src/rust/library`. If the wasm32 target is installed,
`build.sh` uses a normal build. wasm-opt comes from the npm `binaryen` package and is skipped if
missing. The `wasm` profile is opt-level z, LTO, one codegen unit, panic abort, stripped. The crate
has no dependencies.

Sizes: wasm 129 KB raw, 81 KB gzip+base64 embedded; bundle 17 KB; page 114 KB (111 KiB).
