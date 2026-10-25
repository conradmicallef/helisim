# RC Heli Field

**▶ Fly it in your browser: https://conradmicallef.github.io/helisim/**

A 600-class flybarless RC helicopter on a flying field, seen from the pilot station. You fly it with two
on-screen gimbals (Mode 2), the keyboard or a USB transmitter / gamepad, or let the autopilot take off,
hold, fly racetrack laps, land or demonstrate an autorotation. Crash protection, a low-battery failsafe
and a REAL sensor mode (the flight controller flies on noisy, delayed sensors, including a downward
camera that finds the tag on the pad for precision landings) are modelled. There is one landing pad, in
front of the pilot station.

This is a port of the single-file JavaScript page (`rc-heli-field.html`, three.js r128) to the same
architecture as the circuit autopilot project: a Rust core compiled to WebAssembly owns all simulation,
control, autopilot, sensor and rule logic, and a TypeScript page only renders, gathers input and calls
the exports. The built page is one self-contained HTML file of about 139 KB.

## Layout

| Path | What |
|---|---|
| `rust/` | crate `helisim`: lib (rlib + cdylib) and a native `helisim` bin for quick runs |
| `rust/tests/suites.rs` | native ports of the JS harnesses test.js .. test5.js, with asserts |
| `rust/examples/` | `stunt_dev` (trace one stunt), `inv_diag` (per-seed EKF diagnostics through a stunt: bias, tilt, velocity errors, fusion counts), `land_far` (LAND from 30 / 45 m, 5 seeds: time, heights, accuracy; `real`, `nocam`), `land_trace` (one LAND traced every 0.5 s with the camera target error), `fs_far` (battery failsafe far from the pad) |
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
| `assist` | flying assists: throttle curve, ALT HOLD, AUTO-YAW, tail RATE, ground-proximity predictor |
| `stunt` | STUNTS: pirouette, loop, roll, flip, inverted hover; profiles, scoring, cue text |
| `sensors` | gyro, accelerometer, magnetometer, rangefinder, barometer, GPS, ESC rpm and battery telemetry, each on its own clock; runs the camera and the target estimator |
| `precland` | downward camera (pad tag detection from geometry) and the precision-landing target estimator |
| `estimator` | BASIC complementary filter; `Est` is what every controller sees |
| `ekf` | error-state extended Kalman filter (default in REAL mode), fixed-size 18-state matrix code, drag fusion with wind states, gating, GPS reset, health |
| `autopilot` | takeoff, hold, circuit path follower, land (wind-aware transit, camera precision landing), rescue, autorotation, battery failsafe |
| `sim` | world step, commands, landing grading, toasts as an event list, HUD, cue and sensor-panel text |
| `jsonw` | tiny JSON writer and JS-style `toFixed` (no serde_json, no core::fmt float code) |
| `wasm` | C-ABI exports, one `Sim` in a thread_local RefCell |

### TypeScript

| File | Role |
|---|---|
| `engine.ts` | base64 -> DecompressionStream('gzip') -> WebAssembly.instantiate; wraps the exports; `Frame` type; command codes |
| `input.ts` | touch gimbals, keys, gamepad axes (raw values only) |
| `view3d.ts` | three.js scene, the pad with its camera tag, heli model, ghost, windsock, pilot / chase / pad cameras, and the downward camera rendered into the picture-in-picture box (scissored viewport of the same renderer) |
| `hud.ts` | readouts, status line, estimator pill and sensor panel, assist pills, event toasts, action bar and settings states, PiP detection box, all from the frame |
| `main.ts` | wiring, the settings sheet (open / close / Esc), desktop tooltips, the frame loop |
| `three.ts` | the global `THREE` from the script tag, typed with @types/three 0.128 |

### wasm interface

Only plain numbers cross the boundary.

| Export | |
|---|---|
| `sim_init(seed)` | new sim with a seed |
| `sim_input(yaw, pit, rol, thrDelta, thrSet, gp, g0, g1, g2, g3)` | this frame's input; `thrSet` < 0 means the touch throttle did not move; `gp` = 1 with gamepad axes 0..3 |
| `sim_cmd(code)` | 0 arm, 1 hold, 2 protect, 3 sensors, 4 level, 5 pitch, 6 wind, 7 cam, 8 reset, 9 takeoff, 10 circuit, 11 land, 12 autorotate, 13 takeover (a stick was touched), 14 estimator EKF / BASIC, 15 stunt picker toggle, 16..20 fly pirouette / loop / roll / flip / inverted hover, 21 close the picker, 22 ALT HOLD, 23 AUTO-YAW, 24 tail HOLD / RATE, 25 camera view AUTO / ALWAYS (codes 0..24 unchanged) |
| `sim_step(n)` | n fixed steps of 1/240 s |
| `sim_frame()` | builds the JSON frame, returns its length; read it at `sim_out_ptr()` |

Per animation frame the page sends input, runs the elapsed fixed steps (at most 24, the remainder is
dropped as before), reads one frame and renders it. The frame carries true and estimated pose, rotor
rpm, wind, settings (including `ekf` and `sig2`, the 2-sigma horizontal uncertainty radius drawn as a ring around the ghost), the autopilot mode code, the gimbal positions to draw, the HUD strings with their
colour classes, the cue line, the sensor panel and the toasts raised since the last frame. All text is
decided in Rust; the page writes it to the DOM. New fields (all additions): `hud.rpmShow` (show the
headspeed readout), `hud.health` / `hud.healthC` (the collapsed estimator pill), `pip` / `pipSet` (show the
camera view, user setting), `camSt` / `camN` / `camLock` / `camTgt` (camera status, detections, tag locked,
LAND flying the camera target), `camDet` (last detection `[u, v, span px, yaw]` of a 640 x 480 image, empty
if none in the last 0.25 s) and `ev` on each toast (an event the page shows; see "Page layout").

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
(< 0.5 m/s), Soft (< 1), Firm (< 2) or Hard, "on the pad" within 1 m of its centre, and the best on-pad landing
is kept. The beginner goal (status line) is a manual flight that goes at least 8 m out and lands back on the
pad under 1 m/s.

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
| Downward camera | 20 Hz, 80 ms latency | 70 deg FOV, 640 x 480, under the fuselage; detects the pad tag from geometry, 0.7 px noise plus blur, 4% dropouts, rare false detections (see "Downward camera precision landing") |

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
EKF / PX4 ECL. No dependencies: 18x18 arrays, a sparse F P F' product and a 3x3 elimination for the
innovation tests. It adds about 3 microseconds per 240 Hz step in wasm (the two wind states and drag
fusion add about 1 microsecond to the 16-state filter; step timings on the build machine vary 8..13 microseconds run to run).

| | |
|---|---|
| Nominal state | position, velocity (world), attitude quaternion, gyro bias, accelerometer bias, baro offset, horizontal wind |
| Error state (18) | dp, dv, dtheta (world-frame rotation error), dbg, dba, dbaro, dwind (x, z) |
| Predict, 240 Hz | bias-corrected gyro and specific force rotated to world plus gravity; F = I + A dt with the [f]x and -R couplings; Q from the sensor models |
| GPS, 10 Hz | position and velocity measured 150 ms earlier: innovations are formed against a 0.5 s buffer of past nominal states at the measurement time; the correction applies to the current state and is also applied to the buffer |
| Barometer, 25 Hz | height + offset state; noise inflated while the disc is in ground effect (< 1.4 m) |
| Rangefinder, 20 Hz | slant range converted with the filter's own attitude, tilt < 25 deg, 0.05..4 m, flat terrain y = 0 |
| Magnetometer, 50 Hz | yaw-only fusion (no gain into roll / pitch), compass-motor compensation 3.6 deg per unit motor load (10% under the real 4 deg), residual noise inflated with load |
| Gravity aiding, 10 Hz | 0.1 s mean specific force against R' g, only when the magnitude is within 0.25 m/s2 of g and rates are under 0.35 rad/s; updates tilt and accel bias, not heading |
| Drag, 10 Hz | 0.1 s mean specific force across the shaft (body x, z) against the airframe drag `-k (v - wind)` rotated to the body, k = 0.20 + 0.032 \|v_air\| (10% under the true 0.22 + 0.035); airborne above 1 m, spooled, rates under 1 rad/s; sigma 0.35 m/s2, inflated with vibration; NIS gate 13.8 (2 DOF). With GPS it only updates velocity and the wind states. While GPS is lost it also corrects tilt and gyro / accel bias (never heading). This is ArduPilot EK3 / PX4 EKF2 drag fusion, see below |
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
| drag | 0.35 m/s2 per body axis, x (1 + vibration above hover) | 0.87 m/s2 vibration per sample, averaged over 0.1 s, plus disc-tilt thrust and the 10% drag-model error |
| wind random walk | 0.1 m/s/sqrt(s), initial 3 m/s | slow BREEZE gust +-0.8 m/s over ~9 s; the 3 s gust term averages out |
| gates (NIS) | GPS position 11.8 (2 DOF, 3-sigma), velocity 21.1 (3 DOF), height 25, baro 25, range 25, compass 16, gravity 11.3 (3 DOF), drag 13.8 (2 DOF) | |

### EKF vs BASIC

`cargo test --release --test suites t7 -- --nocapture`, REAL sensors, breeze, seeds 700..704 for both
estimators (means over all samples of all 5 runs, max in brackets).

| Scenario | EKF | BASIC |
|---|---|---|
| Hover 60 s: est pos error | 0.47 (0.97) m | 0.49 (1.17) m |
| Hover: altitude / tilt / heading error | 0.009 (0.04) m / 0.34 (1.21) deg / 0.15 (0.65) deg | 0.015 (0.08) m / 0.58 (1.74) deg / 1.51 (2.24) deg |
| Hover: true wander from anchor | 0.48 (1.02) m | 0.51 (1.15) m |
| Circuit 3 laps: est pos error | 0.43 (0.78) m | 0.65 (1.54) m |
| Circuit: altitude / tilt / heading error | 0.09 (0.38) m / 0.39 (1.39) deg / 0.19 (0.72) deg | 0.28 (0.68) m / 3.91 (7.54) deg / 1.46 (3.06) deg |
| Circuit: tilt error in corners | 0.40 (1.39) deg | 4.43 (7.54) deg |
| Circuit: true speed / altitude | 4.38-5.23 m/s / 4.57-5.29 m | 3.50-5.22 m/s / 4.30-5.08 m |
| GPS glitch 3.5 m / 2.5 s: true heli moved | 0.48 (0.71) m, back to 0.29 m | 2.45 (2.78) m, back to 0.33 m |
| ... EKF rejections | 109 of 125 glitched samples; the 2 s reset fires once into the glitch and once back out (10 resets in 5 runs), each shifting the hold target with the estimate | |
| GPS glitch 6 m / 8 s (reset path) | moved 0.58 (0.81) m, back to 0.30 m; 2 resets per run, no crash | moved 6.07 (6.28) m, back to 0.47 m |
| LAND x5: touchdown / from pad centre | 0.29-0.41 m/s / 0.07-0.26 m (camera; GPS only it was 0.22-0.64 m) | 0.23-0.40 m/s / 0.05-0.37 m (camera; was 0.05-0.68 m) |
| AUTOROTATE x5: touchdown / from pad | 0.55-0.60 m/s / 0.99-1.52 m, 0 rescues (was 0.59-0.65 / 0.95-2.24) | 0.61-0.74 m/s / 2.11-3.00 m, 0 rescues (was 0.66-0.84 / 1.48-2.38) |
| Crash protection, full roll at 8 m x5 | trigger +0.29 s, min alt 7.70-8.11 m, hold after 1.46-1.62 s | trigger +0.30 s, min alt 7.42-7.83 m, hold after 1.41-1.57 s |

10 minutes (60 s hover then circuit laps, 5 seeds, EKF): finite throughout, covariance diagonal
positive, below 1e4 and symmetric to 1e-9 (checked every second). |error| < 3 sigma: x 99.7%,
z 99.6%, height 100%, tilt 99.9%, heading 99.0%; horizontal NEES mean 1.88 (2 expected for 2 DOF).
1-sigma ranges: position 0.46-0.83 m, height 0.02-0.47 m (rangefinder near the ground, baro + GPS
at 5 m), tilt 0.35-0.63 deg, heading 0.19-0.21 deg. The largest position error, 4.4 m, is a natural
2..4 m GPS glitch (every 2..3 min) on its way through the reset.

The 3 m glitch reset is a judgement call: a real EKF cannot tell a 2 s glitch from a true position
jump, so after 2 s it believes GPS. The autopilot target moves with the reset, so the heli barely
moves, but the estimate carries the glitch offset until the glitch ends and the second reset takes
it back.

### Dead reckoning without GPS: the inverted-hover drift (root cause and fix)

The REAL inverted hover used to drift 1.3..4.6 m (mean 2.5 m over 20 seeds, score 58..94, mean 78;
IDEAL 1.0 m / 94). GPS is gone for 6..9 s (antenna upside down). Diagnosed with
`cargo run --release --example inv_diag -- <seed0> <n> 4 [trace]`, which logs per seed the true and
estimated gyro bias, accel bias, the world-frame tilt error about x and z, velocity and position
error, gravity-aiding and drag fusion counts, baro use and sigmas through the stunt. During the
investigation the same tool fed the EKF and the controllers oracle (true) values one at a time.
That code was removed afterwards.

What it was **not**:

- Gyro bias. The bias error stayed at 0.02..0.15 deg/s through the flips. With the estimated bias
  forced to the true bias for the whole outage, the mean drift went from 2.46 to 2.38 m over 40 seeds,
  no change. Freezing bias learning above 0.5 or 1 rad/s did nothing either (1.48 / 1.50 m against
  1.50 m with the fix in place). Neither change was kept.
- Gravity aiding. In the outage it passes only 5..21 of about 60 windows (the 0.35 rad/s rate gate
  and the |f| = g gate). Turning it off while GPS is lost changed nothing (2.45 m).
- Accel bias, baro, rangefinder (off above 25 deg), the GPS latency buffer and reacquire handling.
  Re-convergence was already 0.0..0.5 s. Giving the controller true position and velocity only after
  reacquire changed nothing (2.48 m). Giving them only during the outage brought it to IDEAL (1.20 m).
  The error is made during the outage, and the extra metres after reacquire are just the heli
  flying back.
- The stunt controller's gains. Scaling the position or velocity gain down with EKF velocity sigma
  made no difference (1.50..1.54 m).

What it **was**: plain inertial dead reckoning with nothing to bound the velocity. Without GPS the
filter has no horizontal velocity reference at all. The 40 / 20 Hz low-passed, vibrating IMU
integrates a tilt error of 0.3..2 deg (random walk of the vibration-driven gyro noise plus the
0.5 deg tilt the filter already carried into the flip). Times 1 g, that is 0.05..0.35 m/s2, which
becomes 0.4..1.4 m/s of velocity error in 6..9 s. The hold controller then flies the heli there. A
per-term velocity budget over the outage matched the logged error to within 0.1 m/s: tilt x g was
the dominant term in most seeds, IMU noise integrated directly came second, and accel bias was
small. A noise-free IMU with the true bias and the true entry tilt still left 1.7 m, so no
bias-learning or noise-tuning change could reach IDEAL.

The fix is the one real flight controllers use for this (ArduPilot EK3 `EK3_DRAG_*`, PX4 EKF2 drag
fusion). On a helicopter the accelerometer across the shaft measures only airframe drag, because
thrust and disc damping act along the shaft and gravity is not sensed. So
`f_body,xz = R' (-k (v - wind))` is a velocity measurement relative to the air. The filter gains
two wind states (x, z) and fuses the 0.1 s mean of that specific force at 10 Hz (table above). With
GPS, drag fusion only updates velocity and wind. The drag model ignores disc tilt and flapback, and
letting it tilt the attitude cost 1.5 points of the 10-minute 3-sigma tilt consistency. Once GPS is
lost it also corrects tilt and gyro / accel bias, because then it is the only horizontal reference.
The model k is calibrated 10% low, like a field-calibrated coefficient. The wind random walk
(0.1 m/s/sqrt s) is the main tuning knob. At 0.25, the wind state soaks up the velocity drift it is
meant to bound (mean 1.70 m against 1.50 m). No truth or position-hold pseudo-measurement is used.

Inverted hover, REAL EKF, per seed (score / max deviation m / max est velocity error m/s / max est position error m):

| seed | before | after |
|---|---|---|
| 800 | 68 / 3.34 / 0.91 / 2.58 | 84 / 1.98 / 0.46 / 1.19 |
| 801 | 83 / 2.02 / 0.67 / 2.27 | 93 / 1.06 / 0.45 / 1.66 |
| 802 | 78 / 2.64 / 0.79 / 2.60 | 89 / 1.54 / 0.66 / 1.92 |
| 803 | 88 / 1.65 / 0.54 / 1.78 | 93 / 1.15 / 0.52 / 1.54 |
| 804 | 66 / 3.36 / 1.35 / 3.66 | 91 / 1.16 / 0.29 / 1.15 |
| 805 | 93 / 1.16 / 0.42 / 1.68 | 94 / 1.05 / 0.41 / 1.15 |
| 806 | 62 / 3.94 / 1.18 / 2.49 | 86 / 1.79 / 0.65 / 0.78 |
| 807 | 53 / 4.62 / 1.36 / 4.01 | 83 / 2.08 / 0.59 / 1.40 |
| 810 | 63 / 3.69 / 1.04 / 3.48 | 86 / 1.71 / 0.57 / 1.55 |
| 811 | 77 / 2.46 / 0.99 / 2.38 | 94 / 0.98 / 0.34 / 1.28 |
| 812 | 91 / 1.28 / 0.50 / 1.41 | 93 / 1.03 / 0.46 / 0.83 |
| 813 | 76 / 2.75 / 0.51 / 1.65 | 77 / 2.60 / 0.56 / 1.79 |
| 814 | 85 / 1.66 / 0.41 / 1.43 | 80 / 2.19 / 0.45 / 1.61 |

Over 120 seeds (800.., 900.., 1000.., 40 each): mean deviation 1.40..1.50 m, max 2.4..2.6 m, mean
score 89..90, 31..35 of 40 at 85 or more (before: 2.5 m, score 78, 14 of 40). `t8_inverted_dead_reckoning`
(seeds 820..839) measures 1.45 m mean, 2.30 m max, score 89 mean and 79 min, against 2.45 / 4.27 / 78 / 58
before. The target of 1.5 m is met on average and in about 60% of seeds. The rest are the
irreducible part. Even with a perfect IMU and the true entry state, the oracle runs left 1.5..1.7 m
mean: six seconds of dead reckoning on a vibrating helicopter is not a hover with GPS. Nothing else
regressed: every other table in this README was rerun with the new filter.

## Autopilot

| Mode | Behaviour |
|---|---|
| TAKEOFF / hold | spools up (soft start), climbs to 3 m, switches to hover hold 0.3 m below target |
| CIRCUIT | climbs to 5 m, joins a rounded-rectangle racetrack (x -10..14, z -20..-4, 5 m corners, ~70 m) at 5 m/s with path curvature and turn rate fed forward; lap toasts; runs until LAND or a stick touch |
| LAND | flies back to the pad: beyond 5 m on a wind-aware speed profile (up to 8 m/s) at 8..12 m (climbing first if low), nose along the track, then the gentle approach; lines up over the pad (or the camera's tag target) to 0.3 m (0.12 m with the camera), 0.35 m/s and the landing heading, then descends at `0.45 * alt` m/s clamped to 0.35..1.1. See "LAND transit" |
| AUTOROTATE | climbs to 25 m at an entry point 38 m upwind of the pad (the climb-out uses the LAND transit when it is more than 5 m away), runs in at 7 m/s, cuts the motor (throttle hold), glides at 6..8 m/s with collective governing rpm to 99%, flares at 6 m, cushions at a steady 5.5 m/s2 deceleration, sets down near the pad |
| Rescue (PROTECT) | takes over on more than 55 degrees of bank (or inverted on a normal pitch curve), a sink that would hit within 1.4 s below 8 m, ground proximity (below), settling with power, or headspeed under 72% (65% / 45 degrees in an autorotation glide); rolls upright, stops the sink at full power, climbs to a hover and hands back control; hands-off for 1.5 s |
| Battery failsafe | at 15% (2 s debounce) lands on the pad (the same LAND, transit included), a stick touch takes over; at 8% forced landing that ignores sticks and keys (no climb above the height it is at) |

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
| EKF | gravity aiding already gated (`||f| - g| < 0.25` and rates < 0.35 rad/s): no fusion at > 1 rad/s in the stunt tests; dead-reckons while GPS is lost (status GPS LOST, sigma grows), with drag fusion bounding the velocity (see "Dead reckoning without GPS"), and reconverges through the normal gates |

The noise draw order of the GPS model is unchanged, so runs without stunts keep their exact random realisation.

### Stunt results

`cargo test --release --test suites t8_stunt_matrix -- --nocapture` (5 seeds per stunt and mode,
seeds 810..814, breeze; ranges over the 5 runs, mean in brackets). Every run completed with no crash
and no rescue. Deviation is lateral from the line for loop and roll, horizontal for the others.
Heading is the error at exit. The asserts in `matrix_bounds` are these ranges with margin.

IDEAL:

| Stunt | done / crash / rescue | min alt m | height lost m | max dev m | heading deg | recovery s | score |
|---|---|---|---|---|---|---|---|
| Pirouette | 5 / 0 / 0 | 15.1 | 0.0 | 0.2 (0.22) | 2.3-2.4 | 0.5 | 100 |
| Loop | 5 / 0 / 0 | 16.7 | 1.3 | 0.0-0.2 (0.12) | 0.5-0.6 | 1.5-1.6 | 92 |
| Roll | 5 / 0 / 0 | 17.3 | 0.7 | 0.9-1.0 (1.00) | 0.0-0.3 | 1.2 | 96-97 (96) |
| Flip | 5 / 0 / 0 | 17.3 | 0.8 | 1.4-1.5 (1.47) | 0.2-0.4 | 0.9-1.5 | 86-87 (86) |
| Inverted hover | 5 / 0 / 0 | 19.6 | 0.4-0.5 | 1.0 (1.03) | 0.2-0.3 | 0.6-0.8 | 93-94 (94) |

REAL, EKF (default). The last two columns are the largest estimate-vs-truth tilt and horizontal position error
during the manoeuvre, and how long after the GPS fix returned the estimate was back under 1 m:

| Stunt | done / crash / rescue | min alt m | height lost m | max dev m | heading deg | recovery s | score | max est err att deg / pos m | re-converged after s |
|---|---|---|---|---|---|---|---|---|---|
| Pirouette | 5 / 0 / 0 | 15.0-15.2 | 0.0-0.1 | 0.2-0.5 (0.32) | 1.2-1.6 | 0.5 | 100 | 0.4-0.9 / 0.34-0.61 | no GPS loss |
| Loop | 5 / 0 / 0 | 16.5-16.8 | 1.0-1.3 | 0.2-1.2 (0.49) | 0.4-0.8 | 1.7-2.0 | 92-94 (93) | 0.9-1.7 / 0.37-1.02 | 0.0 |
| Roll | 5 / 0 / 0 | 17.0-17.2 | 0.7-0.8 | 1.1-1.3 (1.15) | 0.0-0.2 | 1.1-1.3 | 93-96 (95) | 0.9-2.0 / 0.24-0.67 | 0.0 |
| Flip | 5 / 0 / 0 | 17.3-17.4 | 0.6-0.8 | 1.3-1.6 (1.39) | 0.3-0.9 | 0.9-1.7 | 86-88 (88) | 0.6-1.8 / 0.38-0.75 | 0.0 |
| Inverted hover | 5 / 0 / 0 | 19.6-19.7 | 0.4-0.5 | 1.0-2.6 (1.70) | 0.2-0.7 | 0.5-2.4 | 77-94 (86) | 1.0-1.9 / 0.83-1.79 | 0.0-0.3 |

Before the drag-fusion fix, on the same seeds, the inverted hover was dev 1.3-3.7 m (2.37), score 63-91 (78),
max est pos error 1.41-3.48 m, re-converged after 0.2-0.5 s. The loop, roll and flip lose GPS only for a moment
and are unchanged within noise (loop 91-94, roll 93-97, flip 86-88). `t8_inverted_dead_reckoning` adds 20 more
inverted-hover seeds (above).

REAL, BASIC (the complementary filter), `t8_stunt_matrix_basic`:

| Stunt | done / crash / rescue | min alt m | height lost m | max dev m | heading deg | recovery s | score | max est err att deg / pos m | re-converged after s |
|---|---|---|---|---|---|---|---|---|---|
| Loop | 5 / 0 / 0 | 16.7-17.3 | 0.7-1.2 | 0.6-2.0 (1.25) | 0.6-5.3 | 1.7-5.7 | 81-97 (89) | 8.2-15.4 / 0.83-1.38 | 0.0-0.3 |
| Roll | 5 / 0 / 0 | 17.0-17.6 | 0.6-0.9 | 2.2-4.3 (3.03) | 0.0-0.6 | 1.1-3.2 | 62-85 (76) | 5.9-8.2 / 0.52-1.08 | 0.0 |
| Flip | 5 / 0 / 0 | 17.2-17.6 | 0.5-0.8 | 2.1-2.7 (2.30) | 0.0-0.5 | 0.9-2.0 | 75-82 (79) | 6.2-7.4 / 0.40-0.88 | 0.0 |

BASIC levels on the accelerometer minus the GPS-derived manoeuvre acceleration, so the flips and rolls pull its
attitude 6..15 deg off. The attitude loop still holds the manoeuvre, but the exit drifts 2..4 m. The EKF holds
attitude within 2 deg.

Broken stunts (`t8_stunt_protect`): a loop at 7 m with the collective cut over the top triggers "Too low" 2.6 s in
and recovers (min 2.5 m IDEAL, 2.3 m REAL). A collective cut mid-roll triggers 1.8 s in IDEAL ("Too low and fast",
min 10.2 m) and 2.3 s in REAL ("Sinking", min 6.2 m), and recovers. With PROTECT off, both crash. Shortest-way rescue: rolled 150 deg ->
recovers about roll (-155 deg), pitched 150 / -160 deg -> about pitch.

### LAND transit (`autopilot::land`)

LAND used to fly to the nearest pad at `0.9 * distance` m/s capped at 2.5 m/s, at the height where it
started (2..5 m). From the far side of the field that took 26 s at 5 m, low over the 8..12 m tree ring.
It now flies back fast and high, and keeps the gentle approach for the last 5 m.

| | |
|---|---|
| Speed | beyond the 5 m approach radius `v = min(cap, sqrt(2 a (d - 5) + 2.5^2))`, a braking curve that meets the old 2.5 m/s approach speed at 5 m, so there is no step. `a = 1.5 m/s2` (the braking deceleration is fed forward). Inside 5 m the old `0.9 d`, max 2.5 m/s |
| Wind | estimated wind along the track: a tailwind brakes earlier, `a x (1 - 0.08 x tailwind)`, at least `0.6 a`; into a headwind the ground speed cap is `min(8, 9 - headwind)` (airspeed at most 9 m/s) |
| Height | 8 m beyond 15 m from the pad, rising linearly to 12 m at 30 m (clear of the 5..12 m trees at r >= 45 m around (0, -20)); between 10 and 5 m it comes down linearly to the approach height (the start height clamped to 3..5 m); the target never drops below the approach height before 5 m. More than 1.5 m below the profile it climbs first (horizontal speed capped at 2 m/s, climb up to 2.5 m/s). LAND from closer than 15 m, and the 8% forced landing, never climb above where they start |
| Heading | nose along the track above 3 m/s (0.5 m/s hysteresis, the circuit's yaw law), then the landing heading for the final hover: the tag heading from the camera when it has locked, else the surveyed pad heading. The descent starts only within 10 deg of it |
| Descent | as before (within 0.3 m and 0.35 m/s, `0.45 alt` clamped to 0.35..1.1 m/s), now with a stiffer position hold over the pad (position gain 1.5 instead of 0.9, velocity gain 2.2 instead of 1.6, 1 m/s max), which halved the drift on the way down in gusty wind. If the target moves more than 0.6 m away above 1 m (camera lost or reacquired), the descent pauses at that height and lines up again |
| Ground proximity | the transit is level or climbing until 10 m out and slows below 2.5 m/s before the approach height, so the "Too low and fast" rule never sees a landing (0 rescues in every run below) |
| Reused | the autorotation climb-out to its entry point (beyond 5 m) and both battery failsafe landings fly the same transit |

`cargo test --release --test suites t10_land_transit -- --nocapture` (hover at 5 m at the start point,
breeze, seeds 1000..1004; before = the same runs on the previous build, `examples/land_far`):

| LAND from | before: time to touchdown | after: time | transit height | max ground speed | touchdown | from pad centre | rescues |
|---|---|---|---|---|---|---|---|
| 30 m, IDEAL | 20.4 s | **16.7 s** | 5.0 -> 10.8 m (before flat 5.0 m) | 6.2 m/s (before 2.5) | 0.35 m/s | 0.02 m | 0 |
| 45 m, IDEAL | 26.3 s | **19.1 s** | 5.0 -> 12.1 m | 6.9-7.0 m/s | 0.35 m/s | 0.02-0.03 m | 0 |
| 30 m, REAL EKF | 19.6-21.6 s | **17.9-19.2 s** | 4.9 -> 11.0 m | 5.9-6.4 m/s | 0.33-0.36 m/s | 0.07-0.21 m (before 0.24-0.89) | 0 |
| 45 m, REAL EKF | 25.8-27.7 s | **20.6-24.7 s** | 4.9 -> 12.3 m | 6.8-7.0 m/s | 0.32-0.42 m/s | 0.02-0.23 m (before 0.16-0.66) | 0 |
| 45 m, IDEAL gusty, tailwind (x3) | | 19.2 s | 5.0 -> 12.1 m | 7.7 m/s | 0.35 m/s | 0.07-0.08 m | 0 |
| 45 m, IDEAL gusty, headwind (x3) | | 20.5 s | 5.0 -> 12.1 m | 5.4 m/s | 0.35 m/s | 0.01-0.03 m | 0 |
| 3.6 m short hop, REAL (x3) | | 11.9-13.9 s | no climb | 2.2 m/s | 0.35-0.40 m/s | 0.06-0.16 m | 0 |

The REAL runs take 1..3 s longer than IDEAL: the descent waits for the camera lock and the tighter 0.12 m
line-up. The speed never reaches the 8 m/s cap from 45 m: the climb to 12 m comes first and the braking
curve starts at 33 m out. Battery failsafe from (-30, -25) at 6 m (`examples/fs_far`): 15% IDEAL 22.0 s,
REAL 24.8 s, 8% IDEAL 20.0 s, REAL 24.7 s; all butter-soft (0.33-0.35 m/s), 0.00-0.11 m from centre, 12.1-12.4 m
max height, 0 rescues. The scripted IDEAL 8% forced landing in `t4_battery_ideal` (heli 67 m out) now
reaches the pad before the pack is flat: 0.35 m/s on all 8 seeds 300..307, against 0.93..2.50 m/s on a
coasting rotor before (the README note further down describes the old behaviour).

### Downward camera precision landing (`precland`, REAL mode)

The pad carries an AprilTag-style marker: a 0.9 m black square with a white 6 x 6 payload inside a
one-cell border, on the light pad disc with the orange ring (the pad letter is gone with Pad B). The
camera does not decode the payload; detection is from geometry.

| Camera | |
|---|---|
| Mount | under the fuselage, 0.15 m below and 0.05 m ahead of the origin, looking down the body -y axis, image up = nose |
| Optics | 640 x 480, 70 deg horizontal FOV (focal length 457 px), 20 Hz, 80 ms latency, 6 ms exposure |
| Detected when | the tag centre is in the image, at least 85% of its area is in view (5 x 5 sample grid), it spans 12 px .. 90% of the image height (about 0.6 .. 35 m up, square root of the projected area), the true body tilt is under 35 deg, and the motion blur `(|w_xz| F + |w_y| r_px) x exposure` is under 6 px |
| Output | tag centre (px) with 0.7 px + 0.25 px per px of blur Gaussian noise, tag yaw relative to the image (1 deg noise), span |
| Failures | 4% of detectable frames dropped; 0.2% of frames above 0.5 m a false detection at a random spot (random size and yaw); test hooks: camera blocked (`sens.cam.enabled`), persistent fakes at a given ground offset and rate (`fake_hook`) |
| Panel | `CAMERA SEARCHING / TRACKING / LOST / OFF · n det` in the sensor panel; the collapsed pill shows `CAMERA TRACKING` when nothing more urgent (GPS lost / reset / rejected) is going on |

Fusion: a separate target estimator, as in ArduPilot's AC_PrecLand, rather than a state in the EKF. The
pad is fixed in the world, so the only thing the camera adds is where the pad is in the flight controller's
own (GPS-referenced, wandering) frame. A target estimate fused beside the EKF does exactly that, keeps the
18-state filter unchanged (its 10-minute consistency, glitch and stunt results all still hold) and lets the
autopilot fall back to GPS by simply not using it.

| Target estimator | |
|---|---|
| State | pad position (x, z) in the estimate's world frame, isotropic variance; pad heading from the tag yaw (low-passed, 20 deg gate) |
| Measurement | the pixel ray from the lens, rotated with the estimated attitude at the capture time (80 ms history of the estimate) and intersected with the ground at the estimated height (rangefinder-aided EKF height); sigma `slant x (pixel noise / F + EKF tilt sigma) + EKF height sigma x tan(off-axis) + 2 cm` |
| Gate | normalised innovation squared against 11.8 (2 DOF, 3 sigma), like the GPS position gate |
| Acquisition | 5 detections within 0.6 s that agree to 0.4 m, and agree with the prior (the surveyed pad in the GPS frame with the EKF position sigma) through the same gate: a lone false detection can't capture the target |
| Process noise | 0.3 m/sqrt(s), capped at the prior: the target follows the GPS-referenced estimate as its bias wanders |
| Reset | 8 rejected detections in 1.5 s with no fused one, consistent to 0.3 m: the target jumps to them (an EKF position reset also shifts it, with the autopilot targets) |
| Use | LAND, the autorotation flare / cushion target and the battery landings fly the camera target while it was fused within 1 s (4 s below 1.5 m, where the tag no longer fits the image), else the surveyed pad position in the GPS frame. A switch between the two slews the flown target at 0.8 m/s |
| Forget | no detection for 10 s: back to the GPS prior |

`cargo test --release --test suites t10_precland -- --nocapture` (REAL EKF, breeze unless noted; camera off =
`sens.cam.enabled = false`, the previous behaviour):

| Case | seeds | from pad centre | touchdown | rescues |
|---|---|---|---|---|
| LAND from 30 m, camera | 1010..1017 | **0.03-0.25 m, mean 0.13** | 0.29-0.41 m/s | 0 |
| LAND from 30 m, GPS only | 1010..1017 | 0.04-0.92 m, mean 0.36 | 0.31-0.43 m/s | 0 |
| LAND from 3.6 m, camera | 1010..1017 | **0.04-0.22 m, mean 0.15** | 0.33-0.40 m/s | 0 |
| LAND from 3.6 m, GPS only | 1010..1017 | 0.22-0.83 m, mean 0.53 | 0.31-0.43 m/s | 0 |
| Camera blocked on the way down at 3 m | 1020..1024 | 0.33-0.88 m, mean 0.54 (GPS) | 0.30-0.37 m/s | 0 |
| Camera blocked 8 s into the approach | 1020..1024 | 0.26-1.00 m, mean 0.61 (GPS) | 0.31-0.42 m/s | 0 |
| 25% of frames a false detection 1.2 m off the tag | 1030..1034 | 0.05-0.20 m, mean 0.13 | 0.32-0.38 m/s | 0 |
| Gusty (4.5 m/s), camera | 1040..1044 | 0.11-0.24 m, mean 0.16 | 0.24-0.45 m/s | 0 |
| Gusty, GPS only | 1040..1044 | 0.03-0.96 m, mean 0.41 | 0.29-0.48 m/s | 0 |

With the 25% persistent fakes, fewer than 1 in 10 fused detections is a fake (asserted). The rest are rejected
by the gate, because the locked target's sigma is a few centimetres. The target error relative to the true pad,
as seen from the heli, stays at 0.01-0.08 m through the descent in the traced runs. The remaining 0.1 m is the
position hold in gusts. Camera model checks (`t10_camera_model`, 39 frames each): detected at 5 m and 25 m
above the tag; not at 40 m (10 px), at 0.4 m (fills the image), 3.6 m to the side at 5 m (outside the field of
view), at 40 deg tilt, or at 3 rad/s of rate (blur). The old `t5_real_land_runs` / `t7_ekf_land_autorotate`
landings (from the 3 m hover) went from 0.22-0.64 m to 0.07-0.26 m from the centre (EKF) with the camera.

The picture-in-picture is a second three.js camera at the same mount, rendered into a scissored viewport
of the main renderer over a 120 x 90 px box (no extra render target). The detection box is drawn from
Rust's last detection (orange; green and "LOCK" when the target estimator is tracking, "LOCK · LANDING"
when LAND flies it). It shows in REAL mode during LAND, the autorotation flare and cushion, within 12 m
and 15 m height of the pad, or always when the CAMERA VIEW setting is ALWAYS.

## Flying assists (`assist`)

Manual flight on a phone lost height slowly (thrust tilted in translation and turns, the ETL band, direct
collective on a coarse touch stick) until the skids touched while moving or tilted: "Dynamic rollover"
or "Blade strike", with crash protection silent because the sink stayed under its 2.2 m/s trigger.
Six changes, all in Rust; the page only draws the buttons and the climb bar from the frame.

| | |
|---|---|
| Ground-proximity protection | new PROTECT rule (estimated state, so it runs on the estimate in REAL). Predicted contact `alt + vy t + ay t^2 / 2 = 0` within a 2 s look-ahead, `ay` = vertical acceleration low-passed 0.5 s, clamped to +-1 m/s2 and only counted while sinking. Fires when contact is predicted within 2.0 s AND (horizontal speed > 2.5 m/s OR tilt > 20 deg) AND sinking faster than 0.3 m/s. Never in a normal landing (horizontal < 1.5 m/s, tilt < 15 deg, sink < 1.2 m/s), on the ground, crashed, or in TAKEOFF / LAND / rescue, the autorotation and stunt windows (they keep their own rules). Rescue as before (level, stop, climb to at least 3 m). Toast "Too low and fast: 5.3 m/s at 1.3 m" or "Too low and tilted: 31 deg at 1.2 m" |
| Throttle curve | direct manual collective only: slope 0.5 within +-12% of stick around the hover point, linear outside, monotonic, ends unchanged. Hover point = `g / TMAX` = 0.455 of stick (normal) or 0.727 (3D). The autopilot's stick display goes through the inverse, so takeover is bumpless. (`Settings::thr_curve`, on; the JS-parity suites switch it off because they set raw stick positions) |
| ALT HOLD (Q, `bAlt`, default off) | in the air the left stick sets climb rate: +-8% deadband around hover = hold, full up +3 m/s, full down -2 m/s, descent limited to 1.0 m/s below 1.5 m (tapering to 0.5 m/s at the skids). Collective from the autopilot vertical loop (`coll_for`: tilt, ETL, inflow, ground effect and VRS feed-forward, gain 2.5), demand slewed at 4 m/s2, height loop 1.2 /s. Centring the stick brakes to zero climb, then latches the height. On the ground the stick is the normal throttle; engages once airborne above 0.5 m, upright (up.y > 0.5) and spooled, starting from the current collective (offset bled out over 1.5 s); on touchdown, throttle hold, inverted or OFF it hands back to direct with a 1/s collective slew. Any autopilot mode suspends it |
| Climb bar | `vsi` / `vsiC` / `vsiT` in the frame: estimated v/s, class `bad` below -2 m/s under 5 m (`warn` below -1 m/s under 4 m), and the ALT HOLD climb-rate target. TS draws a 10 px bar (8 px on phones) beside the left stick: centred zero, +-3 m/s, blue target mark |
| AUTO-YAW (Y, `bYaw`, default off) | manual flight only, upright: yaw-rate demand `2 * (heading of the air-relative velocity - heading) + coordinated g tan(bank) / V`, blended in by smoothstep from 2 to 3 m/s of both airspeed and ground speed (a hover in wind keeps heading hold), faded out beyond 120 deg of sideslip (deliberate backward flight), limited to 2.5 rad/s. The yaw stick adds on top and fades the heading term out at half stick. Off in stunts and all autopilot modes |
| Tail RATE (V, `bTail`, default HOLD) | manual flight only: no heading-hold integrator (it bleeds off at 8 /s), tail mixed from motor load instead (torque precompensation), so the tail weathervanes and drifts in gusts. Autopilot modes always fly HOLD |

Key T stays takeoff, so the tail gyro is on V. `cargo test --release --test suites t9 -- --nocapture`:

| Test | Result |
|---|---|
| Figure of eight on stick (two 7 m circles, 5.5 m/s, 22 deg max bank, self-level), ALT HOLD on, 60 s | IDEAL 4.99-5.00 m (x2 seeds); REAL EKF 4.72-5.28 and 4.95-5.49 m; no rescue, no crash |
| Same, ALT HOLD off, fixed hover throttle (the user's case), PROTECT on | IDEAL: lost 3.7 m in 8.7 s, "Too low and fast: 5.3 m/s at 1.3 m", recovered. REAL: lost 3.1 m, trigger at 1.9 m, recovered |
| Same, PROTECT off | IDEAL and REAL: crash, "Dynamic rollover: the skids caught while sliding sideways" |
| Gentle sink: 5 m/s forward, -0.8 m/s from 4 m, PROTECT on (2 seeds) | IDEAL trigger at 1.37 m (v/s -0.58), min 1.35 m; REAL trigger at 1.57-1.58 m, min 1.55 m; no crash |
| Gentle sink, PROTECT off | IDEAL and REAL: crash, dynamic rollover |
| No false triggers (IDEAL and REAL each) | autopilot LAND x3 (0.35 / 0.35 / 0.35, REAL 0.40 / 0.38 / 0.36 m/s), manual gentle landing x3 with 0 / 0.4 / 0.8 m/s drift, AUTOROTATE x2 (0.42 / 0.41, REAL 0.63 / 0.59 m/s), stunts x1 each (IDEAL: 100 / 92 / 96 / 87 / 94): 0 rescues |
| ALT HOLD handover: lift off on the stick, climb, hold 20 s, full up / down 2 s, descend to touchdown | engages at 0.52 m; hold at 3.01 m: band -0.00/+0.01 m (REAL 3.03 m: -0.05/+0.10, 3D 3.04 m: -0.00/+0.02); full up +2.94 m/s, full down -1.77 m/s (slewed); touchdown 0.58-0.62 m/s, back to direct; largest collective step at either handover 0.004 |
| AUTO-YAW, straight 5 m/s across a 4.5 m/s crosswind | heading vs air-relative velocity mean 2.1 deg, max 5.7 deg (off: 44 deg) |
| AUTO-YAW, coordinated turn on roll stick only | sideslip mean 0.3 deg, max 0.4 deg (off: 41 deg) |
| AUTO-YAW, hover in 4.5 m/s wind | heading held within 1.8 deg |
| Tail HOLD / RATE, hover in 4.5 m/s crosswind, 15 s | HOLD within 1.7 deg; RATE weathervanes 64 deg toward the wind; RATE set + autopilot hover: within 1.0 deg |
| Throttle curve | monotonic, f(0) = 0, f(1) = 1, f(hover) = hover, slope 0.50 at hover, 1.14-1.39 outside, normal and 3D |

## Page layout

The old page put 18 buttons in 9 rows, a cue and a toast saying the same thing, sticky tooltips, key-letter
hints and six readouts over about two thirds of a phone screen. The new layout keeps the design language
(the same colour tokens, Barlow Condensed and IBM Plex Mono) and puts the field first.

| | |
|---|---|
| Action bar | ARM, THR HOLD, TAKEOFF, CIRCUIT, LAND, AUTOROT, STUNTS and the settings gear, icon + short label, the active mode lit (autopilot modes blue, ARMED red, THR HOLD orange). One row on desktop and phones (icons with tiny labels, 42 px tall); a slim 52 px column on the left in phone landscape so the field gets the width; the stunt picker opens under it |
| Settings (gear) | slides in as a 320 px side panel on desktop, up from the bottom on phones; closes on an outside tap, the close button or Esc. Groups: FLYING (gyro, pitch curve, crash protection, wind), ASSISTS (alt hold, auto-yaw, tail gyro), SENSORS (sensors, estimator), VIEW (view: pilot / chase / pad close-up, camera view: auto / always), SESSION (landings and best, flight time, rescues, stunts, last stunt result, reset, help). Rows show their value; green when an assist is on, red only for crash protection off |
| Status line | the existing cue, the one place for "what is happening" |
| Toasts | events only (landed, crashed, crash protection, recovered, battery failsafe, stunt result, GPS reset, refused commands), decided in Rust (`Toast::ev`); shown for 3 s in a strip just above the sticks, never over the centre of the view. Mode-change and setting toasts are dropped: the status line, the lit button or the settings row already says it |
| Readouts | ALT, V/S, SPEED, BATTERY. HEADSPEED appears only outside 90..105%, in throttle hold or in an autorotation. THROTTLE is gone (the left stick shows it). In REAL mode one blue EST badge at the end replaces the four "(EST)" labels |
| Estimator pill | REAL only: one pill under the status line, `EKF OK`, `GPS REJECTED`, `GPS LOST`, `GPS RESET`, `BARO ONLY`, `CAMERA TRACKING`, ... with a coloured dot; tap it for the full sensor panel |
| Assist pills | small pills between the sticks for what is on: ALT HOLD, AUTO-YAW, TAIL RATE, 3D PITCH, REAL SENSORS |
| Camera view | the 120 x 90 px picture-in-picture above the right stick (below) |
| Touch | no tooltips and no key-letter hints when the device has no hover and a coarse pointer; desktop keeps both. Keyboard shortcuts are unchanged; Esc also closes the settings |

UI chrome coverage, `web/test/browser.mjs`: the union of the bounding boxes of every painted UI element (own
background, border, text or icon), excluding the 3D canvas, sampled on a 2 px grid, 3.6 s after each step so
toasts have faded. Touch emulation for the phones. Before = the previous build of the page.

| Viewport | state | before | after |
|---|---|---|---|
| Phone portrait 390 x 844 | idle on the pad | 53.2% | 24.7% |
| | hover hold | 53.8% | 26.6% |
| | hover hold, REAL sensors | 63.6% | 33.1% |
| Phone landscape 844 x 390 | idle | 58.7% | 22.0% |
| | hover hold | 61.3% | 22.6% |
| | hover hold, REAL sensors | 67.7% | 28.4% |
| Desktop 960 x 600 | idle | 34.4% | 17.9% |
| | hover hold | 38.3% | 18.5% |
| | hover hold, REAL sensors | 41.8% | 21.9% |

Most of what is left is the two 150 px sticks (about 14% of a portrait phone on their own).

## Tests: Rust vs JS

`cargo test --release -- --nocapture --test-threads=1` in `rust/` (44 suites: the 26 below plus 7 `t8_*` stunt suites (including the full matrices), 7 `t9_*` assist suites and 4 `t10_*` landing transit / camera suites; plus `tests/step.rs`, about 10 s; the 7 `t7_ekf_*`
suites are the EKF vs BASIC table above) against the JS harnesses `node test*.js rc-heli-field.html` on the original page. The JS uses Math.random, the Rust
a seeded RNG, so noisy numbers agree within noise rather than to the digit. The REAL rows now run on
the EKF (the default); the BASIC column of the table above is the original estimator.

| Scenario | JS | Rust |
|---|---|---|
| Takeoff -> hold | 3.0 m, vs -0.0 | 3.00 m, vs -0.00 |
| Circuit, 6 laps | 14.9, 14.2, 14.2, 14.2, 14.2, 14.2 s; 4.44-5.02 m/s; alt 5.00-5.00 m; bat 73%; 6 lap toasts | 14.9, 14.1, 14.1, 14.1, 14.2, 14.2 s; 4.44-5.01 m/s; alt 5.00-5.00 m; bat 73%; 6 lap toasts |
| Land after circuit | Butter-soft on Pad A, 0.35 m/s | Butter-soft on the pad, 0.35 m/s |
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
| Battery IDEAL: 8% forced | true 7.5%, 20/20 inputs ignored, touchdown 1.04 m/s (1.04-1.97, median about 1.55, over 24 runs) | true 7.5%, 20/20 ignored, touchdown 0.35 m/s on all 8 seeds 300..307 (was 0.93-2.50, median about 2.0, over 22 seeds: the faster transit gets it down before the pack is flat) |
| Battery REAL: 15% / 8% forced | true 15.7% est 14.5% / true 8.2% est 7.5%, 20/20, 0.36 m/s | true 15.6% est 14.4% / true 8.3% est 7.4%, 20/20, 0.37 m/s |
| REAL hover 30 s | pos std 0.18 / 0.15 m; max 0.65 m from anchor; alt 2.91-3.08; est err pos 0.26 (max 0.63) m, alt -0.00, tilt 0.58 (1.45), hdg 1.42 (1.82) deg; rpm tlm 0.97-1.02 | pos std 0.15 / 0.20 m; max 0.89 m; alt 2.97-3.07; est err pos 0.30 (max 0.82) m, alt 0.00, tilt 0.33 (0.86), hdg 0.14 (0.45) deg; rpm tlm 0.97-1.02 |
| REAL circuit 3 laps | 14.3, 13.8, 14.1 s; pos err 0.50 (1.07) m; corner tilt err 4.45 (6.91) deg; 3.62-4.93 m/s; alt 4.55-5.08 | 14.7, 14.2, 14.3 s; pos err 0.86 (1.23) m; corner tilt err 0.34 (0.81) deg; 4.39-5.14 m/s; alt 4.75-5.20 |
| REAL land x5 | 0.29-0.41 m/s, 0.31-0.69 m from centre | 0.27-0.42 m/s, 0.12-0.54 m from centre (camera; GPS only 0.24-0.70 m) |
| REAL GPS glitch 3.5 m / 2.5 s | moved 2.91 m, back to 0.24 m, hold | moved 0.58 m, back to 0.29 m, hold |
| REAL battery telemetry at 20% | est 18.2%, dips to 14.7%, recovers 15.7%, no failsafe, 41.4 V; hover failsafe true 15.7% est 14.5% | est 18.3%, dips to 14.6%, recovers 15.8%, no failsafe, 41.4 V; hover failsafe true 15.6% est 14.6% |
| AUTOROTATE IDEAL x5 | cut 25.0 m; sink 7.07-7.08, fwd 5.34-5.35 m/s; rpm 0.61-1.00; touchdown 0.44-0.45 m/s; 0.92-1.09 m from pad; 0 rescues, 0 crashes | cut 25.0 m; sink 7.08-7.10, fwd 5.35-5.39 m/s; rpm 0.61-1.00; touchdown 0.44-0.46 m/s; 1.02-1.11 m from pad; 0 rescues, 0 crashes |
| AUTOROTATE REAL x5 | cut 24.9-25.0 m; sink 6.89-6.98 m/s; rpm 0.72-1.01; touchdown 0.61-0.80 m/s; 1.82-3.06 m from pad; 0 rescues, 0 crashes | now cut 24.9-25.0 m, touchdown 0.58-0.64 m/s, 0.80-1.46 m from pad (camera in the flare and cushion), 0 rescues; was cut 25.0 m; sink 6.84-7.03 m/s; rpm 0.71-1.01; touchdown 0.55-0.62 m/s; 1.15-2.05 m from pad; 0 rescues, 0 crashes |

Notes on differences, all within run-to-run noise of the JS version:

- (Before the LAND transit change.) The IDEAL 8% forced landing is noisy in both versions. In this scripted test the pack goes completely
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
  a Pirouette through the exports, buttons, keys (K, M, E, C, J / Esc, Q / Y / V, G, R), the estimator pill and sensor panel,
  the camera view setting, the settings sheet (open, Esc) and a stick takeover with no runtime errors.
- `PW_DIR=<dir with puppeteer-core and @sparticuz/chromium> node web/test/browser.mjs rc-heli-field.html [shots] [before.html]`:
  headless Chromium (puppeteer-core with @sparticuz/chromium from npm, software WebGL; touch emulation for the
  phone viewports). Answers the cdnjs three.js request with the identical three@0.128.0 build from npm because
  cdnjs is not reachable from the build machine. It measures the UI coverage table above (and the before page's,
  when given, failing if the new one isn't lower), then on desktop: action bar order and lit mode, the four core
  readouts, tooltips and key hints, no toast for a mode change, REAL mode with the EST badge and the health pill
  expanding / collapsing the sensor panel (camera status), the settings side panel (every setting toggles and
  back, pitch refused in the air, outside click, Esc, HELP), a REAL LAND with the 120 x 90 picture-in-picture,
  its detection box and lock, clear of the sticks and the status line, the landing toast low on the screen
  and fading, the stunt picker and a Pirouette (result toast and session stats), and ALT HOLD / AUTO-YAW with
  their pills and the climb bar. On a 390 x 844 touch phone: no tooltips or key hints, no horizontal scroll, one row
  of eight 42 px action buttons, sticks in the bottom corners, the climb bar, the picker, the bottom sheet and
  its outside tap. On 844 x 390: the action bar as a column, sticks in the corners. There are no page errors.
  Screenshots: `desktop.png`, `settings-open.png`, `real-landing-pip.png`, `phone-portrait.png`,
  `phone-picker.png`, `phone-settings-open.png`, `phone-landscape.png` and `before-*.png`. `QUICK=1` skips the coverage pass.

The sim costs about 6..12 microseconds per 240 Hz step in wasm (Node) in REAL mode with both estimators and
the camera
running (3.4 before the EKF; the 18-state filter with drag fusion is about 1 microsecond more than the 16-state one,
within the run-to-run spread on the build machine; the camera and target estimator add no measurable cost), and building a frame with the sensor panel takes about 40 microseconds (2.2 KB of JSON).

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

Sizes: wasm 148 KB raw, 91 KB gzip+base64 embedded; bundle 19 KB; page 142,672 bytes (139 KiB). Before this change: wasm 136 KB, bundle 17 KB, page 120,394 bytes.

## Licence

Copyright (C) 2026 Conrad Micallef

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details. You should have received a copy of the GNU General Public
License along with this program; see [LICENSE](LICENSE). If not, see <https://www.gnu.org/licenses/>.

three.js (MIT licence) is loaded from cdnjs at run time and is not part of this repository.
