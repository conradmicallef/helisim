//! Simulated sensors, each on its own clock, with latency queues where the real part has them.
//! Physics always runs on the true state; the controllers read `Est` (see `estimator`).
//!
//! | sensor | rate | model |
//! |---|---|---|
//! | gyro | 240 Hz | white noise 0.6 deg/s, bias random walk, rotor vibration, 40 Hz low-pass |
//! | accelerometer | 240 Hz | specific force, 0.05 m/s2 noise, strong vibration, 20 Hz low-pass |
//! | magnetometer | 50 Hz | 0.9 deg noise plus up to 4 deg offset from motor current |
//! | rangefinder | 20 Hz | 0.05..4 m, 2 cm noise, drops out above 25 deg tilt |
//! | barometer | 25 Hz | 12 cm noise, slow drift, ground-effect downwash and gust errors |
//! | GPS | 10 Hz, 150 ms latency | 0.4 m / 0.8 m noise, wandering bias, rare glitches |
//! | ESC rpm | 10 Hz, 100 ms latency | 10 rpm steps, 0.6% noise, 2% dropped frames |
//! | battery | 10 Hz | 12S pack, OCV curve, sag through pack resistance, 0.6 s low-pass |
//! | downward camera | 20 Hz, 80 ms latency | pad tag detection, 0.7 px noise, see `precland` |

pub use crate::estimator::Est;
use crate::ekf::Ekf;
use crate::estimator::Filter;
use crate::heli::{ge_of, Heli};
use crate::math::*;
use crate::rng::Rng;
use crate::sim::Sim;
use crate::units::{DEG, G, PI, RPM_FULL};
use crate::wind::WINDS;
use std::collections::VecDeque;

pub const CELLS: f64 = 12.0;
pub const R_PACK: f64 = 0.025;
/// The flight controller's (slightly low) estimate of pack resistance for load compensation.
pub const R_FC: f64 = 0.02;
const OCV: [(f64, f64); 7] = [(0.0, 3.2), (0.05, 3.32), (0.15, 3.5), (0.3, 3.68), (0.5, 3.8), (0.8, 4.0), (1.0, 4.2)];

/// Resting cell voltage for a state of charge.
pub fn ocv_cell(s: f64) -> f64 {
    let s = clamp(s, 0.0, 1.0);
    for i in 1..OCV.len() {
        if s <= OCV[i].0 {
            let ((a, va), (b, vb)) = (OCV[i - 1], OCV[i]);
            return va + (vb - va) * (s - a) / (b - a);
        }
    }
    4.2
}
/// State of charge for a resting cell voltage.
pub fn soc_of_cell(v: f64) -> f64 {
    if v <= OCV[0].1 {
        return 0.0;
    }
    for i in 1..OCV.len() {
        if v <= OCV[i].1 {
            let ((a, va), (b, vb)) = (OCV[i - 1], OCV[i]);
            return a + (b - a) * (v - va) / (vb - va);
        }
    }
    1.0
}
/// Pack current in amps.
pub fn pack_i(h: &Heli) -> f64 {
    if h.armed && !h.hold && !h.crashed {
        0.8 + 120.0 * h.motor * h.rpm / 0.85
    } else {
        0.2
    }
}

/// Status shown in the sensor panel.
#[derive(Clone, Debug)]
pub struct SensStatus {
    pub sats: i32,
    pub rf: bool,
    pub esc: bool,
    pub v: f64,
    pub a: f64,
    pub glitch: bool,
    /// GPS antenna (top of the canopy) has no sky: upside down
    pub gps_lost: bool,
    /// reacquiring after a loss: fewer satellites, 3x position noise
    pub gps_reacq: bool,
    /// vibration level (1 = hover at full headspeed)
    pub vib: f64,
}

/// GPS sky-visibility and vibration constants (see README "Sensor effects during stunts").
pub mod stunt_sens {
    /// antenna loses the sky beyond this tilt, deg
    pub const GPS_LOST_TILT: f64 = 110.0;
    /// satellites lost per 15 deg of tilt beyond 30 deg
    pub const SAT_TILT0: f64 = 30.0;
    pub const SAT_TILT_STEP: f64 = 15.0;
    /// reacquisition: 2..4 s with reduced satellites and this much position noise
    pub const REACQ_MIN: f64 = 2.0;
    pub const REACQ_SPAN: f64 = 2.0;
    pub const REACQ_NOISE: f64 = 3.0;
    /// vibration from rotor loading (per g above 1 g) and disc tilt rate (per rad/s above 0.5)
    pub const VIB_LOAD: f64 = 0.35;
    pub const VIB_FLAP: f64 = 0.15;
    pub const VIB_MAX: f64 = 2.5;
}

#[derive(Clone, Copy, Debug)]
struct Glitch {
    off: V3,
    until: f64,
}
#[derive(Clone, Copy, Debug)]
struct GpsSample {
    due: f64,
    tm: f64,
    pm: V3,
    vm: V3,
    /// reported accuracy multiplier (receiver hAcc / nominal)
    acc: f64,
}
#[derive(Clone, Copy, Debug)]
struct EscSample {
    due: f64,
    val: f64,
    drop: bool,
}
#[derive(Clone, Copy, Debug, Default)]
struct Clocks {
    mag: f64,
    baro: f64,
    rf: f64,
    gps: f64,
    esc: f64,
    bat: f64,
}

#[derive(Clone, Debug)]
pub struct Sensors {
    pub f: Filter,
    /// error-state EKF, runs beside the complementary filter
    pub ekf: Ekf,
    pub st: SensStatus,
    pub g_bias: V3,
    g_lp: V3,
    a_lp: V3,
    pub gps_bias: V3,
    gps_q: VecDeque<GpsSample>,
    esc_q: VecDeque<EscSample>,
    clk: Clocks,
    pub baro_drift: f64,
    glitch: Option<Glitch>,
    next_glitch: f64,
    esc_rpm: f64,
    v_lp: Option<f64>,
    i_lp: f64,
    soc: f64,
    sats: i32,
    rf_t: Option<f64>,
    /// GPS sky state: lost since, reacquiring until
    gps_lost_t: Option<f64>,
    pub gps_reacq_until: f64,
    pub gps_reacq_t0: f64,
    /// number of GPS losses and the time of the last reacquire (tests)
    pub gps_losses: u32,
    pub gps_back_t: Option<f64>,
    /// downward camera and the precision-landing target estimator
    pub cam: crate::precland::Camera,
    pub pl: crate::precland::PrecLand,
}

impl Sensors {
    pub fn new() -> Sensors {
        Sensors {
            f: Filter::default(),
            ekf: Ekf::new(),
            st: SensStatus { sats: 14, rf: false, esc: true, v: 50.4, a: 0.0, glitch: false, gps_lost: false, gps_reacq: false, vib: 0.0 },
            g_bias: V3::ZERO,
            g_lp: V3::ZERO,
            a_lp: v3(0.0, G, 0.0),
            gps_bias: V3::ZERO,
            gps_q: VecDeque::new(),
            esc_q: VecDeque::new(),
            clk: Clocks::default(),
            baro_drift: 0.0,
            glitch: None,
            next_glitch: 0.0,
            esc_rpm: 0.0,
            v_lp: None,
            i_lp: 0.0,
            soc: 1.0,
            sats: 14,
            rf_t: None,
            gps_lost_t: None,
            gps_reacq_until: 0.0,
            gps_reacq_t0: 0.0,
            gps_losses: 0,
            gps_back_t: None,
            cam: crate::precland::Camera::new(0x7a6),
            pl: crate::precland::PrecLand::new(),
        }
    }
    pub fn glitching(&self) -> bool {
        self.glitch.is_some()
    }
}

impl Default for Sensors {
    fn default() -> Self {
        Sensors::new()
    }
}

fn mirror(est: &mut Est, h: &Heli) {
    est.p = h.p;
    est.v = h.v;
    est.q = h.q;
    est.w = h.w;
    est.rpm = h.rpm;
    est.bat = h.bat;
    est.bat_a = pack_i(h);
    est.bat_v = CELLS * ocv_cell(h.bat) - est.bat_a * R_PACK;
    est.alt = est.p.y - 0.29;
}

impl Sim {
    pub fn sensors_reset(&mut self) {
        let t = self.sim_t;
        let rng = &mut self.rng;
        let s = &mut self.sens;
        s.f.reset(self.h.p, self.h.q);
        s.ekf.reset(t);
        s.g_bias = v3(rng.rnd1(), rng.rnd1(), rng.rnd1()) * (0.5 * DEG);
        s.g_lp = V3::ZERO;
        s.a_lp = v3(0.0, G, 0.0);
        s.gps_bias = v3(rng.rnd1() * 0.3, rng.rnd1() * 0.5, rng.rnd1() * 0.3);
        s.baro_drift = rng.rnd1() * 0.2;
        s.gps_q.clear();
        s.esc_q.clear();
        s.glitch = None;
        s.next_glitch = t + 120.0 + rng.f() * 60.0;
        s.esc_rpm = self.h.rpm;
        s.v_lp = None;
        s.i_lp = 0.0;
        s.soc = self.h.bat;
        s.clk = Clocks { mag: t, baro: t, rf: t, gps: t, esc: t, bat: t };
        s.gps_lost_t = None;
        s.gps_reacq_until = 0.0;
        s.gps_losses = 0;
        s.gps_back_t = None;
        s.st.gps_lost = false;
        s.st.gps_reacq = false;
        s.cam.reset(t);
        s.pl = crate::precland::PrecLand::new();
        mirror(&mut self.est, &self.h);
    }

    /// Force a GPS glitch of `m` metres for `dur` seconds (tests and demos).
    pub fn force_glitch(&mut self, m: f64, dur: f64) {
        let a = self.rng.f() * PI * 2.0;
        self.sens.glitch = Some(Glitch { off: v3(a.cos() * m, 0.0, a.sin() * m), until: self.sim_t + dur });
    }

    pub(crate) fn sensors_update(&mut self, dt: f64, wind: V3) {
        let t = self.sim_t;
        let wind_lvl = WINDS[self.set.wind].v;
        let real = self.set.real;
        let use_ekf = self.set.ekf;
        let (h, est, rng, s) = (&mut self.h, &mut self.est, &mut self.rng, &mut self.sens);
        // vibration: headspeed squared, plus rotor loading and fast disc tilt (rapid cyclic)
        let load = (h.thrust_g.abs() - 1.0).max(0.0);
        let flap = (h.rotor.f_rate - 0.5).max(0.0);
        let vib = (h.rpm * h.rpm * (1.0 + stunt_sens::VIB_LOAD * load) + stunt_sens::VIB_FLAP * flap * h.rpm).min(stunt_sens::VIB_MAX);
        s.st.vib = s.st.vib + (vib - s.st.vib) * (dt / 0.3).min(1.0);
        s.ekf.vib = vib;
        let a_t = (h.v - h.v_prev) * (1.0 / dt);
        h.v_prev = h.v;
        let q_inv = h.q.inv();
        // gyro: white noise, bias random walk, rotor vibration, then ~40 Hz low-pass
        let bw = 0.02 * DEG * dt.sqrt();
        s.g_bias.x += rng.randn() * bw;
        s.g_bias.y += rng.randn() * bw;
        s.g_bias.z += rng.randn() * bw;
        let gk = dt / (dt + 1.0 / (2.0 * PI * 40.0));
        let gn = |rng: &mut Rng| rng.randn() * 0.6 * DEG + rng.rnd1() * 3.0 * DEG * vib;
        s.g_lp.x += (h.w.x + s.g_bias.x + gn(rng) - s.g_lp.x) * gk;
        s.g_lp.y += (h.w.y + s.g_bias.y + gn(rng) - s.g_lp.y) * gk;
        s.g_lp.z += (h.w.z + s.g_bias.z + gn(rng) - s.g_lp.z) * gk;
        // accelerometer: specific force in the body frame, noise plus strong rotor vibration, ~20 Hz low-pass
        let f_b = v3(a_t.x, a_t.y + G, a_t.z).rot(q_inv);
        let ak = dt / (dt + 1.0 / (2.0 * PI * 20.0));
        let an = |rng: &mut Rng| rng.randn() * 0.05 + rng.rnd1() * 1.5 * vib;
        s.a_lp.x += (f_b.x + an(rng) - s.a_lp.x) * ak;
        s.a_lp.y += (f_b.y + an(rng) - s.a_lp.y) * ak;
        s.a_lp.z += (f_b.z + an(rng) - s.a_lp.z) * ak;
        s.f.attitude(s.g_lp, s.a_lp, dt);
        s.ekf.predict(s.g_lp, s.a_lp, s.esc_rpm, t, dt);
        // magnetometer, 50 Hz
        if t >= s.clk.mag {
            s.clk.mag += 0.02;
            let meas = heading_of(h.q) + rng.randn() * 0.9 * DEG + (h.motor / 0.85) * 4.0 * DEG;
            s.f.mag(meas);
            s.ekf.mag(meas, s.i_lp, s.esc_rpm);
        }
        s.f.predict(s.a_lp, t, dt);
        let up_t = up_of(h.q);
        let up_e = up_of(s.f.q);
        // rangefinder, 20 Hz, 0.05 to 4 m, drops out above 25 degrees of tilt
        if t >= s.clk.rf {
            s.clk.rf += 0.05;
            let range = if up_t.y > (25.0 * DEG).cos() { (h.p.y - 0.25 * up_t.y) / up_t.y } else { -1.0 };
            if range > 0.05 && range < 4.0 {
                s.rf_t = Some(t);
                let rm = range + rng.randn() * 0.02;
                let py = rm * up_e.y + 0.25 * up_e.y;
                s.f.range(py);
                s.ekf.range(rm);
            }
        }
        let rf_active = matches!(s.rf_t, Some(r) if t - r < 0.15);
        s.st.rf = rf_active;
        // barometer, 25 Hz: noise, slow drift, downwash error in ground effect, gust error
        if t >= s.clk.baro {
            s.clk.baro += 0.04;
            s.baro_drift = clamp(s.baro_drift + rng.randn() * 0.03 * 0.2 - s.baro_drift * 0.04 / 90.0, -1.0, 1.0);
            let ge_e = (ge_of(h.p.y, up_t.y) - 1.0) / 0.18 * 0.4;
            let gust = (wind.x.hypot(wind.z) - wind_lvl) * 0.12;
            let meas = h.p.y + rng.randn() * 0.12 + s.baro_drift + ge_e + gust;
            s.f.baro(meas, rf_active);
            s.ekf.baro(meas);
        }
        // GPS, 10 Hz with 150 ms latency: noise, wandering bias, rare glitches. The antenna sits on
        // top of the canopy: satellites drop with tilt, the fix is lost upside down, and after a
        // loss it reacquires for 2..4 s with fewer satellites and 3x position noise.
        let tilt_t = clamp(up_t.y, -1.0, 1.0).acos() / DEG;
        if tilt_t > stunt_sens::GPS_LOST_TILT {
            if s.gps_lost_t.is_none() {
                s.gps_lost_t = Some(t);
                s.gps_losses += 1;
            }
        } else if s.gps_lost_t.take().is_some() {
            s.gps_reacq_t0 = t;
            s.gps_reacq_until = t + stunt_sens::REACQ_MIN + rng.f() * stunt_sens::REACQ_SPAN;
            s.gps_back_t = Some(t);
        }
        let lost = s.gps_lost_t.is_some();
        let reacq = !lost && t < s.gps_reacq_until;
        s.st.gps_lost = lost;
        s.st.gps_reacq = reacq;
        if t >= s.clk.gps {
            s.clk.gps += 0.1;
            let bh = 0.1 * 0.1f64.sqrt();
            let b = &mut s.gps_bias;
            b.x += rng.randn() * bh - b.x * 0.1 / 40.0;
            b.z += rng.randn() * bh - b.z * 0.1 / 40.0;
            b.y += rng.randn() * bh * 1.5 - b.y * 0.1 / 40.0;
            let hb = b.x.hypot(b.z);
            if hb > 1.2 {
                b.x *= 1.2 / hb;
                b.z *= 1.2 / hb;
            }
            b.y = clamp(b.y, -2.0, 2.0);
            if s.glitch.is_none() && t > s.next_glitch {
                let m = 2.0 + rng.f() * 2.0;
                let dur = 1.5 + rng.f() * 1.5;
                let a = rng.f() * PI * 2.0;
                s.glitch = Some(Glitch { off: v3(a.cos() * m, 0.0, a.sin() * m), until: t + dur });
                s.next_glitch = t + 120.0 + rng.f() * 60.0;
            }
            if matches!(s.glitch, Some(g) if t > g.until) {
                s.glitch = None;
            }
            // noise drawn in the original order (bias, glitch, position, velocity, satellites) so
            // runs without stunts keep their exact realisation
            let nk = if reacq { stunt_sens::REACQ_NOISE } else { 1.0 };
            let b = s.gps_bias;
            let n = 0.4 * nk;
            let mut pm = v3(h.p.x + b.x + rng.randn() * n, h.p.y + b.y + rng.randn() * 0.8 * nk, h.p.z + b.z + rng.randn() * n);
            if let Some(g) = s.glitch {
                pm += g.off;
            }
            let vm = v3(h.v.x + rng.randn() * 0.12 * nk, h.v.y + rng.randn() * 0.25 * nk, h.v.z + rng.randn() * 0.12 * nk);
            if rng.f() < 0.01 {
                let d = if rng.f() < 0.5 { -1 } else { 1 };
                s.sats = (s.sats + d).clamp(11, 17);
            }
            // satellites in view of the tilted antenna
            let tilt_loss = ((tilt_t - stunt_sens::SAT_TILT0).max(0.0) / stunt_sens::SAT_TILT_STEP).floor() as i32;
            let mut sats = (s.sats - tilt_loss).max(0);
            if reacq {
                let f = clamp((t - s.gps_reacq_t0) / (s.gps_reacq_until - s.gps_reacq_t0), 0.0, 1.0);
                sats = sats.min(5 + (f * 4.0) as i32);
            }
            if lost || sats < 4 {
                sats = 0;
            }
            s.st.sats = sats;
            if sats > 0 {
                s.gps_q.push_back(GpsSample { due: t + 0.15, tm: t, pm, vm, acc: nk });
            }
        }
        let resets0 = s.ekf.h.resets;
        while let Some(g) = s.gps_q.front().copied() {
            if g.due > t {
                break;
            }
            s.gps_q.pop_front();
            s.f.gps(g.tm, g.pm, g.vm);
            s.ekf.gps_acc(g.tm, g.pm, g.vm, g.acc);
        }
        let ekf_reset = if s.ekf.h.resets != resets0 { Some(s.ekf.h.reset_dp) } else { None };
        // ESC headspeed telemetry: 10 Hz, 100 ms latency, 10 rpm steps, noise, dropped frames
        if t >= s.clk.esc {
            s.clk.esc += 0.1;
            let val = crate::jsonw::js_round(h.rpm * (1.0 + rng.randn() * 0.006) * RPM_FULL / 10.0) * 10.0 / RPM_FULL;
            let drop = rng.f() < 0.02;
            s.esc_q.push_back(EscSample { due: t + 0.1, val, drop });
        }
        while let Some(e) = s.esc_q.front().copied() {
            if e.due > t {
                break;
            }
            s.esc_q.pop_front();
            s.st.esc = !e.drop;
            if !e.drop {
                s.esc_rpm = e.val;
            }
        }
        // battery: 12S pack, resting voltage from charge, sag through pack resistance, 10 Hz with a 0.6 s low-pass
        if t >= s.clk.bat {
            s.clk.bat += 0.1;
            let i = pack_i(h);
            let v = CELLS * ocv_cell(h.bat) - i * R_PACK;
            let vm = v + rng.randn() * 0.05;
            let im = i * (1.0 + rng.randn() * 0.02);
            match s.v_lp {
                None => {
                    s.v_lp = Some(vm);
                    s.i_lp = im;
                }
                Some(ref mut vl) => {
                    *vl += (vm - *vl) * 0.1 / 0.6;
                    s.i_lp += (im - s.i_lp) * 0.1 / 0.6;
                }
            }
            let vl = s.v_lp.unwrap_or(vm);
            s.soc = soc_of_cell((vl + s.i_lp * R_FC) / CELLS); // load-compensated
            s.st.v = vl;
            s.st.a = s.i_lp;
        }
        est.wind = est.wind.lerp(wind, (dt / 5.0).min(1.0));
        s.st.glitch = s.glitch.is_some();
        s.ekf.h.gps_lost = s.st.gps_lost;
        if real {
            if use_ekf && s.ekf.init {
                est.p = s.ekf.p;
                est.v = s.ekf.v;
                est.q = s.ekf.q;
                est.w = s.ekf.w;
            } else {
                est.p = s.f.p;
                est.v = s.f.v;
                est.q = s.f.q;
                est.w = s.f.w;
            }
            est.rpm = s.esc_rpm;
            est.bat = s.soc;
            est.bat_v = s.v_lp.unwrap_or(50.4);
            est.bat_a = s.i_lp;
            est.alt = est.p.y - 0.29;
        } else {
            mirror(est, h);
            est.wind = wind;
        }
        // EKF position reset: shift the autopilot's world targets by the same jump so the
        // heli doesn't fly to the new estimate (ArduCopter's handle_ekf_xy_reset)
        if let (true, true, Some(dp)) = (real, use_ekf, ekf_reset) {
            if let Some(a) = self.auto.as_mut() {
                a.anchor += dp;
                a.entry += dp;
                a.pad_ref += v3(dp.x, 0.0, dp.z);
                a.tgt += v3(dp.x, 0.0, dp.z);
            }
            self.sens.pl.shift(dp);
            self.toast("GPS reset", format!("The estimate jumped {} m to the GPS fix", crate::jsonw::to_fixed(dp.x.hypot(dp.z), 1)), "bad");
        }
        // downward camera and the pad target estimator (REAL only; IDEAL flies the surveyed pad)
        if real {
            let s = &mut self.sens;
            let dets = s.cam.step(t, &self.h);
            let ekf_on = use_ekf && s.ekf.init;
            let var0 = if ekf_on { s.ekf.sig_pos().powi(2) } else { 1.0 };
            let (sig_tilt, sig_hgt) = if ekf_on { (s.ekf.sig_tilt(), s.ekf.sig_hgt()) } else { (1.0 * DEG, 0.15) };
            s.pl.tick(t, dt, var0);
            s.pl.push(t, self.est.p, self.est.q);
            for d in dets {
                s.pl.update(d, t, sig_tilt, sig_hgt, self.est.w);
            }
        }
    }
}
