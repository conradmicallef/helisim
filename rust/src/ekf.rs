//! Error-state (indirect) extended Kalman filter, in the style of a small ArduPilot EKF / PX4 ECL.
//! Runs beside the complementary filter in `estimator`; `Settings::ekf` picks which one the
//! controllers read in REAL sensor mode.
//!
//! Nominal state: position p (world, m), velocity v (world, m/s), attitude q (body -> world),
//! gyro bias bg (rad/s), accelerometer bias ba (m/s2), barometer offset bb (m).
//! Error state (16): dp, dv, dtheta (world-frame rotation error, q_true = dq(dtheta) * q),
//! dbg, dba, dbb.
//!
//! - Predict at the IMU rate (240 Hz) from the bias-corrected gyro and accelerometer.
//! - Updates on each sensor clock: delayed GPS position + velocity (innovations taken against a
//!   0.5 s buffer of past nominal states at the measurement time), barometer (with the offset
//!   state, noise inflated in ground effect), rangefinder height, magnetometer heading (yaw-only
//!   gain, noise inflated with motor current), optional gravity aiding when |f| is close to g.
//! - Every update is gated on its normalised innovation squared against a chi-square threshold.
//!   GPS horizontal position rejected for more than `GPS_RESET_T` resets p/v to GPS with inflated
//!   covariance.
//! - Sequential scalar updates in Joseph form, symmetry enforcement and variance limits after
//!   every step; the error state is injected into the nominal state after each scalar update.
//!
//! No dependencies: small fixed-size matrix helpers below.

use crate::math::*;
use crate::units::{DEG, G};
use std::collections::VecDeque;

pub const N: usize = 16;
type Mat = [[f64; N]; N];
type Vn = [f64; N];

const IP: usize = 0;
const IV: usize = 3;
const IT: usize = 6;
const IBG: usize = 9;
const IBA: usize = 12;
const IBB: usize = 15;

/// Noise and gate values. Sensor noise comes from the `sensors` models; see README for the reasoning.
pub mod tune {
    use crate::units::DEG;
    /// gyro white noise per sample (rad/s)
    pub const GYRO_N: f64 = 0.6 * DEG;
    /// gyro rotor vibration per sample at full headspeed (uniform +-3 deg/s -> sigma 3/sqrt 3), scales with rpm^2
    pub const GYRO_VIB: f64 = 1.732 * DEG;
    /// accelerometer white noise per sample (m/s2)
    pub const ACC_N: f64 = 0.05;
    /// accelerometer vibration at full headspeed (uniform +-1.5 m/s2 -> sigma 0.87), scales with rpm^2
    pub const ACC_VIB: f64 = 0.866;
    /// process noise multiplier on the IMU terms (covers low-pass lag and unmodelled effects)
    pub const IMU_Q_SCALE: f64 = 2.0;
    /// gyro bias random walk (rad/s/sqrt s); sensor model is 0.02 deg/s/sqrt s
    pub const GYRO_BIAS_RW: f64 = 0.03 * DEG;
    /// accelerometer bias random walk (m/s2/sqrt s); the model has no accel bias, keep it small
    pub const ACC_BIAS_RW: f64 = 0.003;
    /// baro offset random walk (m/sqrt s); sensor drift is 0.006 m per 0.04 s step = 0.03 m/sqrt s
    pub const BARO_RW: f64 = 0.04;
    /// small position process noise (m/sqrt s)
    pub const POS_RW: f64 = 0.02;
    /// GPS horizontal position sigma: 0.4 m noise plus the wandering bias (stationary ~0.45 m)
    pub const GPS_POS_H: f64 = 0.6;
    /// GPS vertical position sigma. The vertical bias (stationary ~0.67 m, 40 s correlation time)
    /// is not white: fused at 10 Hz as white noise it would be averaged into the height. The
    /// equivalent white sigma of a first-order Gauss-Markov error sampled every 0.1 s is
    /// sigma_b * sqrt(2 tau / dt) ~ 19 m, so GPS height is only a weak long-term reference for
    /// the baro offset; the rangefinder is the strong one near the ground.
    pub const GPS_POS_V: f64 = 19.0;
    pub const GPS_VEL_H: f64 = 0.15;
    pub const GPS_VEL_V: f64 = 0.3;
    /// barometer sigma: 0.12 m noise plus gust error
    pub const BARO_N: f64 = 0.2;
    /// extra baro sigma inside ground effect (downwash error up to 0.4 m)
    pub const BARO_GE: f64 = 0.5;
    pub const RNG_N: f64 = 0.05;
    /// magnetometer sigma: 0.9 deg noise, inflated by up to 4 deg with motor current
    pub const MAG_N: f64 = 1.2 * DEG;
    pub const MAG_CUR: f64 = 1.5 * DEG;
    /// compass-motor compensation (like COMPASS_MOT): heading offset per unit of motor load,
    /// calibrated 10% low against the true 4 deg at full motor
    pub const MAG_MOT: f64 = 3.6 * DEG;
    /// GPS wandering bias, stationary sigma per horizontal axis. It can't be observed (no other
    /// absolute position reference), so it is a "consider" term: added to the reported
    /// horizontal sigma, not to the state covariance.
    pub const GPS_BIAS_SIG: f64 = 0.45;
    /// gravity aiding: accelerometer averaged over 0.1 s, used when ||f| - g| < GRAV_WIN and slow rates
    pub const GRAV_AID: bool = true;
    pub const GRAV_N: f64 = 1.0;
    pub const GRAV_WIN: f64 = 0.25;
    /// chi-square gates on the normalised innovation squared (test ratio = NIS / gate, reject > 1)
    pub const GATE_GPS_POS: f64 = 11.8; // 2 DOF, p = 0.0027 (3 sigma equivalent)
    pub const GATE_GPS_HGT: f64 = 25.0; // 1 DOF, 5 sigma
    pub const GATE_GPS_VEL: f64 = 21.1; // 3 DOF, p = 1e-4
    pub const GATE_BARO: f64 = 25.0;
    pub const GATE_RNG: f64 = 25.0;
    pub const GATE_MAG: f64 = 16.0; // 4 sigma (compass offset with current is a bias, not noise)
    pub const GATE_GRAV: f64 = 11.3; // 3 DOF, p = 0.01
    /// GPS position rejected for longer than this: reset p/v to GPS
    pub const GPS_RESET_T: f64 = 2.0;
    pub const RESET_POS_SIG: f64 = 0.7;
    /// number of most recent rejected GPS innovations averaged for the reset jump
    pub const RESET_AVG: usize = 5;
    pub const RESET_VEL_SIG: f64 = 0.5;
}
use tune::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Init = 0,
    Ok = 1,
    Rangefinder = 2,
    GpsRejected = 3,
    GpsReset = 4,
    BaroOnly = 5,
    GpsLost = 6,
}
impl Status {
    pub fn name(self) -> &'static str {
        match self {
            Status::Init => "INIT",
            Status::Ok => "OK",
            Status::Rangefinder => "RANGEFINDER",
            Status::GpsRejected => "GPS REJECTED",
            Status::GpsReset => "GPS RESET",
            Status::BaroOnly => "BARO ONLY",
            Status::GpsLost => "GPS LOST",
        }
    }
}

/// Health and innovation statistics for the sensor panel and tests.
#[derive(Clone, Debug, Default)]
pub struct Health {
    /// latest test ratios (NIS / gate; > 1 rejected)
    pub tr_gps_pos: f64,
    pub tr_gps_vel: f64,
    pub tr_gps_hgt: f64,
    pub tr_baro: f64,
    pub tr_rng: f64,
    pub tr_mag: f64,
    pub tr_grav: f64,
    /// rejected sample counts
    pub rej_gps: u32,
    pub rej_gps_vel: u32,
    pub rej_baro: u32,
    pub rej_rng: u32,
    pub rej_mag: u32,
    pub rej_grav: u32,
    pub fused_gps: u32,
    pub fused_grav: u32,
    pub resets: u32,
    pub last_reset: Option<f64>,
    /// horizontal position jump of the last reset (the autopilot shifts its targets by it)
    pub reset_dp: V3,
    pub gps_rejecting: bool,
    pub last_rng: Option<f64>,
    pub last_gps_any: f64,
    /// receiver has no fix (antenna upside down); set by the sensor model
    pub gps_lost: bool,
}

#[derive(Clone, Copy, Debug)]
struct Hist {
    t: f64,
    p: V3,
    v: V3,
}

#[derive(Clone, Debug)]
pub struct Ekf {
    pub init: bool,
    pub p: V3,
    pub v: V3,
    pub q: Quat,
    pub bg: V3,
    pub ba: V3,
    pub bb: f64,
    /// bias-corrected body rates (what the gyro loop reads)
    pub w: V3,
    pub cov: Mat,
    pub h: Health,
    /// vibration level for the IMU process noise (set by the sensor model each step; 0 = rpm^2)
    pub vib: f64,
    pub t: f64,
    hist: VecDeque<Hist>,
    // initialisation
    f_init: V3,
    n_init: u32,
    mag0: Option<f64>,
    baro0: Option<f64>,
    // GPS rejection timing and the innovations seen while rejecting (for the reset jump)
    last_pos_pass: f64,
    rej_inn: VecDeque<(V3, V3)>,
    // gravity aiding accumulator
    f_sum: V3,
    w_max: f64,
    n_sum: u32,
}

impl Default for Ekf {
    fn default() -> Self {
        Ekf::new()
    }
}

fn zero_m() -> Mat {
    [[0.0; N]; N]
}
fn sel(i: usize) -> Vn {
    let mut h = [0.0; N];
    h[i] = 1.0;
    h
}
fn dot(a: &Vn, b: &Vn) -> f64 {
    let mut s = 0.0;
    for i in 0..N {
        s += a[i] * b[i];
    }
    s
}
fn skew(a: V3) -> [[f64; 3]; 3] {
    [[0.0, -a.z, a.y], [a.z, 0.0, -a.x], [-a.y, a.x, 0.0]]
}
fn comp(v: V3, i: usize) -> f64 {
    match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}
fn e(i: usize) -> V3 {
    match i {
        0 => v3(1.0, 0.0, 0.0),
        1 => v3(0.0, 1.0, 0.0),
        _ => v3(0.0, 0.0, 1.0),
    }
}
/// Rotation vector -> quaternion.
fn quat_rv(r: V3) -> Quat {
    let a = r.length();
    if a < 1e-12 {
        Quat { x: r.x * 0.5, y: r.y * 0.5, z: r.z * 0.5, w: 1.0 }.normalize()
    } else {
        Quat::axis_angle(r * (1.0 / a), a)
    }
}
/// Shortest rotation taking unit vector `a` onto unit vector `b` (three.js setFromUnitVectors).
fn quat_from_units(a: V3, b: V3) -> Quat {
    let r = a.dot(b) + 1.0;
    let q = if r < 1e-9 {
        if a.x.abs() > a.z.abs() {
            Quat { x: -a.y, y: a.x, z: 0.0, w: 0.0 }
        } else {
            Quat { x: 0.0, y: -a.z, z: a.y, w: 0.0 }
        }
    } else {
        let c = a.cross(b);
        Quat { x: c.x, y: c.y, z: c.z, w: r }
    };
    q.normalize()
}

/// Normalised innovation squared y' S^-1 y for k <= 3 measurements, S = H P H' + diag(r).
fn nis(p: &Mat, hs: &[Vn], ys: &[f64], rs: &[f64]) -> f64 {
    let k = hs.len();
    let mut s = [[0.0f64; 4]; 3];
    for i in 0..k {
        let mut ph = [0.0; N];
        for a in 0..N {
            let mut acc = 0.0;
            for b in 0..N {
                acc += p[a][b] * hs[i][b];
            }
            ph[a] = acc;
        }
        for j in 0..k {
            s[i][j] = dot(&hs[j], &ph);
        }
        s[i][i] += rs[i];
        s[i][3] = ys[i];
    }
    // Gaussian elimination with partial pivoting on [S | y]
    for c in 0..k {
        let mut piv = c;
        for r in c + 1..k {
            if s[r][c].abs() > s[piv][c].abs() {
                piv = r;
            }
        }
        s.swap(c, piv);
        if s[c][c].abs() < 1e-15 {
            return f64::INFINITY;
        }
        for r in c + 1..k {
            let f = s[r][c] / s[c][c];
            for cc in c..4 {
                s[r][cc] -= f * s[c][cc];
            }
        }
    }
    let mut x = [0.0f64; 3];
    for c in (0..k).rev() {
        let mut acc = s[c][3];
        for cc in c + 1..k {
            acc -= s[c][cc] * x[cc];
        }
        x[c] = acc / s[c][c];
    }
    let mut out = 0.0;
    for i in 0..k {
        out += ys[i] * x[i];
    }
    out
}

/// Variance limits per error state (min, max).
fn var_lim(i: usize) -> (f64, f64) {
    match i {
        0..=2 => (1e-6, 1e4),
        3..=5 => (1e-6, 100.0),
        6..=8 => (1e-9, 1.0),
        9..=11 => (1e-12, (5.0 * DEG) * (5.0 * DEG)),
        12..=14 => (1e-10, 4.0),
        _ => (1e-6, 100.0),
    }
}

impl Ekf {
    pub fn new() -> Ekf {
        Ekf {
            init: false,
            p: V3::ZERO,
            v: V3::ZERO,
            q: Quat::IDENTITY,
            bg: V3::ZERO,
            ba: V3::ZERO,
            bb: 0.0,
            w: V3::ZERO,
            cov: zero_m(),
            h: Health::default(),
            vib: 0.0,
            t: 0.0,
            hist: VecDeque::new(),
            f_init: V3::ZERO,
            n_init: 0,
            mag0: None,
            baro0: None,
            last_pos_pass: 0.0,
            rej_inn: VecDeque::new(),
            f_sum: V3::ZERO,
            w_max: 0.0,
            n_sum: 0,
        }
    }

    /// Back to uninitialised; it initialises on the ground from the first accelerometer, compass,
    /// barometer and GPS samples.
    pub fn reset(&mut self, t: f64) {
        *self = Ekf::new();
        self.t = t;
        self.h.last_gps_any = t;
    }

    fn initialise(&mut self, gps_p: V3, gps_v: V3, hdg: f64) {
        let f = if self.n_init > 0 { self.f_init * (1.0 / self.n_init as f64) } else { v3(0.0, G, 0.0) };
        let up_b = f.normalize();
        let q_tilt = quat_from_units(up_b, Y); // body up -> world up
        let q = (Quat::axis_angle(Y, wrap(hdg - heading_of(q_tilt))) * q_tilt).normalize();
        self.q = q;
        // normally on the ground (flat terrain at y = 0, body origin 0.29 m above the skids); if
        // the baro says it is airborne (re-initialisation in flight), start from baro height
        let airborne = self.baro0.is_some_and(|b| b > 1.5);
        let y0 = if airborne { self.baro0.unwrap_or(0.29) } else { 0.29 };
        self.p = v3(gps_p.x, y0, gps_p.z);
        self.v = if airborne { gps_v } else { v3(gps_v.x, 0.0, gps_v.z) };
        self.bg = V3::ZERO;
        self.ba = V3::ZERO;
        self.bb = if airborne { 0.0 } else { self.baro0.map_or(0.0, |b| b - self.p.y) };
        let mut c = zero_m();
        let d = [
            GPS_POS_H * GPS_POS_H,
            0.1 * 0.1,
            GPS_POS_H * GPS_POS_H,
            0.2 * 0.2,
            0.2 * 0.2,
            0.2 * 0.2,
            (2.0 * DEG).powi(2),
            (5.0 * DEG).powi(2),
            (2.0 * DEG).powi(2),
            (0.4 * DEG).powi(2),
            (0.4 * DEG).powi(2),
            (0.4 * DEG).powi(2),
            0.05 * 0.05,
            0.05 * 0.05,
            0.05 * 0.05,
            0.5 * 0.5,
        ];
        for i in 0..N {
            c[i][i] = d[i];
        }
        if airborne {
            c[IP + 1][IP + 1] = 1.0;
            c[IBB][IBB] = 1.0;
            for i in 0..3 {
                c[IV + i][IV + i] = 1.0;
                c[IT + i][IT + i] = (10.0 * DEG).powi(2);
            }
        }
        self.cov = c;
        self.init = true;
        self.last_pos_pass = self.t;
        self.h.last_gps_any = self.t;
        self.hist.clear();
    }

    /// IMU prediction: gyro (rad/s, body) and specific force (m/s2, body), estimated headspeed for
    /// the vibration noise.
    pub fn predict(&mut self, gyro: V3, accel: V3, rpm: f64, t: f64, dt: f64) {
        self.t = t;
        if !self.init {
            self.f_init += accel;
            self.n_init += 1;
            if self.n_init > 120 {
                self.f_init = self.f_init * 0.5;
                self.n_init /= 2;
            }
            self.w = gyro;
            return;
        }
        let w = gyro - self.bg;
        self.w = w;
        let fc = accel - self.ba;
        let fw = fc.rot(self.q);
        let a = v3(fw.x, fw.y - G, fw.z);
        self.p += self.v * dt + a * (0.5 * dt * dt);
        self.v += a * dt;
        let ang = w.length() * dt;
        if ang > 1e-12 {
            self.q = (self.q * Quat::axis_angle(w * (1.0 / w.length()), ang)).normalize();
        }
        // F = I + A dt as a sparse list (row, col, value)
        let mut fe: [(u8, u8, f64); N + 30] = [(0, 0, 0.0); N + 30];
        let mut nf = 0;
        for i in 0..N {
            fe[nf] = (i as u8, i as u8, 1.0);
            nf += 1;
        }
        let sk = skew(fw);
        let rc: [V3; 3] = [e(0).rot(self.q), e(1).rot(self.q), e(2).rot(self.q)]; // columns of R
        for i in 0..3 {
            fe[nf] = ((IP + i) as u8, (IV + i) as u8, dt);
            nf += 1;
            for j in 0..3 {
                if sk[i][j] != 0.0 {
                    fe[nf] = ((IV + i) as u8, (IT + j) as u8, -sk[i][j] * dt);
                    nf += 1;
                }
            }
        }
        // P = F P F' (the bias couplings -R dt are applied as dense 3x3 blocks below)
        let p = &self.cov;
        let mut fp = zero_m();
        for &(r, c, v) in &fe[..nf] {
            let (r, c) = (r as usize, c as usize);
            for j in 0..N {
                fp[r][j] += v * p[c][j];
            }
        }
        for i in 0..3 {
            for j in 0..3 {
                let rij = -comp(rc[j], i) * dt;
                for k in 0..N {
                    fp[IV + i][k] += rij * p[IBA + j][k];
                    fp[IT + i][k] += rij * p[IBG + j][k];
                }
            }
        }
        let mut np = zero_m();
        for &(r, c, v) in &fe[..nf] {
            let (r, c) = (r as usize, c as usize);
            for i in 0..N {
                np[i][r] += fp[i][c] * v;
            }
        }
        for i in 0..3 {
            for j in 0..3 {
                let rij = -comp(rc[j], i) * dt;
                for k in 0..N {
                    np[k][IV + i] += fp[k][IBA + j] * rij;
                    np[k][IT + i] += fp[k][IBG + j] * rij;
                }
            }
        }
        // Q from the sensor models
        let vib = if self.vib > 0.0 { self.vib } else { rpm.max(0.0) * rpm.max(0.0) };
        let sg2 = (GYRO_N * GYRO_N + (GYRO_VIB * vib).powi(2)) * IMU_Q_SCALE;
        let sa2 = (ACC_N * ACC_N + (ACC_VIB * vib).powi(2)) * IMU_Q_SCALE;
        for i in 0..3 {
            np[IP + i][IP + i] += POS_RW * POS_RW * dt;
            np[IV + i][IV + i] += sa2 * dt * dt;
            np[IT + i][IT + i] += sg2 * dt * dt;
            np[IBG + i][IBG + i] += GYRO_BIAS_RW * GYRO_BIAS_RW * dt;
            np[IBA + i][IBA + i] += ACC_BIAS_RW * ACC_BIAS_RW * dt;
        }
        np[IBB][IBB] += BARO_RW * BARO_RW * dt;
        self.cov = np;
        self.condition();
        // history for delayed GPS fusion
        self.hist.push_back(Hist { t, p: self.p, v: self.v });
        while let Some(hh) = self.hist.front() {
            if hh.t < t - 0.5 {
                self.hist.pop_front();
            } else {
                break;
            }
        }
        // gravity aiding accumulator (0.1 s)
        if GRAV_AID {
            self.f_sum += accel;
            self.w_max = self.w_max.max(w.length());
            self.n_sum += 1;
            if self.n_sum >= 24 {
                let fm = self.f_sum * (1.0 / self.n_sum as f64);
                let wm = self.w_max;
                self.f_sum = V3::ZERO;
                self.n_sum = 0;
                self.w_max = 0.0;
                self.gravity(fm, wm);
            }
        }
        if !self.finite() {
            self.reset(t);
        }
    }

    fn finite(&self) -> bool {
        let s = self.p.x + self.p.y + self.p.z + self.v.x + self.v.y + self.v.z + self.q.x + self.q.y + self.q.z + self.q.w;
        let s = s + self.bg.x + self.bg.y + self.bg.z + self.ba.x + self.ba.y + self.ba.z + self.bb;
        if !s.is_finite() {
            return false;
        }
        for i in 0..N {
            if !self.cov[i][i].is_finite() || self.cov[i][i] <= 0.0 {
                return false;
            }
        }
        true
    }

    /// Symmetry and variance limits (scaling a row and column keeps P positive semi-definite).
    fn condition(&mut self) {
        let p = &mut self.cov;
        for i in 0..N {
            for j in i + 1..N {
                let m = 0.5 * (p[i][j] + p[j][i]);
                p[i][j] = m;
                p[j][i] = m;
            }
        }
        for i in 0..N {
            let (lo, hi) = var_lim(i);
            if p[i][i] > hi {
                let s = (hi / p[i][i]).sqrt();
                for j in 0..N {
                    p[i][j] *= s;
                    p[j][i] *= s;
                }
            }
            if !(p[i][i] >= lo) {
                p[i][i] = lo;
            }
        }
    }

    fn inject(&mut self, dx: &Vn) {
        self.p += v3(dx[IP], dx[IP + 1], dx[IP + 2]);
        self.v += v3(dx[IV], dx[IV + 1], dx[IV + 2]);
        self.q = (quat_rv(v3(dx[IT], dx[IT + 1], dx[IT + 2])) * self.q).normalize();
        self.bg += v3(dx[IBG], dx[IBG + 1], dx[IBG + 2]);
        self.ba += v3(dx[IBA], dx[IBA + 1], dx[IBA + 2]);
        self.bb += dx[IBB];
        let (dp, dv) = (v3(dx[IP], dx[IP + 1], dx[IP + 2]), v3(dx[IV], dx[IV + 1], dx[IV + 2]));
        for hh in self.hist.iter_mut() {
            hh.p += dp;
            hh.v += dv;
        }
    }

    /// One scalar update in Joseph form; `mask` lists error states that get no gain.
    fn fuse1(&mut self, h: &Vn, y: f64, r: f64, mask: &[usize]) -> Vn {
        let p = &self.cov;
        let mut ph = [0.0; N];
        for i in 0..N {
            let mut acc = 0.0;
            for k in 0..N {
                if h[k] != 0.0 {
                    acc += p[i][k] * h[k];
                }
            }
            ph[i] = acc;
        }
        let s = dot(h, &ph) + r;
        let mut dx = [0.0; N];
        if !(s > 0.0) || !y.is_finite() {
            return dx;
        }
        let mut k = [0.0; N];
        for i in 0..N {
            k[i] = ph[i] / s;
        }
        for &m in mask {
            k[m] = 0.0;
        }
        // Joseph form (I-KH) P (I-KH)' + K r K', valid for any gain K
        let p = &mut self.cov;
        for i in 0..N {
            for j in 0..N {
                p[i][j] += -k[i] * ph[j] - ph[i] * k[j] + s * k[i] * k[j];
            }
        }
        for i in 0..N {
            dx[i] = k[i] * y;
        }
        self.inject(&dx);
        self.condition();
        dx
    }

    /// Sequential scalar updates of a gated measurement group (innovations are corrected for the
    /// states already updated in the group).
    fn fuse_seq(&mut self, hs: &[Vn], ys: &[f64], rs: &[f64], mask: &[usize]) {
        let mut tot = [0.0; N];
        for j in 0..hs.len() {
            let y = ys[j] - dot(&hs[j], &tot);
            let dx = self.fuse1(&hs[j], y, rs[j], mask);
            for i in 0..N {
                tot[i] += dx[i];
            }
        }
    }

    /// Compass heading (rad); pack current (A) and headspeed for the motor compensation and the
    /// noise inflation.
    pub fn mag(&mut self, hdg: f64, current: f64, rpm: f64) {
        let load = clamp((current - 0.8) / (120.0 * rpm.max(0.5)), 0.0, 1.2);
        let hdg = hdg - MAG_MOT * load;
        if !self.init {
            self.mag0 = Some(hdg);
            return;
        }
        if up_of(self.q).y < 0.5 {
            return; // heading ill-defined beyond 60 degrees of tilt
        }
        let y = wrap(hdg - heading_of(self.q));
        let r = MAG_N * MAG_N + (MAG_CUR * load).powi(2);
        let h = sel(IT + 1);
        let tr = nis(&self.cov, &[h], &[y], &[r]) / GATE_MAG;
        self.h.tr_mag = tr;
        if tr > 1.0 {
            self.h.rej_mag += 1;
            return;
        }
        // yaw-only: no gain into roll / pitch, so compass errors can't tilt the attitude
        self.fuse_seq(&[h], &[y], &[r], &[IT, IT + 2]);
    }

    /// Barometer height (m, of the body origin, including its offset).
    pub fn baro(&mut self, z: f64) {
        if !self.init {
            self.baro0 = Some(z);
            return;
        }
        let up = up_of(self.q);
        let disc = self.p.y + 0.29 * up.y;
        let mut r = BARO_N * BARO_N;
        if disc < 1.4 {
            r += BARO_GE * BARO_GE;
        }
        let mut h = sel(IP + 1);
        h[IBB] = 1.0;
        let y = z - (self.p.y + self.bb);
        let tr = nis(&self.cov, &[h], &[y], &[r]) / GATE_BARO;
        self.h.tr_baro = tr;
        if tr > 1.0 {
            self.h.rej_baro += 1;
            return;
        }
        self.fuse_seq(&[h], &[y], &[r], &[]);
    }

    /// Rangefinder slant range (m) along the body down axis from the sensor 0.25 m below the
    /// origin; flat terrain at y = 0. Converted with the filter's own attitude.
    pub fn range(&mut self, range: f64) {
        if !self.init {
            return;
        }
        let up = up_of(self.q);
        if up.y < (25.0 * DEG).cos() || !(0.05..4.0).contains(&range) {
            return;
        }
        let z = range * up.y + 0.25 * up.y;
        let h = sel(IP + 1);
        let y = z - self.p.y;
        let r = RNG_N * RNG_N;
        let tr = nis(&self.cov, &[h], &[y], &[r]) / GATE_RNG;
        self.h.tr_rng = tr;
        if tr > 1.0 {
            self.h.rej_rng += 1;
            return;
        }
        self.h.last_rng = Some(self.t);
        self.fuse_seq(&[h], &[y], &[r], &[]);
    }

    /// Gravity aiding from the 0.1 s mean specific force, only when |f| is close to g and the
    /// rates are low (not in turns or manoeuvres).
    fn gravity(&mut self, fm: V3, wmax: f64) {
        if (fm.length() - G).abs() > GRAV_WIN || wmax > 0.35 {
            return;
        }
        let qi = self.q.inv();
        let gy = v3(0.0, G, 0.0);
        let pred = gy.rot(qi) + self.ba;
        // d(R' g)/d(dtheta) = R' [g]x  (column j = R' (g x e_j))
        let cols: [V3; 3] = [gy.cross(e(0)).rot(qi), gy.cross(e(1)).rot(qi), gy.cross(e(2)).rot(qi)];
        let mut hs = [[0.0; N]; 3];
        let mut ys = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                hs[i][IT + j] = comp(cols[j], i);
            }
            hs[i][IBA + i] = 1.0;
            ys[i] = comp(fm, i) - comp(pred, i);
        }
        let rs = [GRAV_N * GRAV_N; 3];
        let tr = nis(&self.cov, &hs, &ys, &rs) / GATE_GRAV;
        self.h.tr_grav = tr;
        if tr > 1.0 {
            self.h.rej_grav += 1;
            return;
        }
        self.h.fused_grav += 1;
        // tilt and accel bias only: keep heading on the compass
        self.fuse_seq(&hs, &ys, &rs, &[IT + 1]);
    }

    fn hist_at(&self, tm: f64) -> Option<Hist> {
        let mut best: Option<Hist> = None;
        for hh in self.hist.iter() {
            if best.map_or(true, |b| (hh.t - tm).abs() < (b.t - tm).abs()) {
                best = Some(*hh);
            }
        }
        best
    }

    /// GPS fix measured at `tm` (arriving now, 150 ms later): position and velocity.
    /// Innovations are formed against the buffered nominal state at `tm` (the delayed fusion
    /// horizon) and the correction is applied to the current state and to the buffer.
    pub fn gps(&mut self, tm: f64, pm: V3, vm: V3) {
        self.gps_acc(tm, pm, vm, 1.0);
    }

    /// GPS fix with the receiver's reported accuracy as a multiplier on the nominal noise
    /// (e.g. 3 while reacquiring after a loss).
    pub fn gps_acc(&mut self, tm: f64, pm: V3, vm: V3, acc: f64) {
        if !self.init {
            if let Some(hd) = self.mag0 {
                self.initialise(pm, vm, hd);
            }
            return;
        }
        let Some(hp) = self.hist_at(tm) else { return };
        self.h.fused_gps += 1;
        let (ix, iz, iy) = (pm.x - hp.p.x, pm.z - hp.p.z, pm.y - hp.p.y);
        let iv = vm - hp.v;
        let a2 = acc.max(1.0) * acc.max(1.0);
        let rvh = GPS_VEL_H * GPS_VEL_H * a2;
        let rvv = GPS_VEL_V * GPS_VEL_V * a2;
        let rph = GPS_POS_H * GPS_POS_H * a2;
        let rpv = GPS_POS_V * GPS_POS_V * a2;
        let trv = nis(&self.cov, &[sel(IV), sel(IV + 1), sel(IV + 2)], &[iv.x, iv.y, iv.z], &[rvh, rvv, rvh]) / GATE_GPS_VEL;
        let trp = nis(&self.cov, &[sel(IP), sel(IP + 2)], &[ix, iz], &[rph, rph]) / GATE_GPS_POS;
        let trh = nis(&self.cov, &[sel(IP + 1)], &[iy], &[rpv]) / GATE_GPS_HGT;
        self.h.tr_gps_pos = trp;
        self.h.tr_gps_vel = trv;
        self.h.tr_gps_hgt = trh;
        let mut hs: Vec<Vn> = Vec::with_capacity(6);
        let mut ys: Vec<f64> = Vec::with_capacity(6);
        let mut rs: Vec<f64> = Vec::with_capacity(6);
        if trv <= 1.0 {
            hs.extend_from_slice(&[sel(IV), sel(IV + 1), sel(IV + 2)]);
            ys.extend_from_slice(&[iv.x, iv.y, iv.z]);
            rs.extend_from_slice(&[rvh, rvv, rvh]);
        } else {
            self.h.rej_gps_vel += 1;
        }
        let mut reset = false;
        if trp <= 1.0 {
            hs.extend_from_slice(&[sel(IP), sel(IP + 2)]);
            ys.extend_from_slice(&[ix, iz]);
            rs.extend_from_slice(&[rph, rph]);
            self.last_pos_pass = tm;
            self.h.gps_rejecting = false;
            self.rej_inn.clear();
        } else {
            self.h.rej_gps += 1;
            self.h.gps_rejecting = true;
            self.rej_inn.push_back((v3(ix, 0.0, iz), v3(iv.x, 0.0, iv.z)));
            if self.rej_inn.len() > RESET_AVG {
                self.rej_inn.pop_front();
            }
            reset = tm - self.last_pos_pass > GPS_RESET_T;
        }
        if trh <= 1.0 {
            hs.push(sel(IP + 1));
            ys.push(iy);
            rs.push(rpv);
        }
        if !hs.is_empty() {
            self.h.last_gps_any = self.t;
        }
        self.fuse_seq(&hs, &ys, &rs, &[]);
        if reset {
            self.gps_reset(tm);
            self.h.last_gps_any = self.t;
        }
    }

    /// GPS reset: horizontal position and velocity jump to GPS (the mean of the last few
    /// rejected innovations, which are all taken at their own time horizon, so one noisy sample
    /// doesn't set the new position), covariance inflated.
    fn gps_reset(&mut self, tm: f64) {
        let n = self.rej_inn.len().max(1) as f64;
        let (mut dp, mut dv) = (V3::ZERO, V3::ZERO);
        for (a, b) in self.rej_inn.iter() {
            dp += *a;
            dv += *b;
        }
        let (dp, dv) = (dp * (1.0 / n), dv * (1.0 / n));
        self.rej_inn.clear();
        self.p += dp;
        self.v += dv;
        for hh in self.hist.iter_mut() {
            hh.p += dp;
            hh.v += dv;
        }
        for &i in &[IP, IP + 2, IV, IV + 2] {
            for j in 0..N {
                self.cov[i][j] = 0.0;
                self.cov[j][i] = 0.0;
            }
        }
        self.cov[IP][IP] = RESET_POS_SIG * RESET_POS_SIG;
        self.cov[IP + 2][IP + 2] = RESET_POS_SIG * RESET_POS_SIG;
        self.cov[IV][IV] = RESET_VEL_SIG * RESET_VEL_SIG;
        self.cov[IV + 2][IV + 2] = RESET_VEL_SIG * RESET_VEL_SIG;
        self.last_pos_pass = tm;
        self.h.resets += 1;
        self.h.reset_dp = dp;
        self.h.last_reset = Some(self.t);
        self.h.gps_rejecting = false;
        self.h.tr_gps_pos = 0.0;
    }

    pub fn status(&self) -> Status {
        if !self.init {
            return Status::Init;
        }
        if self.h.last_reset.is_some_and(|r| self.t - r < 3.0) {
            return Status::GpsReset;
        }
        if self.h.gps_lost {
            return Status::GpsLost;
        }
        if self.t - self.h.last_gps_any > 1.0 {
            return Status::BaroOnly;
        }
        if self.h.gps_rejecting {
            return Status::GpsRejected;
        }
        if self.h.last_rng.is_some_and(|r| self.t - r < 0.15) {
            return Status::Rangefinder;
        }
        Status::Ok
    }

    /// 1-sigma per-axis horizontal position (m), including the GPS bias consider term.
    pub fn sig_pos(&self) -> f64 {
        (0.5 * (self.cov[IP][IP] + self.cov[IP + 2][IP + 2]) + GPS_BIAS_SIG * GPS_BIAS_SIG).sqrt()
    }
    pub fn sig_x(&self) -> f64 {
        (self.cov[IP][IP] + GPS_BIAS_SIG * GPS_BIAS_SIG).sqrt()
    }
    pub fn sig_z(&self) -> f64 {
        (self.cov[IP + 2][IP + 2] + GPS_BIAS_SIG * GPS_BIAS_SIG).sqrt()
    }
    pub fn sig_hgt(&self) -> f64 {
        self.cov[IP + 1][IP + 1].sqrt()
    }
    /// 1-sigma tilt (rad): largest of the two horizontal rotation error axes.
    pub fn sig_tilt(&self) -> f64 {
        self.cov[IT][IT].max(self.cov[IT + 2][IT + 2]).sqrt()
    }
    pub fn sig_tilt_x(&self) -> f64 {
        self.cov[IT][IT].sqrt()
    }
    pub fn sig_tilt_z(&self) -> f64 {
        self.cov[IT + 2][IT + 2].sqrt()
    }
    pub fn sig_hdg(&self) -> f64 {
        self.cov[IT + 1][IT + 1].sqrt()
    }
    pub fn var(&self, i: usize) -> f64 {
        self.cov[i][i]
    }
    /// Smallest and largest covariance diagonal and the largest asymmetry (for tests).
    pub fn cov_stats(&self) -> (f64, f64, f64) {
        let (mut mn, mut mx, mut asym) = (f64::INFINITY, 0.0f64, 0.0f64);
        for i in 0..N {
            mn = mn.min(self.cov[i][i]);
            mx = mx.max(self.cov[i][i]);
            for j in 0..N {
                asym = asym.max((self.cov[i][j] - self.cov[j][i]).abs());
            }
        }
        (mn, mx, asym)
    }
    pub fn is_finite(&self) -> bool {
        self.finite()
    }
}
