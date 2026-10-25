//! State estimate the flight controller, autopilot, crash protection, failsafes and HUD read.
//! IDEAL: mirrors the truth. REAL: the EKF in `ekf` (default) or this BASIC complementary filter fed by `sensors`:
//! gyro integration with accelerometer gravity correction (minus GPS-derived manoeuvre
//! acceleration) and a gyro-bias integrator, slow compass heading fusion, accelerometer
//! dead-reckoning corrected by rangefinder, barometer and delayed GPS (matched against a 0.5 s
//! history of past estimates, with a soft innovation gate).

use crate::math::*;
use crate::units::G;
use std::collections::VecDeque;

/// What the controllers see.
#[derive(Clone, Debug)]
pub struct Est {
    pub p: V3,
    pub v: V3,
    pub q: Quat,
    pub w: V3,
    pub alt: f64,
    pub rpm: f64,
    pub bat: f64,
    pub bat_v: f64,
    pub bat_a: f64,
    pub wind: V3,
}

impl Est {
    pub fn new() -> Est {
        Est {
            p: v3(0.0, 0.29, 0.0),
            v: V3::ZERO,
            q: Quat::IDENTITY,
            w: V3::ZERO,
            alt: 0.0,
            rpm: 0.0,
            bat: 1.0,
            bat_v: 50.4,
            bat_a: 0.0,
            wind: V3::ZERO,
        }
    }
}

impl Default for Est {
    fn default() -> Self {
        Est::new()
    }
}

#[derive(Clone, Copy, Debug)]
struct Hist {
    t: f64,
    p: V3,
    v: V3,
}

/// Complementary filter state.
#[derive(Clone, Debug, Default)]
pub struct Filter {
    pub p: V3,
    pub v: V3,
    pub q: Quat,
    pub w: V3,
    /// manoeuvre acceleration from GPS velocity differences (world frame, horizontal)
    pub a_cmp: V3,
    /// gyro bias estimate
    pub b_i: V3,
    hist: VecDeque<Hist>,
    last_gv: Option<V3>,
}

impl Filter {
    pub fn reset(&mut self, p: V3, q: Quat) {
        self.p = p;
        self.v = V3::ZERO;
        self.q = q;
        self.w = V3::ZERO;
        self.a_cmp = V3::ZERO;
        self.b_i = V3::ZERO;
        self.hist.clear();
        self.last_gv = None;
    }

    /// Gyro integration with gravity correction from the (low-passed) accelerometer.
    pub fn attitude(&mut self, gyro: V3, accel: V3, dt: f64) {
        let qe_inv = self.q.inv();
        let mut wc = gyro - self.b_i;
        self.w = wc;
        let um = -self.a_cmp.rot(qe_inv) + accel;
        let ml = um.length();
        if ml > 0.5 * G && ml < 1.6 * G {
            let um = um * (1.0 / ml);
            let ue = Y.rot(qe_inv);
            let e = um.cross(ue);
            wc += e * 0.5;
            self.b_i += e * (-0.03 * dt);
        }
        let ang = wc.length() * dt;
        if ang > 1e-12 {
            self.q = (self.q * Quat::axis_angle(wc.normalize(), ang)).normalize();
        }
    }

    /// Compass heading measurement (50 Hz): slow heading fusion and yaw-bias correction.
    pub fn mag(&mut self, meas: f64) {
        let err = wrap(meas - heading_of(self.q));
        self.q = (Quat::axis_angle(Y, err * 0.6 * 0.02) * self.q).normalize();
        self.b_i.y -= err * 0.02 * 0.02;
    }

    /// Velocity / position prediction from the accelerometer, and the history for delayed fixes.
    pub fn predict(&mut self, accel: V3, t: f64, dt: f64) {
        let mut a_w = accel.rot(self.q);
        a_w.y -= G;
        self.v += a_w * dt;
        self.p += self.v * dt;
        self.hist.push_back(Hist { t, p: self.p, v: self.v });
        while let Some(h) = self.hist.front() {
            if h.t < t - 0.5 {
                self.hist.pop_front();
            } else {
                break;
            }
        }
    }

    /// Rangefinder height of the body origin.
    pub fn range(&mut self, py: f64) {
        let e = py - self.p.y;
        self.p.y += 0.3 * e;
        self.v.y += 0.12 * e;
    }

    /// Barometer innovation, de-weighted while the rangefinder is in use.
    pub fn baro(&mut self, meas_y: f64, rf_active: bool) {
        let e = meas_y - self.p.y;
        let wb = if rf_active && self.p.y < 3.3 { 0.1 } else { 1.0 };
        self.p.y += 0.04 * wb * e;
        self.v.y += 0.02 * wb * e;
    }

    /// Delayed GPS fix taken at time `tm`.
    pub fn gps(&mut self, tm: f64, pm: V3, vm: V3) {
        let mut hp = match self.hist.front() {
            Some(h) => *h,
            None => return,
        };
        for e in self.hist.iter() {
            if e.t <= tm {
                hp = *e;
            } else {
                break;
            }
        }
        let (ix, iz, iy) = (pm.x - hp.p.x, pm.z - hp.p.z, pm.y - hp.p.y);
        let gate = if ix.hypot(iz) > 2.5 { 0.3 } else { 1.0 }; // soft innovation gate against jumps
        self.p.x += 0.08 * gate * ix;
        self.p.z += 0.08 * gate * iz;
        self.p.y += 0.004 * iy;
        self.v.x += 0.02 * gate * ix + 0.12 * (vm.x - hp.v.x);
        self.v.z += 0.02 * gate * iz + 0.12 * (vm.z - hp.v.z);
        self.v.y += 0.03 * (vm.y - hp.v.y);
        if let Some(lg) = self.last_gv {
            let k = 0.3;
            self.a_cmp.x += ((vm.x - lg.x) / 0.1 - self.a_cmp.x) * k;
            self.a_cmp.z += ((vm.z - lg.z) / 0.1 - self.a_cmp.z) * k;
        }
        self.last_gv = Some(vm);
    }
}
