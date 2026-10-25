//! Flying assists for manual flight: a throttle / collective curve that is finer around the hover
//! point, ALT HOLD (left stick sets climb rate, the controller flies the collective), AUTO-YAW
//! (nose follows the airflow, coordinated turns), the tail gyro RATE mode, and the ground-proximity
//! predictor used by crash protection. All of it reads only the estimated state.

use crate::heli::TMAX;
use crate::math::*;
use crate::sim::Sim;
use crate::units::{DEG, G};

/// Throttle curve: slope `K` within `D` of stick around the hover point, steeper outside.
pub mod curve {
    pub const K: f64 = 0.5;
    pub const D: f64 = 0.12;
}
/// Altitude hold constants.
pub mod ah {
    /// stick deadband around the hover point (fraction of full stick)
    pub const DB: f64 = 0.08;
    /// full-stick climb / descent rate, m/s
    pub const UP: f64 = 3.0;
    pub const DOWN: f64 = 2.0;
    /// engages above this height (m), once airborne
    pub const ENGAGE: f64 = 0.5;
    /// near the ground: descent limited to `NEAR_V` m/s below `NEAR_ALT`, tapering to `TD_V` at touchdown
    pub const NEAR_ALT: f64 = 1.5;
    pub const NEAR_V: f64 = 1.0;
    pub const TD_V: f64 = 0.5;
    /// height error -> climb rate gain, and the vertical speed loop gain
    pub const KH: f64 = 1.2;
    pub const KZ: f64 = 2.5;
    /// climb-rate demand slew, m/s^2
    pub const VZ_SLEW: f64 = 4.0;
    /// engagement offset (bumpless) bleeds out with this time constant, s
    pub const OFF_TAU: f64 = 1.5;
    /// collective slew while handing back to direct, per second
    pub const HANDBACK: f64 = 1.0;
}
/// Auto-yaw constants.
pub mod ay {
    /// airspeed blend: off below V0, full above V1 (m/s)
    pub const V0: f64 = 2.0;
    pub const V1: f64 = 3.0;
    /// heading error -> yaw rate gain, 1/s
    pub const K: f64 = 2.0;
    /// yaw-rate demand limit, rad/s
    pub const MAX: f64 = 2.5;
}
/// Ground-proximity crash protection.
pub mod gp {
    /// look-ahead and trigger, s
    pub const LOOK: f64 = 2.0;
    pub const T_TRIG: f64 = 2.0;
    /// "moving" or "tilted": horizontal speed m/s, tilt deg
    pub const HS: f64 = 2.5;
    pub const TILT: f64 = 20.0;
    /// a normal pilot landing: slow, level, gentle
    pub const LAND_HS: f64 = 1.5;
    pub const LAND_TILT: f64 = 15.0;
    pub const LAND_VS: f64 = 1.2;
    /// only while actually sinking faster than this, m/s (a level skim in ground effect is fine)
    pub const VS_MIN: f64 = 0.3;
    /// vertical acceleration estimate: low-pass time constant (s) and clamp (m/s^2)
    pub const AY_TAU: f64 = 0.5;
    pub const AY_LIM: f64 = 1.0;
}

/// Hover collective out of ground effect at full headspeed.
pub const HOVER_COLL: f64 = G / TMAX;

/// Stick position of the hover point for a pitch curve (normal 0..1 -> 0..1, 3D 0..1 -> -1..1).
pub fn hover_stick(p3d: bool) -> f64 {
    if p3d {
        (HOVER_COLL + 1.0) / 2.0
    } else {
        HOVER_COLL
    }
}

/// Throttle curve in stick units: monotonic, f(0) = 0, f(1) = 1, f(h) = h, slope `curve::K`
/// within `curve::D` of the hover point `h`.
pub fn thr_curve(s: f64, h: f64) -> f64 {
    use curve::*;
    let s = clamp(s, 0.0, 1.0);
    let (lo, hi) = (h - D, h + D);
    if s < lo {
        s * (h - K * D) / lo
    } else if s > hi {
        h + K * D + (s - hi) * (1.0 - h - K * D) / (1.0 - hi)
    } else {
        h + K * (s - h)
    }
}

/// Inverse of `thr_curve`.
pub fn thr_curve_inv(y: f64, h: f64) -> f64 {
    use curve::*;
    let y = clamp(y, 0.0, 1.0);
    let (ylo, yhi) = (h - K * D, h + K * D);
    if y < ylo {
        y * (h - D) / ylo
    } else if y > yhi {
        h + D + (y - yhi) * (1.0 - h - D) / (1.0 - yhi)
    } else {
        h + (y - h) / K
    }
}

/// ALT HOLD climb-rate demand from the throttle stick (0..1), around the hover point `h`.
pub fn climb_of_stick(s: f64, h: f64) -> f64 {
    let d = s - h;
    if d.abs() <= ah::DB {
        0.0
    } else if d > 0.0 {
        ah::UP * ((d - ah::DB) / (1.0 - h - ah::DB)).min(1.0)
    } else {
        ah::DOWN * ((d + ah::DB) / (h - ah::DB)).max(-1.0)
    }
}

/// First time within `look` s at which `alt + vy t + ay t^2 / 2` reaches 0, or infinity.
pub fn t_contact(alt: f64, vy: f64, ay: f64, look: f64) -> f64 {
    if alt <= 0.0 {
        return 0.0;
    }
    let a = 0.5 * ay;
    let t = if a.abs() < 1e-6 {
        if vy < 0.0 {
            -alt / vy
        } else {
            f64::INFINITY
        }
    } else {
        let disc = vy * vy - 4.0 * a * alt;
        if disc < 0.0 {
            f64::INFINITY
        } else {
            let sq = disc.sqrt();
            let (r1, r2) = ((-vy - sq) / (2.0 * a), (-vy + sq) / (2.0 * a));
            let (r1, r2) = (r1.min(r2), r1.max(r2));
            if r1 > 0.0 {
                r1
            } else if r2 > 0.0 {
                r2
            } else {
                f64::INFINITY
            }
        }
    };
    if t <= look {
        t
    } else {
        f64::INFINITY
    }
}

/// ALT HOLD state.
#[derive(Clone, Copy, Debug, Default)]
pub struct AltHold {
    /// controller flying the collective now
    pub active: bool,
    /// held height (m, estimated)
    pub target: f64,
    /// climb rate demand after the slew, and the stick demand (for the climb bar)
    pub vz: f64,
    pub vz_stick: f64,
    /// bumpless engagement offset on the collective
    pub off: f64,
    /// handing back to the direct collective: slew from the last value
    pub handback: bool,
    /// stick was in the deadband last step
    pub in_db: bool,
    /// stick just centred: bringing the climb rate to zero before latching the height
    pub braking: bool,
}

impl Sim {
    /// Collective for a throttle stick position, through the pitch curve and the throttle curve.
    pub fn coll_of_stick(&self, s: f64) -> f64 {
        let s = if self.set.thr_curve { thr_curve(s, hover_stick(self.set.pitch3d)) } else { s };
        if self.set.pitch3d {
            s * 2.0 - 1.0
        } else {
            s
        }
    }
    /// Stick position that gives a collective (inverse of `coll_of_stick`), for bumpless handovers.
    pub fn stick_of_coll(&self, c: f64) -> f64 {
        let s = if self.set.pitch3d { clamp((c + 1.0) / 2.0, 0.0, 1.0) } else { clamp(c, 0.0, 1.0) };
        if self.set.thr_curve {
            thr_curve_inv(s, hover_stick(self.set.pitch3d))
        } else {
            s
        }
    }

    /// Track the estimated vertical acceleration (for the ground-proximity predictor).
    pub(crate) fn track_ay(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let vy = self.est.v.y;
        let raw = (vy - self.vy_prev) / dt;
        self.vy_prev = vy;
        if raw.is_finite() {
            self.ay_f += (clamp(raw, -30.0, 30.0) - self.ay_f) * (dt / gp::AY_TAU).min(1.0);
        }
    }

    /// Ground-proximity rule: predicted contact soon while moving fast or tilted. Returns the reason.
    pub(crate) fn ground_prox(&self) -> Option<String> {
        use crate::jsonw::to_fixed;
        let est = &self.est;
        let up = up_of(est.q);
        let tilt = clamp(up.y, -1.0, 1.0).acos() / DEG;
        let (alt, vy) = (est.alt, est.v.y);
        let hs = est.v.hyp_xz();
        if hs < gp::LAND_HS && tilt < gp::LAND_TILT && vy > -gp::LAND_VS {
            return None; // a normal landing
        }
        if (hs <= gp::HS && tilt <= gp::TILT) || vy > -gp::VS_MIN {
            return None;
        }
        // vertical acceleration only counts while sinking (noise at level flight would false-trigger)
        let ay = if vy < -0.2 { clamp(self.ay_f, -gp::AY_LIM, gp::AY_LIM) } else { 0.0 };
        let tc = t_contact(alt, vy, ay, gp::LOOK);
        if tc >= gp::T_TRIG {
            return None;
        }
        Some(if hs > gp::HS {
            format!("Too low and fast: {} m/s at {} m", to_fixed(hs, 1), to_fixed(alt.max(0.0), 1))
        } else {
            format!("Too low and tilted: {}° at {} m", to_fixed(tilt, 0), to_fixed(alt.max(0.0), 1))
        })
    }

    /// Manual collective for this step: ALT HOLD when it is engaged, else the throttle stick through
    /// the curves, with a slew while handing back from ALT HOLD.
    pub(crate) fn manual_coll(&mut self, dt: f64) -> f64 {
        let direct = self.coll_of_stick(self.sticks.thr);
        let last = self.h.coll_cmd;
        let est = &self.est;
        let h = &self.h;
        let up = up_of(est.q);
        let alt = est.alt;
        let can = self.set.alt_hold && h.armed && !h.hold && !h.crashed && up.y > 0.5 && est.rpm > 0.8;
        let mut a = self.ah;
        if a.active && (!can || h.grounded) {
            a.active = false;
            a.handback = true;
        } else if !a.active && can && !h.grounded && h.airborne && alt > ah::ENGAGE {
            a.active = true;
            a.handback = false;
            a.target = alt;
            a.vz = est.v.y;
            a.off = 0.0;
            a.in_db = false;
            a.braking = true;
        }
        let hs = hover_stick(self.set.pitch3d);
        a.vz_stick = climb_of_stick(self.sticks.thr, hs);
        let out = if a.active {
            let db = a.vz_stick == 0.0;
            if db && !a.in_db {
                a.braking = true; // stick back in the deadband: stop the climb, then hold where it stopped
            }
            a.in_db = db;
            if a.braking && (!db || (est.v.y.abs() < 0.15 && a.vz.abs() < 0.15)) {
                a.braking = false;
                a.target = alt;
            }
            let mut vz = if !db || a.braking { if db { 0.0 } else { a.vz_stick } } else { clamp(ah::KH * (a.target - alt), -1.5, 2.0) };
            if !db || a.braking {
                a.target = alt;
            }
            if alt < ah::NEAR_ALT {
                let lim = ah::TD_V + (ah::NEAR_V - ah::TD_V) * (alt.max(0.0) / ah::NEAR_ALT);
                vz = vz.max(-lim);
            }
            let step = ah::VZ_SLEW * dt;
            a.vz += clamp(vz - a.vz, -step, step);
            let w = est.wind;
            let (rx, ry, rz) = (est.v.x - w.x, est.v.y, est.v.z - w.z);
            let dk = 0.22 + 0.035 * (rx * rx + ry * ry + rz * rz).sqrt();
            let c0 = self.coll_for(a.vz, up, rx, ry, rz, dk, ah::KZ, 0.0);
            if !self.ah.active {
                a.off = last - c0; // bumpless: start from the collective it had
            }
            a.off *= 1.0 - (dt / ah::OFF_TAU).min(1.0);
            let lo = if self.set.pitch3d { -1.0 } else { 0.0 };
            clamp(c0 + a.off, lo, 1.0)
        } else if a.handback {
            let d = direct - last;
            let step = ah::HANDBACK * dt;
            if d.abs() <= step {
                a.handback = false;
                direct
            } else {
                last + d.signum() * step
            }
        } else {
            direct
        };
        self.ah = a;
        out
    }

    /// AUTO-YAW yaw-rate demand (rad/s, + = heading increasing) for manual flight: nose toward the
    /// air-relative velocity, plus the coordinated-turn rate g tan(bank) / V, blended in from
    /// 2 to 3 m/s of airspeed. The pilot's yaw stick fades the heading term out.
    pub(crate) fn auto_yaw_rate(&self, yaw_stick: f64) -> f64 {
        if !self.set.auto_yaw || self.h.grounded {
            return 0.0;
        }
        let est = &self.est;
        let up = up_of(est.q);
        if up.y < 0.5 {
            return 0.0;
        }
        let (vx, vz) = (est.v.x - est.wind.x, est.v.z - est.wind.z);
        let sp = vx.hypot(vz);
        // blended in by airspeed, and by ground speed so a hover in wind keeps heading hold
        let gs = est.v.hyp_xz();
        let k = smooth01((sp - ay::V0) / (ay::V1 - ay::V0)) * smooth01((gs - ay::V0) / (ay::V1 - ay::V0));
        if k <= 0.0 {
            return 0.0;
        }
        let hdg = heading_of(est.q);
        let e = wrap((-vx).atan2(-vz) - hdg);
        // backward flight is deliberate: fade out beyond 120 deg of sideslip
        let kd = clamp((150.0 * DEG - e.abs()) / (30.0 * DEG), 0.0, 1.0);
        let (_, right) = fwd_right(est.q);
        let bank = up.dot(right).atan2(up.y);
        let coord = -G * bank.tan() / sp.max(ay::V1);
        let pilot = 1.0 - (2.0 * yaw_stick.abs()).min(1.0);
        clamp(k * kd * (ay::K * e * pilot + coord), -ay::MAX, ay::MAX)
    }
}
