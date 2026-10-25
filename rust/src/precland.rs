//! Downward camera and precision landing (REAL sensor mode).
//!
//! Camera: mounted under the fuselage looking straight down the body -y axis, 70 deg horizontal
//! field of view, 640 x 480 equivalent, 20 Hz, 80 ms latency. The image's up direction is the
//! nose. It detects the square AprilTag-style marker on the pad from geometry: the tag centre and
//! at least 85% of its area must be in the image, the tag must span 12 px .. 90% of the image
//! height, the body tilt must be under 35 deg, and the motion blur (angular rate x exposure) under
//! 6 px. A detection reports the tag centre in pixels and the tag yaw relative to the image, with
//! 0.7 px noise, blur-dependent noise, 4% dropouts and rare false detections at a random spot.
//!
//! Target estimator (like ArduPilot's AC_PrecLand): the pad position in the flight controller's
//! own (EKF) world frame, 2 states with an isotropic variance. Each detection is turned into a
//! ground point by casting the pixel ray from the camera with the estimated attitude and height
//! at the capture time (an 80 ms history of the estimate), and fused with a chi-square gate on
//! the normalised innovation (2 DOF, 11.8). Before the first lock the prior is the surveyed pad
//! position in the GPS frame with the EKF's position uncertainty; acquisition needs 5 consistent
//! detections, so a stray false detection can't capture the target. Process noise 0.3 m/sqrt s
//! lets the target follow the slow wander of the GPS-referenced estimate, so the landing is
//! flown relative to the pad. Detections rejected for 1.5 s in a row with a consistent offset
//! reset the target to them (like the EKF's GPS reset). The tag yaw gives the pad heading in the
//! estimate's frame for the final alignment.

use crate::heli::{Heli, PAD, TAG_SIZE};
use crate::math::*;
use crate::rng::Rng;
use crate::units::DEG;
use std::collections::VecDeque;

/// Camera and estimator constants.
pub mod cam {
    use crate::math::{v3, V3};
    pub const W: f64 = 640.0;
    pub const H: f64 = 480.0;
    /// horizontal field of view 70 deg: focal length W / 2 / tan(35 deg), px
    pub const F: f64 = 457.01;
    pub const RATE: f64 = 0.05;
    pub const LAT: f64 = 0.08;
    /// exposure time, s (blur = angular rate x exposure x focal length)
    pub const EXPO: f64 = 0.006;
    pub const BLUR_MAX: f64 = 6.0;
    pub const MIN_PX: f64 = 12.0;
    /// largest tag span as a fraction of the image height
    pub const MAX_FRAC: f64 = 0.9;
    /// visible fraction of the tag area needed
    pub const MIN_AREA: f64 = 0.85;
    pub const MAX_TILT: f64 = 35.0;
    pub const PIX_N: f64 = 0.7;
    /// extra pixel noise per px of blur
    pub const BLUR_N: f64 = 0.25;
    pub const YAW_N: f64 = 1.0;
    pub const DROP: f64 = 0.04;
    /// false detection probability per frame (a random spot in the image)
    pub const FAKE_P: f64 = 0.002;
    /// lens position in the body frame
    pub const MOUNT: V3 = v3(0.0, -0.15, -0.05);
    /// height of the tag surface above the ground
    pub const TAG_Y: f64 = 0.006;
    // target estimator
    pub const GATE: f64 = 11.8;
    pub const Q: f64 = 0.3;
    pub const ACQ_N: usize = 5;
    pub const ACQ_TOL: f64 = 0.4;
    pub const RESET_T: f64 = 1.5;
    /// tracking: fused within this long
    pub const TRACK_T: f64 = 0.6;
    /// the autopilot uses the target while fused within USE_T (USE_LOW_T below USE_LOW_ALT, where
    /// the tag fills the image), otherwise it falls back to GPS
    pub const USE_T: f64 = 1.0;
    pub const USE_LOW_T: f64 = 4.0;
    pub const USE_LOW_ALT: f64 = 1.5;
    /// no detection for this long: back to the GPS prior
    pub const FORGET_T: f64 = 10.0;
}
use cam::*;

/// One detection as the camera reports it.
#[derive(Clone, Copy, Debug)]
pub struct Det {
    /// capture time
    pub tm: f64,
    pub u: f64,
    pub v: f64,
    /// tag size in the image, px (square root of its area)
    pub span: f64,
    /// tag top edge relative to the image up (nose), rad, + = tag heading greater than the nose
    pub yaw: f64,
    /// a false detection (for tests; the estimator does not know)
    pub fake: bool,
}

/// Camera sensor.
#[derive(Clone, Debug)]
pub struct Camera {
    /// test hook / failure: camera blocked or off
    pub enabled: bool,
    pub rng: Rng,
    clk: f64,
    q: VecDeque<(f64, Det)>,
    /// tag geometrically detectable at the last frame (before dropouts)
    pub in_view: bool,
    pub blur: f64,
    /// detections delivered (including false ones)
    pub n_det: u32,
    pub n_fake: u32,
    /// test hook: false detections at this ground offset (world x, z) from the tag, with this
    /// probability per frame
    pub fake_hook: Option<(f64, f64, f64)>,
}

impl Camera {
    pub fn new(seed: u64) -> Camera {
        Camera { enabled: true, rng: Rng::new(seed), clk: 0.0, q: VecDeque::new(), in_view: false, blur: 0.0, n_det: 0, n_fake: 0, fake_hook: None }
    }
    pub fn reset(&mut self, t: f64) {
        self.clk = t;
        self.q.clear();
        self.in_view = false;
        self.n_det = 0;
        self.n_fake = 0;
    }
}

/// Lens position and attitude of the camera.
fn lens(p: V3, q: Quat) -> V3 {
    p + MOUNT.rot(q)
}
/// Project a world point: (u, v, depth).
pub fn project(pw: V3, c: V3, q: Quat) -> Option<(f64, f64, f64)> {
    let d = (pw - c).rot(q.inv());
    let depth = -d.y;
    if depth < 0.05 {
        return None;
    }
    Some((W / 2.0 + F * d.x / depth, H / 2.0 + F * d.z / depth, depth))
}
fn inside(u: f64, v: f64) -> bool {
    (0.0..=W).contains(&u) && (0.0..=H).contains(&v)
}
/// Tag centre and the forward / right half-edge vectors on the ground.
fn tag_geom() -> (V3, V3, V3) {
    let s = TAG_SIZE / 2.0;
    let f = v3(-PAD.hdg.sin(), 0.0, -PAD.hdg.cos());
    let r = f.cross(Y);
    (v3(PAD.x, TAG_Y, PAD.z), f * s, r * s)
}

impl Camera {
    /// One camera frame from the true state. Returns detections now due (after the latency).
    pub fn step(&mut self, t: f64, h: &Heli) -> Vec<Det> {
        let mut out = vec![];
        if t >= self.clk {
            self.clk += RATE;
            if let Some(d) = self.capture(t, h) {
                self.q.push_back((t + LAT, d));
            }
        }
        while let Some(&(due, d)) = self.q.front() {
            if due > t {
                break;
            }
            self.q.pop_front();
            self.n_det += 1;
            if d.fake {
                self.n_fake += 1;
            }
            out.push(d);
        }
        out
    }

    fn capture(&mut self, t: f64, h: &Heli) -> Option<Det> {
        self.in_view = false;
        if !self.enabled || h.crashed {
            return None;
        }
        let q = h.q;
        let up = up_of(q);
        if up.y < (MAX_TILT * DEG).cos() {
            return None;
        }
        let c = lens(h.p, q);
        let wxz = h.w.x.hypot(h.w.z);
        let (tc, tf, tr) = tag_geom();
        let (uc, vc, _) = project(tc, c, q)?;
        let r_px = (uc - W / 2.0).hypot(vc - H / 2.0);
        let blur = (wxz * F + h.w.y.abs() * r_px) * EXPO;
        self.blur = blur;
        let rng = &mut self.rng;
        // rare false detection anywhere in the image, and the test hook's offset fakes
        let fake = if let Some((dx, dz, p)) = self.fake_hook {
            if rng.f() < p {
                project(tc + v3(dx, 0.0, dz), c, q).map(|(u, v, dep)| (u, v, TAG_SIZE * F / dep))
            } else {
                None
            }
        } else if rng.f() < FAKE_P && c.y > 0.5 {
            Some((rng.f() * W, rng.f() * H, MIN_PX + rng.f() * 48.0))
        } else {
            None
        };
        if let Some((u, v, span)) = fake {
            if inside(u, v) {
                return Some(Det { tm: t, u, v, span, yaw: rng.rnd1() * PI_, fake: true });
            }
        }
        if !inside(uc, vc) || blur > BLUR_MAX {
            return None;
        }
        // corners: span from the projected area; visible fraction from a 5 x 5 grid
        let mut pc = [(0.0, 0.0); 4];
        for (i, (a, b)) in [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)].iter().enumerate() {
            pc[i] = match project(tc + tf * *a + tr * *b, c, q) {
                Some((u, v, _)) => (u, v),
                None => return None,
            };
        }
        let mut area = 0.0;
        for i in 0..4 {
            let (a, b) = (pc[i], pc[(i + 1) % 4]);
            area += a.0 * b.1 - b.0 * a.1;
        }
        let span = (area.abs() / 2.0).sqrt();
        if span < MIN_PX || span > MAX_FRAC * H {
            return None;
        }
        let mut vis = 0;
        for i in 0..5 {
            for j in 0..5 {
                let (a, b) = (-1.0 + 0.5 * i as f64, -1.0 + 0.5 * j as f64);
                if let Some((u, v, _)) = project(tc + tf * a + tr * b, c, q) {
                    if inside(u, v) {
                        vis += 1;
                    }
                }
            }
        }
        if (vis as f64) < MIN_AREA * 25.0 {
            return None;
        }
        self.in_view = true;
        if rng.f() < DROP {
            return None;
        }
        let n = PIX_N + BLUR_N * blur;
        let yaw = wrap(PAD.hdg - heading_of(q) + rng.randn() * YAW_N * DEG);
        Some(Det { tm: t, u: uc + rng.randn() * n, v: vc + rng.randn() * n, span, yaw, fake: false })
    }
}
const PI_: f64 = core::f64::consts::PI;

/// Precision-landing target estimator.
#[derive(Clone, Debug)]
pub struct PrecLand {
    /// pad position in the estimate's world frame (y = 0)
    pub pad: V3,
    pub var: f64,
    /// pad heading in the estimate's frame, from the tag yaw
    pub hdg: Option<f64>,
    /// surveyed (GPS) pad position in the estimate's frame: moves with EKF position resets
    pub nom: V3,
    pub locked: bool,
    hist: VecDeque<(f64, V3, Quat)>,
    cand: VecDeque<(f64, V3)>,
    rej: VecDeque<(f64, V3)>,
    pub last_fused: Option<f64>,
    pub last_det: Option<(f64, Det)>,
    pub n_fused: u32,
    pub n_rej: u32,
    pub n_fake_fused: u32,
    pub resets: u32,
    /// last test ratio (NIS / gate)
    pub tr: f64,
    t: f64,
}

impl Default for PrecLand {
    fn default() -> Self {
        PrecLand::new()
    }
}

impl PrecLand {
    pub fn new() -> PrecLand {
        PrecLand {
            pad: v3(PAD.x, 0.0, PAD.z),
            var: 1.0,
            hdg: None,
            nom: v3(PAD.x, 0.0, PAD.z),
            locked: false,
            hist: VecDeque::new(),
            cand: VecDeque::new(),
            rej: VecDeque::new(),
            last_fused: None,
            last_det: None,
            n_fused: 0,
            n_rej: 0,
            n_fake_fused: 0,
            resets: 0,
            tr: 0.0,
            t: 0.0,
        }
    }

    /// Record the estimate (what the controllers see) for the delayed detections.
    pub fn push(&mut self, t: f64, p: V3, q: Quat) {
        self.t = t;
        self.hist.push_back((t, p, q));
        while self.hist.front().is_some_and(|h| h.0 < t - 0.3) {
            self.hist.pop_front();
        }
    }

    /// Predict: random walk of the target in the estimate's frame, capped at the prior; forget
    /// the camera after a long gap.
    pub fn tick(&mut self, t: f64, dt: f64, var0: f64) {
        self.t = t;
        if self.last_fused.map_or(true, |f| t - f > FORGET_T) {
            self.pad = self.nom;
            self.var = var0;
            self.hdg = None;
            self.locked = false;
            return;
        }
        self.var = (self.var + Q * Q * dt).min(var0.max(0.01));
    }

    /// Shift by an EKF position reset (the frame moved).
    pub fn shift(&mut self, dp: V3) {
        let d = v3(dp.x, 0.0, dp.z);
        self.pad += d;
        self.nom += d;
        for h in self.hist.iter_mut() {
            h.1 += dp;
        }
        for c in self.cand.iter_mut() {
            c.1 += d;
        }
        self.rej.clear();
    }

    fn at(&self, tm: f64) -> Option<(V3, Quat)> {
        let mut best: Option<(f64, V3, Quat)> = None;
        for h in self.hist.iter() {
            if best.map_or(true, |b| (h.0 - tm).abs() < (b.0 - tm).abs()) {
                best = Some(*h);
            }
        }
        best.map(|b| (b.1, b.2))
    }

    /// Ground point of a detection from the estimate at the capture time, and its sigma.
    pub fn ground(&self, d: &Det, sig_tilt: f64, sig_hgt: f64, w: V3) -> Option<(V3, f64, Quat)> {
        let (p, q) = self.at(d.tm)?;
        let c = lens(p, q);
        let ray = v3((d.u - W / 2.0) / F, -1.0, (d.v - H / 2.0) / F).rot(q);
        let hgt = c.y - TAG_Y;
        if ray.y > -0.3 || hgt < 0.1 {
            return None;
        }
        let s = hgt / -ray.y;
        let g = c + ray * s;
        let slant = ray.length() * s;
        let off = (ray.x.hypot(ray.z)) / -ray.y;
        let blur = w.x.hypot(w.z) * F * EXPO;
        let sd = slant * ((PIX_N + BLUR_N * blur) / F + sig_tilt) + sig_hgt * off + 0.02;
        Some((v3(g.x, 0.0, g.z), sd, q))
    }

    /// Fuse one detection.
    pub fn update(&mut self, d: Det, t: f64, sig_tilt: f64, sig_hgt: f64, w: V3) {
        self.last_det = Some((t, d));
        let Some((z, sd, q)) = self.ground(&d, sig_tilt, sig_hgt, w) else { return };
        let r = sd * sd;
        if !self.locked {
            // acquisition: the last ACQ_N detections agree, and agree with the prior
            self.cand.push_back((t, z));
            while self.cand.len() > ACQ_N || self.cand.front().is_some_and(|c| c.0 < t - 0.6) {
                self.cand.pop_front();
            }
            if self.cand.len() < ACQ_N {
                return;
            }
            let n = self.cand.len() as f64;
            let m = self.cand.iter().fold(V3::ZERO, |a, c| a + c.1) * (1.0 / n);
            if self.cand.iter().any(|c| (c.1 - m).length() > ACQ_TOL) {
                return;
            }
            let y = m - self.pad;
            self.tr = y.length_sq() / (self.var + r / n) / GATE;
            if self.tr > 1.0 {
                self.n_rej += 1;
                return;
            }
            self.pad = m;
            self.var = r / n;
            self.locked = true;
            self.cand.clear();
            self.fused(t, &d, q);
            return;
        }
        let y = z - self.pad;
        self.tr = y.length_sq() / (self.var + r) / GATE;
        if self.tr > 1.0 {
            self.n_rej += 1;
            self.rej.push_back((t, z));
            while self.rej.front().is_some_and(|c| c.0 < t - RESET_T - 0.2) {
                self.rej.pop_front();
            }
            // everything rejected for RESET_T with a consistent offset: the target really moved
            let fused_recent = self.last_fused.is_some_and(|f| t - f < RESET_T);
            if !fused_recent && self.rej.len() >= 8 {
                let n = self.rej.len() as f64;
                let m = self.rej.iter().fold(V3::ZERO, |a, c| a + c.1) * (1.0 / n);
                if self.rej.iter().all(|c| (c.1 - m).length() < 0.3) {
                    self.pad = m;
                    self.var = 0.3 * 0.3;
                    self.resets += 1;
                    self.rej.clear();
                    self.fused(t, &d, q);
                }
            }
            return;
        }
        let k = self.var / (self.var + r);
        self.pad += y * k;
        self.var *= 1.0 - k;
        self.rej.clear();
        self.fused(t, &d, q);
    }

    fn fused(&mut self, t: f64, d: &Det, q: Quat) {
        self.last_fused = Some(t);
        self.n_fused += 1;
        if d.fake {
            self.n_fake_fused += 1;
        }
        let m = wrap(heading_of(q) + d.yaw);
        self.hdg = Some(match self.hdg {
            None => m,
            Some(h) => {
                let e = wrap(m - h);
                if e.abs() < 20.0 * DEG {
                    wrap(h + 0.3 * e)
                } else {
                    h
                }
            }
        });
    }

    pub fn tracking(&self) -> bool {
        self.locked && self.last_fused.is_some_and(|f| self.t - f < TRACK_T)
    }
    /// Target the autopilot may fly to (None: fall back to GPS).
    pub fn usable(&self, alt: f64) -> bool {
        self.locked
            && self.last_fused.is_some_and(|f| {
                let a = self.t - f;
                a < USE_T || (alt < USE_LOW_ALT && a < USE_LOW_T)
            })
    }
    /// SEARCHING / TRACKING / LOST
    pub fn status(&self, enabled: bool) -> &'static str {
        if !enabled {
            "OFF"
        } else if self.tracking() {
            "TRACKING"
        } else if self.locked && self.last_fused.is_some_and(|f| self.t - f < 5.0) {
            "LOST"
        } else {
            "SEARCHING"
        }
    }
}
