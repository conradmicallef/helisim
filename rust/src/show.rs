//! SHOW: an F3C-style precision competition routine with simulated judges.
//!
//! Real FAI F3C schedules change between seasons; this is not an official schedule. It models
//! the structure: a hovering section and an aerobatic section, each manoeuvre with a K-factor,
//! flown in a flight box in front of the judges and marked 0..10 by five judges (`judge`).
//!
//! Every manoeuvre has an ideal reference (`Track`): position, velocity, acceleration, attitude
//! (body up and nose), body rate and timing, sampled at 50 Hz. The autopilot demo flies it with a
//! reference-tracking thrust-vector controller (`track_cmd`); the judges compare the true flight
//! against it; the page draws it as the ghost path. The autorotation is flown by the existing
//! AUTOROTATE autopilot (throttle hold), its reference is only drawn and its judging is geometric.
//!
//! Flow per manoeuvre: Position (autopilot to the entry, settle) -> Ready (manual: hand-over, wait
//! for the start) -> Fly (recording; demo: tracking the reference) -> Judged (result card) -> next.
//! A rescue by crash protection scores the manoeuvre 0 and the show continues after the recovery.

use crate::autopilot::{land, Auto, Mode};
use crate::heli::*;
use crate::judge::{self, Assess, Sample, Touch};
use crate::jsonw::to_fixed;
use crate::math::*;
use crate::rng::Rng;
use crate::sim::{Cmd, Sim};
use crate::units::{DEG, G, PI};

/// Flight box and manoeuvre geometry (world: x to the pilot's right, z toward the pilot, the pad at
/// the origin, the pilot and judges at z = +9, the flight-line rope at z = +6.5). Heights are
/// skid heights (`alt`).
pub mod geo {
    /// hovering centre: the start circle on the hovering line, 10 m beyond the rope
    pub const HX: f64 = 0.0;
    pub const HZ: f64 = -3.5;
    /// hovering height
    pub const HA: f64 = 1.75;
    /// aerobatic line 20.5 m beyond the rope, centre marker at AX, box edges at AX +- EDGE
    pub const AX: f64 = 0.0;
    pub const AZ: f64 = -14.0;
    pub const EDGE: f64 = 25.0;
    /// run-up start, from the centre (clear of the trees at r >= 45 m around (0, -20))
    pub const RUN: f64 = 40.0;
    /// run-up / run-out acceleration time, s
    pub const T_ACC: f64 = 3.5;
    pub const SQ: f64 = 4.0;
    pub const TRI_W: f64 = 3.0;
    pub const TRI_H: f64 = 3.0;
    pub const PIR_H0: f64 = 5.0;
    pub const PIR_H1: f64 = 2.0;
    pub const PIR_T: f64 = 8.0;
    pub const EIGHT_R: f64 = 2.0;
    pub const EIGHT_V: f64 = 1.4;
    pub const LOOP_H: f64 = 12.0;
    pub const LOOP_R: f64 = 7.0;
    pub const LOOP_V: f64 = 8.5;
    pub const ROLL_H: f64 = 15.0;
    pub const ROLL_V: f64 = 8.0;
    pub const ROLL_W: f64 = 4.0;
    pub const ST_H: f64 = 13.0;
    pub const ST_R: f64 = 6.0;
    pub const ST_V: f64 = 8.0;
    /// stall turn: the pivot starts at this climb speed and takes ST_PIV s
    pub const ST_VP: f64 = 3.0;
    pub const ST_PIV: f64 = 0.8;
    pub const RC_H: f64 = 15.0;
    pub const RC_R: f64 = 9.0;
    pub const RC_V: f64 = 7.0;
    pub const RC_W: f64 = 4.0;
    /// autorotation entry (the AUTOROTATE autopilot's entry point: 38 m upwind of the pad at 25 m)
    pub const AR_X: f64 = 38.0;
    pub const AR_H: f64 = 25.0;
}

/// Reference sample period, s.
pub const TS: f64 = 0.02;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Man {
    Square = 0,
    Triangle = 1,
    DescPir = 2,
    Eight = 3,
    Land = 4,
    Loop = 5,
    Roll = 6,
    StallTurn = 7,
    RollCircle = 8,
    Autorot = 9,
}
pub const SCHED: [Man; 10] = [
    Man::Square,
    Man::Triangle,
    Man::DescPir,
    Man::Eight,
    Man::Land,
    Man::Loop,
    Man::Roll,
    Man::StallTurn,
    Man::RollCircle,
    Man::Autorot,
];

impl Man {
    pub fn name(self) -> &'static str {
        match self {
            Man::Square => "Hover square",
            Man::Triangle => "Hover triangle",
            Man::DescPir => "Descending pirouette",
            Man::Eight => "Nose-in figure eight",
            Man::Land => "Stop and landing",
            Man::Loop => "Loop",
            Man::Roll => "Roll",
            Man::StallTurn => "Stall turn",
            Man::RollCircle => "Rolling circle",
            Man::Autorot => "Autorotation",
        }
    }
    /// K-factor (difficulty weight).
    pub fn k(self) -> f64 {
        match self {
            Man::Square | Man::Land => 1.0,
            Man::Triangle | Man::DescPir | Man::Loop | Man::Roll => 1.5,
            Man::Eight | Man::StallTurn | Man::Autorot => 2.0,
            Man::RollCircle => 2.5,
        }
    }
    pub fn hover(self) -> bool {
        (self as u8) <= Man::Land as u8
    }
    /// One-line instruction for the status line.
    pub fn instr(self) -> &'static str {
        match self {
            Man::Square => "4 m square beyond the start circle, stop at each corner, tail in, 1.8 m high.",
            Man::Triangle => "climb diagonally right, across at 4.8 m, descend diagonally back to the circle; stop at the corners.",
            Man::DescPir => "one slow 360° turn while descending from 5 m to 2 m over the circle.",
            Man::Eight => "nose toward you, two 2 m circles through the start circle, 1.8 m high.",
            Man::Land => "back over the pad, stop, then straight down to a soft touchdown on the centre.",
            Man::Loop => "along the line at 12 m, one round 7 m loop at the centre flag.",
            Man::Roll => "along the line at 15 m, one axial roll at the centre flag.",
            Man::StallTurn => "pull up vertical at the centre flag, turn 180° at the top, dive and pull out back along the line.",
            Man::RollCircle => "a level 360° circle away from you at 15 m with one full roll at the far point.",
            Man::Autorot => "from 25 m: throttle hold, glide to the pad, flare and touch down.",
        }
    }
}

/// Which part of the schedule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    Full = 0,
    Hover = 1,
    Aero = 2,
}
impl Part {
    pub fn name(self) -> &'static str {
        match self {
            Part::Full => "Full schedule",
            Part::Hover => "Hovering only",
            Part::Aero => "Aerobatics only",
        }
    }
    pub fn list(self) -> Vec<Man> {
        SCHED.iter().copied().filter(|m| self == Part::Full || (self == Part::Hover) == m.hover()).collect()
    }
}

/// One reference sample. `p` has y = skid height.
#[derive(Clone, Copy, Debug)]
pub struct RefPt {
    pub p: V3,
    pub v: V3,
    pub a: V3,
    /// body up and nose (world)
    pub up: V3,
    pub nose: V3,
    /// world angular velocity of the reference attitude, rad/s
    pub w: V3,
    /// attitude weight: 0 = attitude from the thrust vector the path needs, 1 = prescribed (rolls)
    pub att: f64,
    /// sign hint for the thrust direction (upright: Y; loops: toward the centre)
    pub uph: V3,
    pub leg: u8,
}

/// The reference of one manoeuvre.
#[derive(Clone, Debug)]
pub struct Track {
    pub m: Man,
    pub pts: Vec<RefPt>,
    pub dur: f64,
    /// figure centre (hover: of the figure; aerobatics: the box centre), y = 0
    pub centre: V3,
    /// nominal size: square side, triangle base, eight / loop / circle radius, stall-turn pull-up radius
    pub size: f64,
    /// stop points (hover square / triangle corners, then the start circle)
    pub stops: Vec<V3>,
    /// entry: position (y = skid height) and heading
    pub entry: V3,
    pub hdg: f64,
}

/// Test hooks: deliberately sloppy flights (the flown reference is changed, the judges compare
/// against the nominal one).
#[derive(Clone, Copy, Debug)]
pub struct ShowHook {
    /// shift every figure (off-centre box), m
    pub off: V3,
    /// hover heading error, rad
    pub hdg: f64,
    /// size factor for every figure (oversized loop)
    pub scale: f64,
    /// landing descent rate all the way to the ground, m/s (0 = nominal)
    pub land_vs: f64,
    /// sideways sway amplitude on the hover legs, m (position error)
    pub sway: f64,
    /// in manoeuvre (index into SCHED) from t s into it, force the collective (a broken flight: rescue)
    pub cut: Option<(u8, f64, f64)>,
}
impl Default for ShowHook {
    fn default() -> Self {
        ShowHook { off: V3::ZERO, hdg: 0.0, scale: 1.0, land_vs: 0.0, sway: 0.0, cut: None }
    }
}

// ------------------------------------------------------------------ reference builder
#[derive(Clone, Copy)]
struct Pose {
    p: V3,
    nose: V3,
    uph: V3,
    roll: f64,
    att: f64,
    leg: u8,
}
fn pose(p: V3, nose: V3, leg: u8) -> Pose {
    Pose { p, nose, uph: Y, roll: 0.0, att: 0.0, leg }
}
fn hdir(h: f64) -> V3 {
    v3(-h.sin(), 0.0, -h.cos())
}
/// s-curve 0..1
fn ease(u: f64) -> f64 {
    let u = clamp(u, 0.0, 1.0);
    0.5 * (1.0 - (PI * u).cos())
}
/// Sine-ramped angle profile (same as the stunts): angle at time t for total `a` at peak rate `w`.
fn ramp_ang(a: f64, w: f64, tr: f64, t: f64) -> f64 {
    crate::stunt::ramp(a, w, tr, t).0
}

/// Drag the airframe needs to fly against at a speed (per unit mass, along the velocity).
fn drag_k(v: f64) -> f64 {
    0.22 + 0.035 * v
}

/// The body up for a needed specific force `f`, a sign hint and an attitude weight: the thrust
/// direction where the force is large, the hint where it is small (|f| < 3 m/s2), the hint when
/// prescribed. Shared by the reference builder and the controller.
pub fn up_for(f: V3, uph: V3, att: f64) -> V3 {
    let fm = f.length();
    let s = if f.dot(uph) < -0.3 * fm { -1.0 } else { 1.0 };
    let c = (3.0 - fm).max(0.0);
    let mut u = f * s + uph * c;
    if u.length_sq() < 1e-9 {
        u = uph;
    }
    let u = u.normalize();
    (u * (1.0 - att) + uph * att).normalize()
}

struct B {
    ps: Vec<Pose>,
}
impl B {
    fn new() -> B {
        B { ps: Vec::new() }
    }
    /// Append samples of `f` over `dur` s (t = 0, TS, ... < dur).
    #[inline(never)]
    fn seg(&mut self, dur: f64, f: &dyn Fn(f64) -> Pose) {
        let n = (dur / TS).round().max(1.0) as usize;
        for i in 0..n {
            self.ps.push(f(i as f64 * TS));
        }
    }
    fn last(&self) -> Pose {
        *self.ps.last().unwrap()
    }
    fn hold(&mut self, dur: f64, leg: u8) {
        let mut l = self.last();
        l.leg = leg;
        self.seg(dur, &move |_| l);
    }
    /// Straight move with an s-curve from A to B in `dur` (hover legs), nose fixed.
    fn line(&mut self, a: V3, b: V3, dur: f64, nose: V3, leg: u8) {
        self.seg(dur, &move |t| pose(a + (b - a) * ease(t / dur), nose, leg));
    }
    /// Straight run along `d` from `a`: speed from v0 to v1 with an s-curve in `dur`.
    fn run(&mut self, a: V3, d: V3, v0: f64, v1: f64, dur: f64, leg: u8) -> V3 {
        let s = move |t: f64| v0 * t + (v1 - v0) * (t / 2.0 - dur / (2.0 * PI) * (PI * t / dur).sin());
        self.seg(dur, &move |t| pose(a + d * s(t), d, leg));
        a + d * s(dur)
    }
    /// Arc in the vertical plane of the line `dir` (x): centre `c`, radius `r`, from angle th0 to th1
    /// (0 = bottom moving +dir), speed law `v' = 2 (vt - v)` (at most 3 m/s2) within the
    /// tangential force the disc can give next to the normal force (1.9 g in all). Returns the end speed.
    #[allow(clippy::too_many_arguments)]
    fn varc(&mut self, c: V3, dir: f64, r: f64, th0: f64, th1: f64, v0: f64, vt: f64, leg: u8) -> f64 {
        let (mut th, mut v, mut tacc) = (th0, v0, 0.0);
        let h = 0.002;
        let at = |th: f64| {
            let p = c + v3(r * th.sin() * dir, -r * th.cos(), 0.0);
            let tan = v3(th.cos() * dir, th.sin(), 0.0);
            let inw = v3(-th.sin() * dir, th.cos(), 0.0);
            (p, tan, inw)
        };
        while th < th1 {
            if tacc <= 1e-9 {
                let (p, tan, inw) = at(th);
                self.ps.push(Pose { p, nose: tan, uph: inw, roll: 0.0, att: 0.0, leg });
                tacc += TS;
            }
            let fn_ = v * v / r + G * th.cos();
            let tmax = 1.9 * G;
            let lim = (tmax * tmax - fn_ * fn_).max(0.0).sqrt();
            let base = G * th.sin() + drag_k(v) * v;
            let sdd = clamp(clamp(2.0 * (vt - v), -3.0, 3.0), -lim - base, lim - base);
            v = (v + sdd * h).max(0.5);
            th += v / r * h;
            tacc -= h;
        }
        v
    }
    /// Vertical ballistic line from `p` with vertical speed `vy` (drag opposes it) until `stop`
    /// says so; the attitude from `pf(t)`. Returns end position and vertical speed.
    fn vline(&mut self, p: V3, vy: f64, stop: impl Fn(V3, f64, f64) -> bool, pf: impl Fn(f64) -> (V3, V3, f64), leg: u8) -> (V3, f64) {
        let (mut p, mut vy, mut t, mut tacc) = (p, vy, 0.0, 0.0);
        let h = 0.002;
        loop {
            if tacc <= 1e-9 {
                if stop(p, vy, t) {
                    break;
                }
                let (nose, uph, att) = pf(t);
                self.ps.push(Pose { p, nose, uph, roll: 0.0, att, leg });
                tacc += TS;
            }
            let a = -G - drag_k(vy.abs()) * vy;
            vy += a * h;
            p.y += vy * h;
            t += h;
            tacc -= h;
            if t > 30.0 {
                break;
            }
        }
        (p, vy)
    }
}

/// Finish a reference: velocity and acceleration by central differences, attitude from the force
/// the path needs (or prescribed), the roll about the body nose, angular velocity from the attitude.
#[allow(clippy::needless_range_loop)]
fn finish(m: Man, ps: Vec<Pose>, centre: V3, size: f64, stops: Vec<V3>, entry: V3, hdg: f64) -> Track {
    let n = ps.len();
    let p = |i: usize| ps[i.min(n - 1)].p;
    let mut v = vec![V3::ZERO; n];
    for i in 0..n {
        let (a, b) = (if i == 0 { 0 } else { i - 1 }, (i + 1).min(n - 1));
        if b > a {
            v[i] = (p(b) - p(a)) * (1.0 / ((b - a) as f64 * TS));
        }
    }
    let mut acc = vec![V3::ZERO; n];
    for i in 0..n {
        let (a, b) = (if i == 0 { 0 } else { i - 1 }, (i + 1).min(n - 1));
        if b > a {
            acc[i] = (v[b] - v[a]) * (1.0 / ((b - a) as f64 * TS));
        }
    }
    let mut pts = Vec::with_capacity(n);
    let mut qs = Vec::with_capacity(n);
    for i in 0..n {
        let s = ps[i];
        let f = acc[i] + Y * G + v[i] * drag_k(v[i].length());
        // rolls: about the horizontal nose from 10 deg nose down (keeps some forward drive)
        let up0 = if s.roll != 0.0 { (Y * (10.0 * DEG).cos() + s.nose * (10.0 * DEG).sin()).normalize() } else { up_for(f, s.uph, s.att) };
        let mut nb = s.nose - up0 * s.nose.dot(up0);
        if nb.length_sq() < 1e-8 {
            nb = s.nose;
        }
        let nb = nb.normalize();
        let up = if s.roll != 0.0 { up0 * s.roll.cos() + nb.cross(up0) * s.roll.sin() } else { up0 };
        let att = if s.roll != 0.0 { 1.0 } else { s.att };
        let uph = if s.roll != 0.0 { up } else { s.uph };
        qs.push(quat_up_nose(up, nb));
        pts.push(RefPt { p: s.p, v: v[i], a: acc[i], up, nose: nb, w: V3::ZERO, att, uph, leg: s.leg });
    }
    for i in 0..n {
        let (a, b) = (if i == 0 { 0 } else { i - 1 }, (i + 1).min(n - 1));
        if b > a {
            let mut dq = qs[b] * qs[a].inv();
            if dq.w < 0.0 {
                dq = Quat { x: -dq.x, y: -dq.y, z: -dq.z, w: -dq.w };
            }
            pts[i].w = rotvec(dq) * (1.0 / ((b - a) as f64 * TS));
        }
    }
    Track { m, dur: n as f64 * TS, pts, centre, size, stops, entry, hdg }
}

/// The reference for a manoeuvre (`hook` default = nominal).
pub fn build(m: Man, hook: &ShowHook) -> Track {
    use geo::*;
    let o = v3(hook.off.x, 0.0, hook.off.z);
    let sc = hook.scale;
    let s = v3(HX, HA, HZ) + o;
    let h0 = hook.hdg;
    let n0 = hdir(h0);
    let mut b = B::new();
    match m {
        Man::Square => {
            let l = SQ * sc;
            let a = s + v3(l / 2.0, 0.0, 0.0);
            let bb = a + v3(0.0, 0.0, -l);
            let c = bb + v3(-l, 0.0, 0.0);
            let d = c + v3(0.0, 0.0, l);
            let sw = hook.sway;
            b.seg(1.0, &|_| pose(s, n0, 0));
            let tl = 4.5 * l.sqrt() / 2.0;
            for (i, (p0, p1, dur)) in [(s, a, tl * 0.7), (a, bb, tl), (bb, c, tl), (c, d, tl), (d, s, tl * 0.7)].into_iter().enumerate() {
                let leg = i as u8 + 1;
                b.seg(dur, &move |t| {
                    let u = t / dur;
                    let dd = (p1 - p0).normalize();
                    let side = v3(-dd.z, 0.0, dd.x);
                    pose(p0 + (p1 - p0) * ease(u) + side * (sw * (2.0 * PI * u).sin()), n0, leg)
                });
                b.hold(1.5, 0);
            }
            let st = vec![a, bb, c, d, s];
            finish(m, b.ps, s + v3(0.0, -HA, -l / 2.0), l, st, s, h0)
        }
        Man::Triangle => {
            let (w, h) = (TRI_W * sc, TRI_H * sc);
            let a = s + v3(w, h, 0.0);
            let bb = s + v3(-w, h, 0.0);
            b.seg(1.0, &|_| pose(s, n0, 0));
            b.line(s, a, 5.0, n0, 1);
            b.hold(1.5, 0);
            b.line(a, bb, 6.0, n0, 2);
            b.hold(1.5, 0);
            b.line(bb, s, 5.0, n0, 3);
            b.hold(1.5, 0);
            finish(m, b.ps, v3(s.x, 0.0, s.z), 2.0 * w, vec![a, bb, s], s, h0)
        }
        Man::DescPir => {
            let p0 = v3(s.x, PIR_H0, s.z);
            b.seg(1.0, &|_| pose(p0, n0, 0));
            b.seg(PIR_T, &|t| {
                let tr = 0.6;
                let w = 2.0 * PI / (PIR_T - tr);
                let ang = ramp_ang(2.0 * PI, w, tr, t);
                let y = PIR_H0 + (PIR_H1 - PIR_H0) * ang / (2.0 * PI);
                pose(v3(s.x, y, s.z), hdir(h0 + ang), 1)
            });
            let p1 = v3(s.x, PIR_H1, s.z);
            b.seg(1.0, &|_| pose(p1, n0, 0));
            finish(m, b.ps, v3(s.x, 0.0, s.z), 0.0, vec![p1], p0, h0)
        }
        Man::Eight => {
            let r = EIGHT_R * sc;
            let hn = PI + h0;
            let nn = hdir(hn);
            let len = 4.0 * PI * r;
            let tr = 2.0;
            let dur = len / EIGHT_V + tr;
            b.seg(1.0, &|_| pose(s, nn, 0));
            b.seg(dur, &|t| {
                let d = ramp_ang(len, EIGHT_V, tr, t);
                let ph = d / r; // 0..4 pi
                let p = if ph < 2.0 * PI {
                    s + v3(r - r * ph.cos(), 0.0, -r * ph.sin())
                } else {
                    let q = ph - 2.0 * PI;
                    s + v3(-r + r * q.cos(), 0.0, -r * q.sin())
                };
                pose(p, nn, if ph < 2.0 * PI { 1 } else { 2 })
            });
            b.hold(1.0, 0);
            finish(m, b.ps, v3(s.x, 0.0, s.z), r, vec![s], s, hn)
        }
        Man::Land => {
            let pad = v3(PAD.x, HA, PAD.z) + o;
            b.seg(1.0, &|_| pose(s, n0, 0));
            b.line(s, pad, 4.0, n0, 1);
            b.hold(2.0, 0);
            // vertical descent: 0.6 m/s, easing to 0.3 m/s near the ground (or a hard one)
            let mut y = HA;
            let mut pts = Vec::new();
            while y > -0.3 {
                pts.push(y);
                let vs = if hook.land_vs > 0.0 { hook.land_vs } else { clamp(0.3 + 0.35 * y, 0.3, 0.6) };
                y -= vs * TS;
            }
            for yy in pts {
                b.ps.push(pose(v3(pad.x, yy, pad.z), n0, 2));
            }
            finish(m, b.ps, v3(pad.x, 0.0, pad.z), 0.0, vec![v3(pad.x, 0.0, pad.z)], s, h0)
        }
        Man::Loop | Man::Roll | Man::StallTurn | Man::RollCircle => aero(m, hook),
        Man::Autorot => {
            // drawn only: the AUTOROTATE autopilot flies it (run-in, cut, glide, flare, cushion)
            let e = v3(AR_X, AR_H, 0.0) + o;
            let d = v3(-1.0, 0.0, 0.0);
            let cut = v3(23.0, AR_H, 0.0) + o;
            let fl = v3(7.0, 6.0, 0.0) + o;
            let pad = v3(PAD.x, 0.0, PAD.z) + o;
            b.line(e, cut, 2.2, d, 1);
            let gl = (fl - cut).length() / 8.5;
            b.seg(gl, &|t| pose(cut + (fl - cut) * (t / gl), d, 2));
            b.seg(2.5, &|t| {
                let u = t / 2.5;
                let x = fl.x + (pad.x - fl.x) * (1.0 - (1.0 - u) * (1.0 - u));
                let y = fl.y * (1.0 - u) * (1.0 - u) * (1.0 + 0.5 * u);
                pose(v3(x, y.max(0.0), fl.z), d, 3)
            });
            b.hold(0.5, 0);
            finish(m, b.ps, v3(PAD.x, 0.0, PAD.z), 0.0, vec![pad], e, PI / 2.0)
        }
    }
}

/// Aerobatic references: run-up from rest outside the box, the figure centred on the box, the
/// exit line out of the box and a stop. Direction: loop +x, roll -x, stall turn +x (exits -x),
/// rolling circle +x: each starts where the previous one stopped.
fn aero(m: Man, hook: &ShowHook) -> Track {
    use geo::*;
    let o = v3(hook.off.x, 0.0, hook.off.z);
    let sc = hook.scale;
    let dir = if m == Man::Roll { -1.0 } else { 1.0 };
    let dx = v3(dir, 0.0, 0.0);
    let cx = AX + o.x;
    let z = AZ + o.z;
    let (hgt, vv) = match m {
        Man::Loop => (LOOP_H, LOOP_V),
        Man::Roll => (ROLL_H, ROLL_V),
        Man::StallTurn => (ST_H, ST_V),
        _ => (RC_H, RC_V),
    };
    let start = v3(cx - dir * RUN, hgt, z);
    let mut b = B::new();
    b.seg(1.0, &|_| pose(start, dx, 0));
    let p = b.run(start, dx, 0.0, vv, T_ACC, 1);
    let edge_out = |b: &mut B, from: V3, d: f64, v: f64| {
        // exit line to the box edge on the side of travel, then a stop
        let dd = v3(d, 0.0, 0.0);
        let x_edge = cx + d * EDGE;
        let len = ((x_edge - from.x) * d).max(0.0);
        let t = len / v;
        b.seg(t, &|tt| pose(from + dd * (v * tt), dd, 3));
        let p = from + dd * len;
        let p = b.run(p, dd, v, 0.0, T_ACC, 4);
        let mut l = b.last();
        l.p = p;
        l.leg = 0;
        b.seg(1.5, &move |_| l);
    };
    let to = |b: &mut B, from: V3, x: f64, v: f64| -> V3 {
        let len = ((x - from.x) * dir).max(0.0);
        let t = len / v;
        b.seg(t, &|tt| pose(from + dx * (v * tt), dx, 1));
        from + dx * len
    };
    match m {
        Man::Loop => {
            let r = LOOP_R * sc;
            let p = to(&mut b, p, cx, vv);
            let v1 = b.varc(v3(p.x, hgt + r, z), dir, r, 0.0, 2.0 * PI, vv, vv, 2);
            edge_out(&mut b, v3(p.x, hgt, z), dir, v1.max(4.0));
            let entry = start;
            finish(m, b.ps, v3(cx, 0.0, z), r, vec![], entry, -dir * PI / 2.0)
        }
        Man::Roll => {
            // axial roll on the line: prescribed attitude, the path straight and level (the
            // controller holds height with the vertical thrust it has, like the stunt roll)
            let tr = 0.25;
            let troll = 2.0 * PI / ROLL_W + tr;
            let p = to(&mut b, p, cx - dir * vv * troll / 2.0, vv);
            b.seg(troll, &|t| {
                let mut ps = pose(p + dx * (vv * t), dx, 2);
                ps.roll = -crate::stunt::ramp(2.0 * PI, ROLL_W, tr, t).0 - 1e-9;
                ps
            });
            let p = p + dx * (vv * troll);
            edge_out(&mut b, p, dir, vv);
            finish(m, b.ps, v3(cx, 0.0, z), 0.0, vec![], start, -dir * PI / 2.0)
        }
        Man::StallTurn => {
            let r = ST_R * sc;
            let x0 = cx - dir * r;
            let p = to(&mut b, p, x0, vv);
            let c = v3(x0, hgt + r, z);
            let va = b.varc(c, dir, r, 0.0, PI / 2.0, vv, vv, 2);
            let x1 = x0 + dir * r;
            let up_ax = v3(-dir, 0.0, 0.0);
            let side = up_ax.cross(Y);
            // vertical up until the pivot speed, pivot (ballistic, the nose swings over), down
            let (pt, vt) = b.vline(v3(x1, hgt + r, z), va, |_, vy, _| vy <= ST_VP, |_| (Y, up_ax, 0.0), 2);
            let (pd, vd) = b.vline(
                pt,
                vt,
                |_, _, t| t >= ST_PIV,
                move |t| {
                    let ph = PI * ease(t / ST_PIV);
                    (Y * ph.cos() + side * ph.sin(), up_ax, 1.0)
                },
                2,
            );
            let (_pe, ve) = b.vline(pd, vd, |p, vy, _| p.y <= hgt + r && vy < 0.0, |_| (-Y, up_ax, 0.0), 2);
            let v2 = b.varc(c, -dir, r, 1.5 * PI, 2.0 * PI, ve.abs(), vv, 2);
            let _ = p;
            edge_out(&mut b, v3(x0, hgt, z), -dir, v2.max(4.0));
            finish(m, b.ps, v3(cx, 0.0, z), r, vec![], start, -dir * PI / 2.0)
        }
        _ => {
            // rolling circle (simplified): a level 360 deg turn away from the judges with one full
            // axial roll at the far point. The roll is flown wings level on a short straight chord
            // across the far side (a heli disc can't hold the turn while rolling), so the figure
            // is two half circles joined by the roll chord, the chord centred on the box. A real
            // rolling circle blends the rolls into the turn all the way round.
            let tr = 0.25;
            let r = RC_R * sc;
            let troll = 2.0 * PI / RC_W + tr;
            let lc = vv * troll;
            let p = to(&mut b, p, cx + dir * lc / 2.0, vv);
            let half = |b: &mut B, c: V3, a0: f64, leg: u8| {
                let ta = PI * r / vv;
                b.seg(ta, &move |t| {
                    let a = a0 + t * vv / r;
                    pose(c + v3(dir * r * a.sin(), 0.0, r * a.cos()), v3(dir * a.cos(), 0.0, -a.sin()), leg)
                });
            };
            let c1 = v3(p.x, hgt, z - r);
            half(&mut b, c1, 0.0, 2);
            let far = c1 + v3(0.0, 0.0, -r);
            b.seg(troll, &|t| {
                let mut ps = pose(far - dx * (vv * t), -dx, 5);
                ps.roll = -crate::stunt::ramp(2.0 * PI, RC_W, tr, t).0 - 1e-9;
                ps
            });
            let c2 = v3(p.x - dir * lc, hgt, z - r);
            half(&mut b, c2, PI, 6);
            edge_out(&mut b, v3(c2.x, hgt, z), dir, vv);
            finish(m, b.ps, v3(cx, 0.0, z), r, vec![], start, -dir * PI / 2.0)
        }
    }
}

impl Track {
    /// Reference at time t (clamped to the track), linear interpolation.
    pub fn at(&self, t: f64) -> RefPt {
        let n = self.pts.len();
        let x = clamp(t / TS, 0.0, (n - 1) as f64);
        let i = x.floor() as usize;
        let j = (i + 1).min(n - 1);
        let u = x - i as f64;
        let (a, b) = (&self.pts[i], &self.pts[j]);
        let l = |p: V3, q: V3| p + (q - p) * u;
        let mut r = *a;
        r.p = l(a.p, b.p);
        r.v = l(a.v, b.v);
        r.a = l(a.a, b.a);
        r.up = l(a.up, b.up).normalize();
        r.nose = l(a.nose, b.nose);
        if r.nose.length_sq() > 1e-9 {
            r.nose = r.nose.normalize();
        }
        r.uph = l(a.uph, b.uph);
        if r.uph.length_sq() > 1e-9 {
            r.uph = r.uph.normalize();
        }
        r.w = l(a.w, b.w);
        r.att = a.att + (b.att - a.att) * u;
        if t > self.dur {
            r.v = V3::ZERO;
            r.a = V3::ZERO;
            r.w = V3::ZERO;
        }
        r
    }
    /// Ghost polyline for the page: every 0.2 s, body-origin heights.
    pub fn ghost(&self) -> Vec<V3> {
        self.pts.iter().step_by(10).chain(core::iter::once(self.pts.last().unwrap())).map(|r| v3(r.p.x, r.p.y + REST_Y, r.p.z)).collect()
    }
}

// ------------------------------------------------------------------ show state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ph {
    /// autopilot (or the pilot) to the entry, settling
    Position = 0,
    /// manual: handed over, waiting for the start
    Ready = 1,
    /// the manoeuvre: recording (demo: flying the reference)
    Fly = 2,
    /// result card, then the next one
    Judged = 3,
}

/// Judged manoeuvre.
#[derive(Clone, Debug)]
pub struct ManRes {
    pub m: Man,
    pub k: f64,
    pub base: f64,
    pub marks: [f64; judge::rules::JUDGES],
    pub hi: usize,
    pub lo: usize,
    /// mean of the three kept marks, and times K
    pub score: f64,
    pub ks: f64,
    pub why: Vec<String>,
    pub zero: Option<String>,
    /// sim time judged (the page shows the card for a few seconds)
    pub t: f64,
}

/// A finished (or stopped) show.
#[derive(Clone, Debug)]
pub struct ShowRes {
    pub demo: bool,
    pub part: Part,
    pub real: bool,
    pub ekf: bool,
    pub list: Vec<Man>,
    pub res: Vec<ManRes>,
    pub total: f64,
    pub max: f64,
    pub pct: f64,
    pub complete: bool,
    pub note: String,
}

#[derive(Clone, Debug)]
pub struct Show {
    pub demo: bool,
    pub part: Part,
    pub list: Vec<Man>,
    pub i: usize,
    pub ph: Ph,
    pub t_ph: f64,
    pub t_ok: f64,
    /// nominal reference (judged, drawn) and the flown one (test hooks)
    pub nom: Track,
    pub track: Track,
    pub gid: u32,
    pub rec: Vec<Sample>,
    rec_acc: f64,
    pub td: Option<Touch>,
    pub res: Vec<ManRes>,
    pub bias: [f64; judge::rules::JUDGES],
    pub rng: Rng,
    /// manual: the pilot took over the positioning
    pub pilot_pos: bool,
    /// manual: time inside the box (aerobatics), time back at the end point (hover)
    pub t_in: f64,
    pub t_end: f64,
    /// a rescue zeroed this manoeuvre: wait for the recovery
    pub resc: bool,
    pub was_gnd: bool,
    pub done_req: bool,
    pub hook: ShowHook,
}

impl Show {
    pub fn man(&self) -> Man {
        self.list[self.i]
    }
    /// Manual mode: the pilot flies (Ready / Fly / Judged), the autopilot only positions.
    pub fn pilot_flies(&self) -> bool {
        !self.demo && matches!(self.ph, Ph::Ready | Ph::Fly | Ph::Judged)
    }
}

/// Show settings on the page (the setup sheet, the scoresheet).
#[derive(Clone, Copy, Debug)]
pub struct ShowUi {
    pub sheet: bool,
    pub demo: bool,
    pub part: Part,
    pub res_open: bool,
}
impl Default for ShowUi {
    fn default() -> Self {
        ShowUi { sheet: false, demo: true, part: Part::Full, res_open: false }
    }
}

/// Guidance the standard autopilot path flies while positioning.
pub(crate) struct ShowGuide {
    pub tx: f64,
    pub tz: f64,
    pub alt_t: f64,
    pub vmax: f64,
    pub vd: Option<(f64, f64)>,
    pub ff: (f64, f64),
    pub lim: f64,
    pub vz: Option<f64>,
}
pub(crate) enum ShowOut {
    Direct(Cmd),
    Guide(ShowGuide),
}

/// Result card display time (sim seconds).
pub const CARD_T: f64 = 6.0;
/// Pause after a result before the next manoeuvre.
pub const NEXT_T: f64 = 2.5;

impl Sim {
    /// Start a show with the setup sheet's choices.
    pub fn show_start(&mut self) {
        self.show_ui.sheet = false;
        self.picker = false;
        if self.h.crashed {
            self.toast("Crashed", "Tap RESET first", "bad");
            return;
        }
        if let Some(a) = &self.auto {
            if a.forced {
                self.toast("Forced landing", "Battery critical. The autopilot is landing.", "bad");
                return;
            }
        }
        if self.h.grounded {
            // a fresh pack for the competition flight (the estimators restart on the ground)
            if self.h.bat < 0.999 {
                self.h.bat = 1.0;
                self.sensors_reset();
            }
        } else if self.est.bat < 0.6 {
            self.toast("Battery too low for the show", "Land and start the show on the ground for a fresh pack.", "bad");
            return;
        }
        let ui = self.show_ui;
        let list = ui.part.list();
        // the judges' own random stream: from the sim's seed (without drawing from it, so the
        // flight's noise is unchanged), the time and the show count
        let seed = (self.rng.clone().f() * 9.007e15) as u64 ^ self.sim_t.to_bits() ^ 0x5_4057_ca7d ^ ((self.show_n as u64) << 32);
        let mut rng = Rng::new(seed);
        let mut bias = [0.0; judge::rules::JUDGES];
        for b in bias.iter_mut() {
            *b = rng.rnd1() * judge::rules::BIAS;
        }
        let hook = self.show_hook;
        let nom = build(list[0], &ShowHook::default());
        let track = build(list[0], &hook);
        self.show_n += 1;
        self.show_gid += 1;
        self.show = Some(Show {
            demo: ui.demo,
            part: ui.part,
            list,
            i: 0,
            ph: Ph::Position,
            t_ph: 0.0,
            t_ok: 0.0,
            nom,
            track,
            gid: self.show_gid,
            rec: Vec::new(),
            rec_acc: 0.0,
            td: None,
            res: Vec::new(),
            bias,
            rng,
            pilot_pos: false,
            t_in: 0.0,
            t_end: 0.0,
            resc: false,
            was_gnd: self.h.grounded,
            done_req: false,
            hook,
        });
        self.show_ui.res_open = false;
        self.show_auto_on();
    }

    /// The show autopilot (positioning, demo flying).
    fn show_auto_on(&mut self) {
        if matches!(&self.auto, Some(a) if a.mode == Mode::Show) {
            return;
        }
        self.restore_pitch();
        let hold_on = self.h.hold;
        if hold_on {
            self.set_hold(false, true);
        }
        let a = Auto::new(Mode::Show, v3(self.est.p.x, 0.0, self.est.p.z), self.est.alt.max(1.0), heading_of(self.est.q));
        self.auto = Some(a);
        self.auto_out = None;
        if !self.h.armed {
            self.sticks.thr = 0.0;
            self.h.armed = true;
            self.h.soft_start = self.h.rpm < 0.5;
        }
    }

    /// Stop the running show (results so far become the last result).
    pub fn show_stop(&mut self, why: &str) {
        let Some(sh) = self.show.take() else { return };
        self.restore_pitch();
        if matches!(&self.auto, Some(a) if a.mode == Mode::Show) {
            let alt = self.est.alt;
            if self.h.grounded {
                self.auto = None;
                self.auto_out = None;
            } else if let Some(a) = self.auto.as_mut() {
                a.mode = Mode::Hold;
                a.anchor = v3(self.est.p.x, 0.0, self.est.p.z);
                a.alt = alt.max(1.0);
                a.heading = heading_of(self.est.q);
            }
        }
        self.show_close(sh, false, why);
    }

    fn show_close(&mut self, sh: Show, complete: bool, note: &str) {
        let total: f64 = sh.res.iter().map(|r| r.ks).sum();
        let max: f64 = sh.list.iter().map(|m| 10.0 * m.k()).sum();
        let r = ShowRes {
            demo: sh.demo,
            part: sh.part,
            real: self.set.real,
            ekf: self.set.ekf,
            list: sh.list.clone(),
            res: sh.res,
            total,
            max,
            pct: if max > 0.0 { 100.0 * total / max } else { 0.0 },
            complete,
            note: note.to_string(),
        };
        let (t, s) = if complete {
            ("Show complete".to_string(), format!("{} / {} points · {}%", to_fixed(r.total, 1), to_fixed(r.max, 0), to_fixed(r.pct, 1)))
        } else {
            (format!("Show stopped: {note}"), format!("{} manoeuvres judged · {} points", r.res.len(), to_fixed(r.total, 1)))
        };
        self.toast(t, s, if complete { "good" } else { "" });
        if !r.res.is_empty() {
            self.show_ui.res_open = true;
        }
        self.show_last = Some(r);
    }

    fn rec_sample(&mut self, sh: &mut Show, dt: f64) {
        sh.rec_acc -= dt;
        if sh.rec_acc > 1e-9 {
            return;
        }
        sh.rec_acc += TS;
        let h = &self.h;
        let up = up_of(h.q);
        let nose = v3(0.0, 0.0, -1.0).rot(h.q);
        sh.rec.push(Sample { t: self.sim_t, p: v3(h.p.x, h.alt(), h.p.z), v: h.v, up, nose, hdg: heading_of(h.q), w: h.w, gnd: h.grounded, hold: h.hold });
        if sh.rec.len() > 50 * 200 {
            sh.rec.drain(..50 * 20);
        }
    }

    fn show_judge(&mut self, sh: &mut Show, zero: Option<String>) {
        let m = sh.man();
        let a: Assess = match zero {
            Some(z) => judge::zero(z),
            None => judge::assess(m, &sh.nom, &sh.rec, sh.td),
        };
        let marks = judge::marks(a.base, a.zero.is_some(), &sh.bias, &mut sh.rng);
        let (hi, lo) = judge::drop_hi_lo(&marks);
        let score = judge::kept_mean(&marks, hi, lo);
        let mut why: Vec<String> = a.downs.iter().take(4).map(|d| d.1.clone()).collect();
        if let Some(z) = &a.zero {
            why.insert(0, z.clone());
        }
        sh.res.push(ManRes { m, k: m.k(), base: a.base, marks, hi, lo, score, ks: score * m.k(), why, zero: a.zero, t: self.sim_t });
        sh.ph = Ph::Judged;
        sh.t_ph = 0.0;
        // aerobatics fly on the 3D curve: the user's pitch curve back (a rescue does that itself
        // once upright)
        if !matches!(&self.auto, Some(a) if a.mode == Mode::Rescue) {
            self.restore_pitch();
        }
    }

    fn show_next(&mut self, sh: &mut Show) -> bool {
        sh.i += 1;
        if sh.i >= sh.list.len() {
            return false;
        }
        let m = sh.man();
        sh.nom = build(m, &ShowHook::default());
        sh.track = build(m, &sh.hook);
        self.show_gid += 1;
        sh.gid = self.show_gid;
        sh.ph = Ph::Position;
        sh.t_ph = 0.0;
        sh.t_ok = 0.0;
        sh.rec.clear();
        sh.td = None;
        sh.pilot_pos = false;
        sh.t_in = 0.0;
        sh.t_end = 0.0;
        sh.resc = false;
        sh.done_req = false;
        // the autopilot positions for the next one (fly it yourself: a stick touch takes over)
        if !self.h.crashed && !matches!(&self.auto, Some(a) if a.mode == Mode::Rescue) {
            self.show_auto_on();
        }
        true
    }

    /// Stunt-aware crash protection window for the show's aerobatic manoeuvres: (name, time in, expected).
    pub fn show_window(&self) -> Option<(&'static str, f64, f64, bool)> {
        let sh = self.show.as_ref()?;
        let m = sh.man();
        if m.hover() || m == Man::Autorot {
            return None;
        }
        if sh.ph == Ph::Fly || (sh.ph == Ph::Ready && !sh.demo && self.h.alt() > 6.0) {
            return Some((m.name(), sh.t_ph, sh.track.dur, sh.demo));
        }
        None
    }

    /// Manual autorotation in the show: the autorotation crash-protection rules.
    pub fn show_ar_window(&self) -> bool {
        self.show.as_ref().is_some_and(|sh| !sh.demo && sh.man() == Man::Autorot && sh.ph != Ph::Position && self.h.hold)
    }

    /// The show's per-step state machine (after physics and sensors).
    pub(crate) fn show_tick(&mut self, dt: f64) {
        let Some(mut sh) = self.show.take() else { return };
        sh.t_ph += dt;
        let m = sh.man();
        let mode = self.auto.as_ref().map(|a| a.mode);
        // aborts
        if self.h.crashed {
            if sh.ph != Ph::Judged {
                self.show_judge(&mut sh, Some("crashed".into()));
            }
            while sh.i + 1 < sh.list.len() {
                sh.i += 1;
                self.show_judge(&mut sh, Some("not flown (crashed)".into()));
            }
            self.show_close(sh, false, "crashed");
            return;
        }
        if matches!(&self.auto, Some(a) if a.mode == Mode::Land && a.fs) {
            self.show = Some(sh);
            self.show_stop("battery low");
            return;
        }
        let rescue = mode == Some(Mode::Rescue);
        let grounded = self.h.grounded;
        let touch_now = grounded && !sh.was_gnd;
        sh.was_gnd = grounded;
        match sh.ph {
            Ph::Position => {
                if rescue {
                    sh.t_ok = 0.0;
                } else if !sh.pilot_pos {
                    if mode.is_none() && !sh.demo && sh.t_ph > 0.1 {
                        sh.pilot_pos = true; // the pilot took over the positioning
                    } else if mode != Some(Mode::Show) {
                        self.show_auto_on();
                    }
                }
                let e = sh.track.entry;
                let est = &self.est;
                let ok = if !sh.demo && sh.pilot_pos {
                    if m.hover() {
                        (est.p.x - e.x).hypot(est.p.z - e.z) < 0.6 && (est.alt - e.y).abs() < 0.4 && est.v.length() < 0.3 && wrap(heading_of(est.q) - sh.track.hdg).abs() < 15.0 * DEG
                    } else if m == Man::Autorot {
                        est.alt > 15.0
                    } else {
                        (est.p.x - geo::AX).abs() > geo::EDGE + 3.0 && est.alt > 8.0
                    }
                } else {
                    let hov = m.hover();
                    let (tp, tv, th, ta) = if hov { (0.25, 0.15, 4.0, 0.15) } else { (1.5, 0.6, 8.0, 0.6) };
                    (est.p.x - e.x).hypot(est.p.z - e.z) < tp
                        && est.v.length() < tv
                        && (est.alt - e.y).abs() < ta
                        && wrap(heading_of(est.q) - sh.track.hdg).abs() < th * DEG
                        && !self.h.grounded
                };
                sh.t_ok = if ok { sh.t_ok + dt } else { 0.0 };
                if sh.t_ok > if m.hover() { 1.2 } else { 1.0 } {
                    sh.t_ph = 0.0;
                    sh.rec.clear();
                    sh.rec_acc = 0.0;
                    sh.td = None;
                    if !m.hover() && m != Man::Autorot {
                        // aerobatics on the 3D pitch curve, demo and pilot alike (the user's
                        // curve comes back after the manoeuvre; the stick is remapped, no jump)
                        if self.stunt_p3d.is_none() {
                            self.stunt_p3d = Some(self.set.pitch3d);
                        }
                        if !self.set.pitch3d {
                            let c = self.h.coll_cmd;
                            self.set.pitch3d = true;
                            self.sticks.thr = self.stick_of_coll(c);
                        }
                    }
                    if sh.demo {
                        sh.ph = Ph::Fly;
                        if m == Man::Autorot {
                            self.start_auto(Mode::Autorot, true, false, false, true);
                        }
                    } else {
                        sh.ph = Ph::Ready;
                        if mode == Some(Mode::Show) {
                            self.auto = None;
                            self.auto_out = None;
                        }
                    }
                }
            }
            Ph::Ready => {
                self.rec_sample(&mut sh, dt);
                if rescue {
                    sh.ph = Ph::Position;
                    sh.t_ok = 0.0;
                    sh.pilot_pos = false;
                } else {
                    let e = sh.track.entry;
                    let h = &self.h;
                    let started = if m.hover() {
                        v3(h.p.x - e.x, h.alt() - e.y, h.p.z - e.z).length() > 0.4 || wrap(heading_of(h.q) - sh.track.hdg).abs() > 10.0 * DEG
                    } else if m == Man::Autorot {
                        h.hold && !h.grounded
                    } else {
                        (h.p.x - geo::AX).abs() < geo::EDGE
                    };
                    if sh.done_req {
                        sh.done_req = false;
                        self.show_judge(&mut sh, Some("skipped".into()));
                    } else if started {
                        // keep 2 s before the start
                        let t0 = self.sim_t - 2.0;
                        sh.rec.retain(|s| s.t >= t0);
                        sh.ph = Ph::Fly;
                        sh.t_ph = 0.0;
                    }
                }
            }
            Ph::Fly => {
                self.rec_sample(&mut sh, dt);
                if touch_now && sh.td.is_none() {
                    sh.td = Some(Touch { vs: self.h.last_impact, hs: self.h.last_touch_hs, p: v3(self.h.p.x, 0.0, self.h.p.z) });
                }
                if rescue {
                    let why = self.auto.as_ref().map(|a| a.why.clone()).unwrap_or_default();
                    sh.resc = true;
                    self.show_judge(&mut sh, Some(format!("crash protection: {}", why.to_lowercase())));
                } else {
                    let h = &self.h;
                    let fin = if sh.done_req {
                        true
                    } else if sh.demo {
                        match m {
                            Man::Land => sh.td.is_some() && grounded && sh.t_ph > sh.track.dur * 0.3 && self.since_td(&sh) > 1.0,
                            Man::Autorot => sh.td.is_some() && self.auto.is_none() && self.since_td(&sh) > 1.0,
                            _ => sh.t_ph >= sh.track.dur,
                        }
                    } else if m == Man::Land || m == Man::Autorot {
                        sh.td.is_some() && grounded && self.since_td(&sh) > 1.0
                    } else if m.hover() {
                        let last = sh.track.pts.last().unwrap().p;
                        let back = (h.p.x - last.x).hypot(h.p.z - last.z) < 0.6 && (h.alt() - last.y).abs() < 0.5 && h.v.length() < 0.25;
                        sh.t_end = if back && sh.t_ph > 0.5 * sh.track.dur { sh.t_end + dt } else { 0.0 };
                        sh.t_end > 1.0
                    } else {
                        let inside = (h.p.x - geo::AX).abs() < geo::EDGE;
                        if inside {
                            sh.t_in += dt;
                        }
                        !inside && sh.t_in > 2.0
                    };
                    let timeout = sh.t_ph > sh.track.dur * 2.5 + 15.0;
                    if fin || timeout {
                        self.show_judge(&mut sh, None);
                    }
                }
            }
            Ph::Judged => {
                // after a rescue: wait for the recovery (hover hold), then go on
                let wait = rescue || (sh.resc && mode == Some(Mode::Rescue));
                if !wait && sh.t_ph > NEXT_T && !self.show_next(&mut sh) {
                    // done: on the ground the autopilot lets go, in the air it holds a hover
                    if matches!(&self.auto, Some(a) if a.mode == Mode::Show) {
                        if self.h.grounded {
                            self.auto = None;
                            self.auto_out = None;
                            self.sticks.thr = 0.0;
                        } else if let Some(a) = self.auto.as_mut() {
                            a.mode = Mode::Hold;
                            a.anchor = v3(self.est.p.x, 0.0, self.est.p.z);
                            a.alt = self.est.alt;
                        }
                    }
                    self.show_close(sh, true, "");
                    return;
                }
            }
        }
        self.show = Some(sh);
    }

    fn since_td(&self, sh: &Show) -> f64 {
        sh.rec.iter().rev().take_while(|s| s.gnd).count() as f64 * TS
    }

    /// Reference-tracking thrust-vector controller (upright, inverted, vertical): acceleration
    /// from the reference plus position / velocity feedback, the specific force it needs (drag
    /// fed forward), body up from that force (or prescribed in rolls), nose from the reference,
    /// body rates toward that attitude with the reference rate fed forward, collective from the
    /// force along the shaft.
    pub(crate) fn track_cmd(&self, r: &RefPt, hover: bool) -> Cmd {
        let est = &self.est;
        let p = v3(est.p.x, est.alt, est.p.z);
        let (kp, kv, emax) = if hover { (1.0, 2.2, 1.2) } else { (0.8, 1.8, 4.0) };
        let mut ev = (r.v - est.v) + (r.p - p) * kp;
        let em = ev.length();
        if em > emax {
            ev *= emax / em;
        }
        let a = r.a + ev * kv;
        let vr = est.v - est.wind;
        let dk = drag_k(vr.length());
        let mut f = a + Y * G + vr * dk;
        if hover {
            f.y = f.y.max(2.0);
            let hmax = (24.0 * DEG).tan() * f.y;
            let hm = f.x.hypot(f.z);
            if hm > hmax {
                f.x *= hmax / hm;
                f.z *= hmax / hm;
            }
        }
        let up_des = up_for(f, r.uph, r.att.max(0.0));
        let up_des = if r.att > 0.0 { (up_des * (1.0 - r.att) + r.up * r.att).normalize() } else { up_des };
        let q_des = quat_up_nose(up_des, r.nose);
        let ff = r.w.rot(est.q.inv());
        let (rate, _) = self.att_rate(q_des, ff);
        let up = up_of(est.q);
        let vu = vr.dot(up);
        let ip = (vr.length_sq() - vu * vu).max(0.0).sqrt();
        let lo = if self.set.pitch3d { -1.0 } else { 0.0 };
        let coll = if r.att > 0.5 {
            // prescribed attitude (rolls): hold the height with the vertical thrust the disc can
            // give (saturated at 2.3 g, faded near knife edge), like the stunt roll
            let az = r.a.y + 2.0 * (r.p.y - est.alt) + 2.5 * (r.v.y - est.v.y);
            clamp(self.coll_vert(az, up), lo, 1.0)
        } else {
            let t = f.dot(up) + damp_of(est.rpm.max(0.3), ip, 0.0) * vu;
            clamp(t / self.rpm_k(up.y, ip), lo, 1.0)
        };
        Cmd { coll, rate: Some(rate), ..Cmd::default() }
    }

    /// Mode::Show autopilot step: positioning guidance, or the demo flying the reference.
    pub(crate) fn show_fly(&mut self, a: &mut Auto, _dt: f64) -> ShowOut {
        let hold_here = |s: &Sim, a: &Auto| ShowOut::Guide(ShowGuide { tx: a.anchor.x, tz: a.anchor.z, alt_t: a.alt, vmax: 2.5, vd: None, ff: (0.0, 0.0), lim: 24.0 * DEG, vz: if s.h.grounded { Some(-0.5) } else { None } });
        let Some(sh) = self.show.as_ref() else { return hold_here(self, a) };
        let m = sh.man();
        match sh.ph {
            Ph::Position => {
                let e = sh.track.entry;
                let hd = sh.track.hdg;
                let d = (e.x - self.est.p.x).hypot(e.z - self.est.p.z);
                a.anchor = v3(e.x, 0.0, e.z);
                a.alt = e.y;
                if d > land::R_APP {
                    let h_tr = if m.hover() { e.y.max(3.0) } else { e.y };
                    let tr = self.transit(a, e.x, e.z, h_tr);
                    if d < 10.0 {
                        a.heading = hd;
                    }
                    let vz = if tr.climbing { Some(clamp(1.2 * (h_tr - self.est.alt), -1.5, land::CLIMB_VZ)) } else { None };
                    return ShowOut::Guide(ShowGuide { tx: e.x, tz: e.z, alt_t: tr.alt_t, vmax: 2.5, vd: Some(tr.vd), ff: tr.ff, lim: land::LIM * DEG, vz });
                }
                a.heading = hd;
                let vmax = if m.hover() { 1.0 } else { 2.5 };
                ShowOut::Guide(ShowGuide { tx: e.x, tz: e.z, alt_t: e.y, vmax, vd: None, ff: (0.0, 0.0), lim: 24.0 * DEG, vz: None })
            }
            Ph::Fly if sh.demo && m != Man::Autorot => {
                if m == Man::Land && (sh.td.is_some() || self.h.grounded && sh.t_ph > 2.0) {
                    return ShowOut::Direct(Cmd { coll: if self.set.pitch3d { -0.2 } else { 0.0 }, ..Cmd::default() });
                }
                let r = sh.track.at(sh.t_ph);
                ShowOut::Direct(self.track_cmd(&r, m.hover()))
            }
            _ => {
                // judged / waiting: hold the end of the manoeuvre (on the skids after a landing)
                if self.h.grounded {
                    return ShowOut::Direct(Cmd { coll: if self.set.pitch3d { -0.2 } else { 0.0 }, ..Cmd::default() });
                }
                if sh.ph == Ph::Judged && sh.t_ph < 0.05 {
                    a.anchor = v3(self.est.p.x, 0.0, self.est.p.z);
                    a.alt = self.est.alt;
                    a.heading = heading_of(self.est.q);
                }
                hold_here(self, a)
            }
        }
    }

    /// Status line during a show.
    pub(crate) fn show_cue(&self) -> Option<String> {
        let sh = self.show.as_ref()?;
        let m = sh.man();
        let n = sh.list.len();
        let head = format!("SHOW {}/{} · ", sh.i + 1, n);
        let nm = m.name();
        let k = to_fixed(m.k(), 1);
        if matches!(&self.auto, Some(a) if a.mode == Mode::Rescue) {
            return Some(format!("{head}Crash protection <em>recovering</em>: {nm} scores 0, the show continues."));
        }
        Some(match sh.ph {
            Ph::Position => {
                if !sh.demo && sh.pilot_pos {
                    let e = sh.track.entry;
                    if m.hover() {
                        let d = (self.est.p.x - e.x).hypot(self.est.p.z - e.z);
                        format!(
                            "{head}<em>{nm}</em> (K {k}): hover over the start circle, {} m high{} ({} m to go).",
                            to_fixed(e.y, 1),
                            if m == Man::Eight { ", nose toward you" } else { ", tail in" },
                            to_fixed(d, 1)
                        )
                    } else if m == Man::Autorot {
                        format!("{head}<em>{nm}</em> (K {k}): climb above 15 m upwind of the pad (the right side).")
                    } else {
                        format!("{head}<em>{nm}</em> (K {k}): fly out beyond the box edge pole at 12 to 15 m, then in along the line.")
                    }
                } else if sh.demo {
                    format!("{head}<em>{nm}</em> (K {k}): autopilot positioning for the entry. Touch a stick to stop the show.")
                } else {
                    format!("{head}<em>{nm}</em> (K {k}): the autopilot takes you to the entry, then hands over.")
                }
            }
            Ph::Ready => format!(
                "<em>GO</em> {head}{nm}: {}{} Tap DONE when finished.",
                m.instr(),
                if !m.hover() && m != Man::Autorot { " 3D pitch is on." } else { "" }
            ),
            Ph::Fly => {
                if sh.demo {
                    format!("{head}<em>{nm}</em> (K {k}): {}", m.instr())
                } else {
                    format!("{head}<em>{nm}</em>: flying, {} s. {}", to_fixed(sh.t_ph, 0), if m.hover() && m != Man::Land { "Stop back on the start circle to finish, or tap DONE." } else if m == Man::Land || m == Man::Autorot { "Touch down to finish." } else { "Leave the box to finish, or tap DONE." })
                }
            }
            Ph::Judged => {
                let r = sh.res.last();
                let sc = r.map(|r| to_fixed(r.score, 1)).unwrap_or_default();
                let next = sh.list.get(sh.i + 1).map(|m| format!(" Next: {}.", m.name())).unwrap_or_else(|| " That was the last one.".into());
                format!("{head}{nm}: <em>{sc}</em> from the judges.{next}")
            }
        })
    }
}
