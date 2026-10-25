//! STUNTS: scripted aerobatic manoeuvres flown by the autopilot.
//!
//! Each stunt climbs to its safe height (15..20 m), builds speed on a straight line in front of
//! the pilot for the loop and the roll, switches to the 3D pitch curve, flies the manoeuvre as a
//! scripted attitude / rate / collective / yaw profile with attitude feedback, then recovers to
//! hover hold. Phases: Climb -> (Speed) -> Man -> Recover -> hold.
//!
//! - Pirouette: 360 deg of yaw at 180 deg/s in a position-held hover.
//! - Loop: 360 deg nose-up pitch at 8 m/s; collective from a vertical acceleration schedule
//!   `az = 9 cos(theta)` (positive at the bottom, negative collective over the top).
//! - Roll: 360 deg roll at 230 deg/s in forward flight; collective holds altitude and goes
//!   negative through inverted.
//! - Flip: stationary 360 deg forward pitch flip at 230 deg/s.
//! - Inverted hover: half flip to inverted, 5 s position- and heading-held inverted hover on
//!   negative collective, half flip back upright.
//!
//! Rotation segments follow a reference attitude `q_ref = q_base * rot(axis, theta(t))` where
//! theta(t) is a sine-ramped rate profile; the rate demand to the flybarless rate loop is the
//! profile rate fed forward plus 7 x the attitude error (rotation vector, body frame).
//! Collective in a rotation keeps the vertical thrust component at g plus the wanted vertical
//! acceleration, saturated at 1.7 g and faded out when the disc is near vertical.
//! Hover segments use a thrust-vector controller: a desired acceleration (position / velocity /
//! altitude hold) becomes a desired disc normal (negated when inverted) and a thrust along it.

use crate::autopilot::{Auto, Mode};
use crate::heli::*;
use crate::jsonw::{round_str, to_fixed};
use crate::math::*;
use crate::sim::{Cmd, Sim};
use crate::units::{DEG, G, PI};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Pirouette = 0,
    Loop = 1,
    Roll = 2,
    Flip = 3,
    Inverted = 4,
}
pub const KINDS: [Kind; 5] = [Kind::Pirouette, Kind::Loop, Kind::Roll, Kind::Flip, Kind::Inverted];

impl Kind {
    pub fn from_idx(i: usize) -> Option<Kind> {
        KINDS.get(i).copied()
    }
    pub fn name(self) -> &'static str {
        match self {
            Kind::Pirouette => "Pirouette",
            Kind::Loop => "Loop",
            Kind::Roll => "Roll",
            Kind::Flip => "Flip",
            Kind::Inverted => "Inverted hover",
        }
    }
    pub fn desc(self) -> &'static str {
        match self {
            Kind::Pirouette => "360° yaw at 180°/s, position held",
            Kind::Loop => "360° nose-up loop at 8 m/s",
            Kind::Roll => "360° roll in forward flight",
            Kind::Flip => "stationary 360° pitch flip",
            Kind::Inverted => "half flip, 5 s inverted, flip back",
        }
    }
    /// Safe height for the manoeuvre, m.
    pub fn alt(self) -> f64 {
        match self {
            Kind::Pirouette => 15.0,
            Kind::Loop | Kind::Roll | Kind::Flip => 18.0,
            Kind::Inverted => 20.0,
        }
    }
    /// Flown on a straight line with forward speed.
    pub fn line(self) -> bool {
        matches!(self, Kind::Loop | Kind::Roll)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SPhase {
    Climb = 0,
    Speed = 1,
    Man = 2,
    Recover = 3,
}

/// One segment of a manoeuvre profile.
#[derive(Clone, Copy, Debug)]
pub enum Seg {
    /// rotate about a body axis by `ang` (signed) with peak rate `w` and sine ramps of `tr` s
    Rot { axis: V3, ang: f64, w: f64, tr: f64 },
    /// yaw about the vertical by `ang` in a held hover
    Yaw { ang: f64, w: f64, tr: f64 },
    /// held inverted hover for `dur` s
    Inv { dur: f64 },
}

const XB: V3 = v3(1.0, 0.0, 0.0);
const ZB: V3 = v3(0.0, 0.0, 1.0);
/// Profile constants (see README "Stunts").
pub mod prof {
    pub const LINE_V: f64 = 8.0;
    pub const PIR_W: f64 = core::f64::consts::PI; // 180 deg/s
    pub const LOOP_W: f64 = 2.0;
    pub const LOOP_AZ: f64 = 12.0;
    pub const ROLL_W: f64 = 4.0;
    pub const FLIP_W: f64 = 4.0;
    pub const HALF_W: f64 = 3.5;
    pub const INV_T: f64 = 5.0;
    pub const RAMP: f64 = 0.25;
    /// attitude error -> rate gain, and the rate demand limit
    pub const K_ATT: f64 = 7.0;
    pub const W_MAX: f64 = 5.0;
    /// collective in rotations: vertical thrust saturates at this many g, fades below |up.y| = FADE
    pub const T_LIM: f64 = 2.3;
    pub const FADE: f64 = 0.15;
    /// a rotation segment ends once its reference is done and the attitude is within this
    pub const DONE_ERR: f64 = 20.0;
    /// line stunts are pitched this far nose down at entry (keeps some forward drive)
    pub const LINE_PITCH: f64 = 10.0;
}

pub fn segs(k: Kind) -> Vec<Seg> {
    use prof::*;
    match k {
        Kind::Pirouette => vec![Seg::Yaw { ang: 2.0 * PI, w: PIR_W, tr: 0.35 }],
        Kind::Loop => vec![Seg::Rot { axis: XB, ang: 2.0 * PI, w: LOOP_W, tr: 0.3 }],
        Kind::Roll => vec![Seg::Rot { axis: ZB, ang: -2.0 * PI, w: ROLL_W, tr: RAMP }],
        Kind::Flip => vec![Seg::Rot { axis: XB, ang: -2.0 * PI, w: FLIP_W, tr: RAMP }],
        Kind::Inverted => vec![
            Seg::Rot { axis: XB, ang: -PI, w: HALF_W, tr: RAMP },
            Seg::Inv { dur: INV_T },
            Seg::Rot { axis: XB, ang: -PI, w: HALF_W, tr: RAMP },
        ],
    }
}

pub fn seg_dur(s: &Seg) -> f64 {
    match *s {
        Seg::Rot { ang, w, tr, .. } | Seg::Yaw { ang, w, tr } => ang.abs() / w + tr,
        Seg::Inv { dur } => dur,
    }
}

/// Expected manoeuvre duration, s.
pub fn expected(k: Kind) -> f64 {
    segs(k).iter().map(seg_dur).sum()
}

/// Sine-ramped rate profile: angle and rate at time t for a total angle `a` (> 0).
pub fn ramp(a: f64, w: f64, tr: f64, t: f64) -> (f64, f64) {
    let tt = a / w + tr;
    let t = clamp(t, 0.0, tt);
    let up = |s: f64| (w * (0.5 * s - tr / (2.0 * PI) * (PI * s / tr).sin()), w * 0.5 * (1.0 - (PI * s / tr).cos()));
    if t < tr {
        up(t)
    } else if t > tt - tr {
        let (th, r) = up(tt - t);
        (a - th, r)
    } else {
        (w * (0.5 * tr + (t - tr)), w)
    }
}

/// Live stunt state (inside `Auto`).
#[derive(Clone, Debug)]
pub struct StuntSt {
    pub kind: Kind,
    pub phase: SPhase,
    /// time in the current phase
    pub t_ph: f64,
    /// time the climb / settle conditions have held
    pub t_ok: f64,
    pub alt: f64,
    /// line stunts: direction along x (+1 / -1), line z and start x
    pub dir: f64,
    pub z_line: f64,
    pub x0: f64,
    // manoeuvre
    pub t_man: f64,
    pub t_exp: f64,
    pub seg: usize,
    pub seg_t: f64,
    pub q_base: Quat,
    pub q_ref: Quat,
    pub p_ref: V3,
    pub alt_ref: f64,
    pub hdg_ref: f64,
    /// largest attitude error against the reference during the manoeuvre, deg
    pub max_att_err: f64,
    // scoring, from the true state
    pub e_alt: f64,
    pub e_p: V3,
    pub e_hdg: f64,
    pub min_alt: f64,
    pub max_dev: f64,
    pub hdg_err: f64,
    pub t_rec: f64,
    /// upright again after the manoeuvre: stunt-aware crash protection ends here
    pub upright: bool,
}

/// Result of a completed stunt.
#[derive(Clone, Debug)]
pub struct StuntRes {
    pub kind: Kind,
    pub entry_alt: f64,
    pub min_alt: f64,
    pub height_lost: f64,
    pub max_dev: f64,
    pub hdg_err: f64,
    pub rec_t: f64,
    pub man_t: f64,
    pub score: u32,
}

/// Test hooks for deliberately broken stunts.
#[derive(Clone, Copy, Debug)]
pub struct StuntHook {
    /// fly the manoeuvre at this height instead of the safe height
    pub alt: Option<f64>,
    /// from this time into the manoeuvre, force the collective to this value
    pub cut_coll: Option<(f64, f64)>,
    /// scale the rate demand (a weak / failing head)
    pub rate_scale: f64,
}
impl Default for StuntHook {
    fn default() -> Self {
        StuntHook { alt: None, cut_coll: None, rate_scale: 1.0 }
    }
}

pub fn score(k: Kind, lost: f64, dev: f64, hdg: f64, rec: f64) -> u32 {
    let rec_ok = if k.line() { 4.5 } else { 2.0 };
    let dev_ok = if k.line() { 1.0 } else { 0.5 };
    let s = 100.0 - 8.0 * (lost - 0.3).max(0.0) - 10.0 * (dev - dev_ok).max(0.0) - 0.8 * (hdg.abs() - 3.0).max(0.0) - 6.0 * (rec - rec_ok).max(0.0);
    clamp(s, 0.0, 100.0).round() as u32
}

/// Guidance the standard autopilot path flies in the climb, speed and recover phases.
pub(crate) struct Guide {
    pub tx: f64,
    pub tz: f64,
    pub alt_t: f64,
    pub vmax: f64,
    pub vd: Option<(f64, f64)>,
    pub lim: f64,
}

pub(crate) enum StuntOut {
    Direct(Cmd),
    Guide(Guide),
}

impl Sim {
    /// Start a stunt (button / key / picker). Guards: crashed, forced landing, rescue, battery.
    pub fn start_stunt(&mut self, k: Kind) {
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
            if a.mode == Mode::Rescue && a.t < 1.5 {
                return;
            }
            if a.mode == Mode::Stunt && a.st.as_ref().is_some_and(|s| s.phase == SPhase::Man || (s.phase == SPhase::Recover && !s.upright)) {
                self.toast("Stunt in progress", "Wait for the recovery to finish", "");
                return;
            }
        }
        if self.est.bat < 0.25 {
            self.toast("Battery too low for stunts", format!("Stunts need 25%. You have {}%.", round_str(self.est.bat * 100.0)), "bad");
            return;
        }
        self.restore_pitch();
        self.set_hold(false, true);
        let (px, pz) = (self.est.p.x, self.est.p.z);
        let line = k.line();
        let alt = self.stunt_hook.alt.unwrap_or_else(|| k.alt().max(self.est.alt.min(60.0)));
        let dir = if px <= 0.0 { 1.0 } else { -1.0 };
        let z_line = clamp(pz, -24.0, -12.0);
        let x0 = -22.0 * dir;
        let anchor = if line { v3(x0, 0.0, z_line) } else { v3(clamp(px, -20.0, 20.0), 0.0, clamp(pz, -30.0, -8.0)) };
        let heading = if line { (-dir).atan2(0.0) } else { heading_of(self.est.q) };
        let mut a = Auto::new(Mode::Stunt, anchor, alt, heading);
        a.st = Some(StuntSt {
            kind: k,
            phase: SPhase::Climb,
            t_ph: 0.0,
            t_ok: 0.0,
            alt,
            dir,
            z_line,
            x0,
            t_man: 0.0,
            t_exp: expected(k),
            seg: 0,
            seg_t: 0.0,
            q_base: Quat::IDENTITY,
            q_ref: Quat::IDENTITY,
            p_ref: anchor,
            alt_ref: alt,
            hdg_ref: heading,
            max_att_err: 0.0,
            e_alt: 0.0,
            e_p: V3::ZERO,
            e_hdg: 0.0,
            min_alt: 1e9,
            max_dev: 0.0,
            hdg_err: 0.0,
            t_rec: 0.0,
            upright: false,
        });
        self.auto = Some(a);
        self.auto_out = None;
        if !self.h.armed {
            self.sticks.thr = 0.0;
            self.h.armed = true;
            self.h.soft_start = self.h.rpm < 0.5;
        }
        let sub = if line {
            format!("Climbing to {} m, then {} m/s along the field. Touch a stick to take over.", to_fixed(alt, 0), to_fixed(prof::LINE_V, 0))
        } else {
            format!("Climbing to {} m. Touch a stick to take over.", to_fixed(alt, 0))
        };
        self.toast(format!("Stunt: {}", k.name()), sub, "");
    }

    /// Put the user's pitch curve back after a stunt (remapping the throttle stick so the
    /// collective doesn't jump).
    pub(crate) fn restore_pitch(&mut self) {
        if let Some(p) = self.stunt_p3d.take() {
            if p != self.set.pitch3d {
                let c = self.h.coll_cmd;
                self.set.pitch3d = p;
                self.sticks.thr = self.stick_of_coll(c);
            }
        }
    }

    /// Stunt-aware crash protection window: from the manoeuvre start until upright again.
    pub fn stunt_window(&self) -> Option<&StuntSt> {
        let a = self.auto.as_ref()?;
        if a.mode != Mode::Stunt {
            return None;
        }
        let s = a.st.as_ref()?;
        if s.phase == SPhase::Man || (s.phase == SPhase::Recover && !s.upright) {
            Some(s)
        } else {
            None
        }
    }

    fn rpm_k(&self, up_y: f64, ip: f64) -> f64 {
        let rpm = self.est.rpm.max(0.3);
        TMAX * rpm * rpm * ge_of(self.est.p.y, up_y) * eta_of(ip)
    }

    /// Collective that holds a vertical acceleration `az` with the disc where it is.
    fn coll_vert(&self, az: f64, up: V3) -> f64 {
        let uy = up.y;
        let d = if uy.abs() < 1e-3 { 1e-3f64.copysign(uy) } else { uy };
        let t = clamp((G + az) / d, -prof::T_LIM * G, prof::T_LIM * G) * clamp(uy.abs() / prof::FADE, 0.0, 1.0);
        let vr = self.est.v - self.est.wind;
        let vu = vr.dot(up);
        let ip = (vr.length_sq() - vu * vu).max(0.0).sqrt();
        let t = t + damp_of(self.est.rpm.max(0.3), ip, 0.0) * vu * clamp(uy.abs() / prof::FADE, 0.0, 1.0);
        clamp(t / self.rpm_k(uy, ip), -1.0, 1.0)
    }

    /// Rate demand toward a reference attitude, body frame, with a feed-forward rate.
    fn att_rate(&self, q_des: Quat, w_ff: V3) -> (V3, f64) {
        let e = rotvec(self.est.q.inv() * q_des);
        let mut w = e * prof::K_ATT + w_ff;
        let m = w.length();
        if m > prof::W_MAX {
            w = w * (prof::W_MAX / m);
        }
        (w * self.stunt_hook.rate_scale, e.length() / DEG)
    }

    /// Thrust-vector hover controller (upright or inverted): position, altitude and heading hold.
    fn tv_hold(&self, p_ref: V3, alt_ref: f64, hdg: f64, yaw_ff: f64, inverted: bool) -> (Cmd, f64) {
        let est = &self.est;
        let (mut vx, mut vz) = (0.9 * (p_ref.x - est.p.x), 0.9 * (p_ref.z - est.p.z));
        let vm = vx.hypot(vz);
        if vm > 2.5 {
            vx *= 2.5 / vm;
            vz *= 2.5 / vm;
        }
        let w = est.wind;
        let (rx, ry, rz) = (est.v.x - w.x, est.v.y, est.v.z - w.z);
        let dk = 0.22 + 0.035 * (rx * rx + ry * ry + rz * rz).sqrt();
        let (mut ax, mut az) = (1.6 * (vx - est.v.x) + dk * rx, 1.6 * (vz - est.v.z) + dk * rz);
        let vy_des = clamp(1.2 * (alt_ref - est.alt), -1.5, 2.0);
        let ay = 2.5 * (vy_des - est.v.y) + dk * ry;
        let hmax = (24.0 * DEG).tan() * (G + ay).max(2.0);
        let hm = ax.hypot(az);
        if hm > hmax {
            ax *= hmax / hm;
            az *= hmax / hm;
        }
        let f = v3(ax, G + ay, az);
        let up_des = if inverted { -f.normalize() } else { f.normalize() };
        let q_des = quat_up_heading(up_des, hdg);
        let ff = Y.rot(est.q.inv()) * yaw_ff;
        let (rate, err) = self.att_rate(q_des, ff);
        let up = up_of(est.q);
        let vr = v3(rx, ry, rz);
        let vu = vr.dot(up);
        let ip = (vr.length_sq() - vu * vu).max(0.0).sqrt();
        let t = f.dot(up) + damp_of(est.rpm.max(0.3), ip, 0.0) * vu;
        let coll = clamp(t / self.rpm_k(up.y, ip), -1.0, 1.0);
        (Cmd { coll, rate: Some(rate), ..Cmd::default() }, err)
    }

    fn enter_man(&mut self, a: &mut Auto) {
        if self.stunt_p3d.is_none() {
            self.stunt_p3d = Some(self.set.pitch3d);
        }
        if !self.set.pitch3d {
            self.set.pitch3d = true;
        }
        let (est_q, est_p, est_alt) = (self.est.q, self.est.p, self.est.alt);
        let h = &self.h;
        let s = a.st.as_mut().unwrap();
        s.phase = SPhase::Man;
        s.t_ph = 0.0;
        s.t_man = 0.0;
        s.seg = 0;
        s.seg_t = 0.0;
        let hd = heading_of(est_q);
        s.hdg_ref = hd;
        s.q_base = quat_up_heading(Y, hd);
        if s.kind.line() {
            s.q_base = s.q_base * Quat::axis_angle(XB, -prof::LINE_PITCH * DEG);
        }
        s.q_ref = s.q_base;
        s.p_ref = v3(est_p.x, 0.0, est_p.z);
        s.alt_ref = est_alt;
        s.e_alt = h.alt();
        s.e_p = h.p;
        s.e_hdg = heading_of(h.q);
        s.min_alt = h.alt();
        s.max_dev = 0.0;
        s.max_att_err = 0.0;
        let name = s.kind.name();
        self.toast(name, "3D pitch on. Hands off.", "");
    }

    fn man_step(&mut self, a: &mut Auto, dt: f64) -> Cmd {
        let segs_v = segs(a.st.as_ref().unwrap().kind);
        let st = a.st.as_ref().unwrap().clone();
        let seg = segs_v[st.seg];
        let seg_t = st.seg_t + dt;
        let up = up_of(self.est.q);
        let mut next = false;
        let (mut out, err) = match seg {
            Seg::Rot { axis, ang, w, tr } => {
                let sg = ang.signum();
                let (th, thd) = ramp(ang.abs(), w, tr, seg_t);
                let q_ref = st.q_base * Quat::axis_angle(axis, sg * th);
                let (rate, err) = self.att_rate(q_ref, axis * (sg * thd));
                let az = if st.kind == Kind::Loop {
                    // positive at the bottom, negative (pushing up while inverted) over the top,
                    // with altitude feedback around the scheduled height bump
                    let bump = prof::LOOP_AZ / (prof::LOOP_W * prof::LOOP_W) * (1.0 - th.cos());
                    prof::LOOP_AZ * th.cos() + 3.0 * (st.alt_ref + bump - self.est.alt) - 2.5 * (self.est.v.y - prof::LOOP_AZ / prof::LOOP_W * th.sin())
                } else {
                    2.0 * (st.alt_ref - self.est.alt) - 2.5 * self.est.v.y
                };
                let coll = self.coll_vert(az, up);
                if seg_t >= seg_dur(&seg) && err < prof::DONE_ERR {
                    next = true;
                }
                a.st.as_mut().unwrap().q_ref = q_ref;
                (Cmd { coll, rate: Some(rate), ..Cmd::default() }, err)
            }
            Seg::Yaw { ang, w, tr } => {
                let (th, thd) = ramp(ang.abs(), w, tr, seg_t);
                let hd = st.hdg_ref + ang.signum() * th;
                let (c, _) = self.tv_hold(st.p_ref, st.alt_ref, hd, ang.signum() * thd, false);
                let e = wrap(hd - heading_of(self.est.q)).abs() / DEG;
                if seg_t >= seg_dur(&seg) && e < 15.0 {
                    next = true;
                }
                (c, e)
            }
            Seg::Inv { dur } => {
                let hd = heading_of(st.q_base);
                let (c, e) = self.tv_hold(st.p_ref, st.alt_ref, hd, 0.0, true);
                if seg_t >= dur && e < prof::DONE_ERR {
                    next = true;
                }
                (c, e)
            }
        };
        if let Some((t0, c)) = self.stunt_hook.cut_coll {
            if st.t_man >= t0 {
                out.coll = c;
            }
        }
        let s = a.st.as_mut().unwrap();
        s.seg_t = seg_t;
        s.t_man += dt;
        if seg_t > 0.3 {
            s.max_att_err = s.max_att_err.max(err);
        }
        if next {
            // the next segment starts from where this one's reference ended
            match seg {
                Seg::Rot { axis, ang, .. } => s.q_base = s.q_base * Quat::axis_angle(axis, ang),
                Seg::Inv { .. } => s.q_base = quat_up_heading(-Y, heading_of(s.q_base)),
                Seg::Yaw { ang, .. } => s.hdg_ref = wrap(s.hdg_ref + ang),
            }
            s.seg += 1;
            s.seg_t = 0.0;
            if s.seg >= segs_v.len() {
                // manoeuvre done: recover to a hover
                s.phase = SPhase::Recover;
                s.t_ph = 0.0;
                s.t_ok = 0.0;
                s.hdg_err = wrap(heading_of(self.h.q) - s.e_hdg) / DEG;
                let est = &self.est;
                let v = v3(est.v.x, 0.0, est.v.z);
                let stop = v * (v.length() / (2.0 * 3.5));
                a.anchor = if s.kind.line() { v3(est.p.x, 0.0, est.p.z) + stop } else { s.p_ref };
                a.alt = s.alt_ref;
                a.heading = if s.kind.line() { heading_of(s.q_base) } else { s.hdg_ref };
            }
        }
        out
    }

    /// Stunt autopilot step: Direct command in the manoeuvre, guidance for the standard path otherwise.
    pub(crate) fn stunt_step(&mut self, a: &mut Auto, dt: f64) -> StuntOut {
        let est_alt = self.est.alt;
        let (h_alt, h_p, h_up) = (self.h.alt(), self.h.p, up_of(self.h.q));
        let est = &self.est;
        let hs = est.v.hyp_xz();
        let hd_err = wrap(a.heading - heading_of(est.q)).abs();
        let s = a.st.as_mut().unwrap();
        s.t_ph += dt;
        // scoring from the true state, manoeuvre start to recovered
        if matches!(s.phase, SPhase::Man | SPhase::Recover) {
            s.min_alt = s.min_alt.min(h_alt);
            let dev = if s.kind.line() { (h_p.z - s.e_p.z).abs() } else { (h_p.x - s.e_p.x).hypot(h_p.z - s.e_p.z) };
            s.max_dev = s.max_dev.max(dev);
        }
        match s.phase {
            SPhase::Climb => {
                let de = (a.anchor.x - est.p.x).hypot(a.anchor.z - est.p.z);
                let ok = de < if s.kind.line() { 2.5 } else { 1.2 }
                    && (est_alt - s.alt).abs() < 0.8
                    && hs < if s.kind.line() { 1.2 } else { 0.5 }
                    && est.v.y.abs() < 0.5
                    && hd_err < 0.15;
                s.t_ok = if ok { s.t_ok + dt } else { 0.0 };
                if s.t_ok > 0.5 {
                    s.t_ok = 0.0;
                    if s.kind.line() {
                        s.phase = SPhase::Speed;
                        s.t_ph = 0.0;
                    } else {
                        self.enter_man(a);
                        return StuntOut::Direct(self.man_step(a, dt));
                    }
                }
                let lim = 24.0 * DEG;
                StuntOut::Guide(Guide { tx: a.anchor.x, tz: a.anchor.z, alt_t: s.alt, vmax: 5.0, vd: None, lim })
            }
            SPhase::Speed => {
                let v = (1.0 + 2.5 * s.t_ph).min(prof::LINE_V);
                let along = est.v.x * s.dir;
                let prog = (est.p.x - s.x0) * s.dir;
                let vd = (s.dir * v, clamp(0.8 * (s.z_line - est.p.z), -2.0, 2.0));
                if (along > prof::LINE_V - 0.4 && s.t_ph > 2.0) || prog > 24.0 {
                    self.enter_man(a);
                    return StuntOut::Direct(self.man_step(a, dt));
                }
                StuntOut::Guide(Guide { tx: 0.0, tz: 0.0, alt_t: s.alt, vmax: 2.5, vd: Some(vd), lim: 30.0 * DEG })
            }
            SPhase::Man => StuntOut::Direct(self.man_step(a, dt)),
            SPhase::Recover => {
                s.t_rec += dt;
                if h_up.y > (30.0 * DEG).cos() || self.est.q.w.is_nan() {
                    s.upright = true;
                }
                let up = up_of(est.q);
                let ok = s.t_ph > 0.5 && up.y > (10.0 * DEG).cos() && est.v.length() < 1.0 && (est_alt - a.alt).abs() < 1.2;
                if ok {
                    self.finish_stunt(a);
                    return StuntOut::Guide(Guide { tx: a.anchor.x, tz: a.anchor.z, alt_t: a.alt, vmax: 2.5, vd: None, lim: 24.0 * DEG });
                }
                StuntOut::Guide(Guide { tx: a.anchor.x, tz: a.anchor.z, alt_t: a.alt, vmax: 2.5, vd: None, lim: 24.0 * DEG })
            }
        }
    }

    fn finish_stunt(&mut self, a: &mut Auto) {
        let s = a.st.take().unwrap();
        let lost = (s.e_alt - s.min_alt).max(0.0);
        let sc = score(s.kind, lost, s.max_dev, s.hdg_err, s.t_rec);
        let res = StuntRes {
            kind: s.kind,
            entry_alt: s.e_alt,
            min_alt: s.min_alt,
            height_lost: lost,
            max_dev: s.max_dev,
            hdg_err: s.hdg_err,
            rec_t: s.t_rec,
            man_t: s.t_man,
            score: sc,
        };
        let i = s.kind as usize;
        if self.stunt_best[i].map_or(true, |b| sc > b) {
            self.stunt_best[i] = Some(sc);
        }
        self.stunt_last = Some(res.clone());
        self.stunt_count += 1;
        self.restore_pitch();
        a.mode = Mode::Hold;
        a.anchor = v3(self.est.p.x, 0.0, self.est.p.z);
        a.alt = self.est.alt;
        a.heading = heading_of(self.est.q);
        self.toast(
            format!("{}: score {}", s.kind.name(), sc),
            format!(
                "Height lost {} m · max deviation {} m · heading {}° · recovered in {} s. Hover hold.",
                to_fixed(lost, 1),
                to_fixed(s.max_dev, 1),
                to_fixed(s.hdg_err.abs(), 0),
                to_fixed(s.t_rec, 1)
            ),
            if sc >= 70 { "good" } else { "" },
        );
    }

    /// Cue line for a stunt.
    pub(crate) fn stunt_cue(&self, s: &StuntSt) -> String {
        let n = s.kind.name();
        match s.phase {
            SPhase::Climb => format!("Stunt ({n}): <em>climbing</em> to {} m. Touch a stick to take over.", to_fixed(s.alt, 0)),
            SPhase::Speed => format!(
                "Stunt ({n}): <em>building speed</em>, {} of {} m/s. Touch a stick to take over.",
                to_fixed(self.est.v.hyp_xz(), 1),
                to_fixed(prof::LINE_V, 0)
            ),
            SPhase::Man => format!("Stunt: <em>{}</em> in progress. Touch a stick to take over.", n.to_lowercase()),
            SPhase::Recover => format!("Stunt ({n}): <em>recovering</em> to a hover."),
        }
    }
}
