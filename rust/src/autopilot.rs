//! Autopilot: takeoff, hover hold, continuous racetrack circuit, land, crash protection (rescue)
//! and a practice-autorotation demo, plus the low-battery failsafe. Reads only the estimated
//! state and returns commands in self-level stick units plus a raw collective.

use crate::heli::*;
use crate::heli::PAD;
use crate::jsonw::{round_str, to_fixed};
use crate::math::*;
use crate::sim::{AutoResult, Cmd, FsAt, Sim};
use crate::units::{DEG, G, PI};
use std::sync::OnceLock;

pub const CRUISE: f64 = 5.0;
pub const CIRC_ALT: f64 = 5.0;

/// Practice autorotation geometry and gains.
pub mod ar {
    pub const ENTRY_ALT: f64 = 25.0;
    pub const ENTRY_D: f64 = 22.0;
    pub const RUN_D: f64 = 16.0;
    pub const RUN_V: f64 = 7.0;
    pub const COLL: f64 = 0.1;
    pub const FLARE_ALT: f64 = 6.0;
    pub const FLARE_D: f64 = 5.0;
    pub const CUSH_A: f64 = 5.5;
}

/// LAND transit: speed and height by distance to the pad (see README "LAND transit").
pub mod land {
    /// cruise speed cap, m/s, and the braking deceleration the speed profile is built on, m/s^2
    pub const V_MAX: f64 = 8.0;
    pub const A_BRAKE: f64 = 1.5;
    /// gentle approach inside this radius: 0.9 m/s per m of distance, at most V_APP
    pub const R_APP: f64 = 5.0;
    pub const V_APP: f64 = 2.5;
    /// airspeed cap: ground speed into a headwind is at most AIR_MAX - headwind
    pub const AIR_MAX: f64 = 9.0;
    /// braking starts earlier with a tailwind: deceleration x (1 - TAIL_K x tailwind), at least TAIL_MIN
    pub const TAIL_K: f64 = 0.08;
    pub const TAIL_MIN: f64 = 0.6;
    /// cruise height: H_NEAR beyond D_NEAR, rising to H_FAR at D_FAR; approach height from D_APP
    pub const H_NEAR: f64 = 8.0;
    pub const H_FAR: f64 = 12.0;
    pub const D_NEAR: f64 = 15.0;
    pub const D_FAR: f64 = 30.0;
    pub const D_APP: f64 = 10.0;
    /// approach height range (the height LAND was started at, clamped)
    pub const H_APP_MIN: f64 = 3.0;
    pub const H_APP_MAX: f64 = 5.0;
    /// climb first: horizontal speed limit while more than CLIMB_DEF below the cruise height
    pub const CLIMB_DEF: f64 = 1.5;
    pub const CLIMB_V: f64 = 2.0;
    pub const CLIMB_VZ: f64 = 2.5;
    /// nose along the track above this speed (with 0.5 m/s hysteresis)
    pub const YAW_V: f64 = 3.0;
    /// tilt limit in transit
    pub const LIM: f64 = 30.0;
    /// the flown target follows a switch between camera and GPS targets at this speed, m/s
    pub const TGT_SLEW: f64 = 0.8;
    /// descend when within E_DESC of the target, slower than HS_DESC and within HDG_DESC of the
    /// landing heading (tighter with the camera)
    pub const E_DESC: f64 = 0.3;
    pub const E_DESC_CAM: f64 = 0.12;
    pub const HS_DESC: f64 = 0.35;
    pub const HDG_DESC: f64 = 10.0;
    /// a descent pauses (holding height) when the target moves more than this away, above 1 m
    pub const E_PAUSE: f64 = 0.6;
    /// position / velocity gains over the pad in the descent (the normal hold is 0.9 / 1.6)
    pub const KP_DESC: f64 = 1.5;
    pub const KV_DESC: f64 = 2.2;
}

/// Cruise height for a distance to the pad (before the approach-height floor and caps).
pub fn cruise_height(d: f64, h_app: f64) -> f64 {
    use land::*;
    if d >= D_FAR {
        H_FAR
    } else if d >= D_NEAR {
        H_NEAR + (H_FAR - H_NEAR) * (d - D_NEAR) / (D_FAR - D_NEAR)
    } else if d >= D_APP {
        H_NEAR
    } else if d >= R_APP {
        h_app + (H_NEAR - h_app).max(0.0) * (d - R_APP) / (D_APP - R_APP)
    } else {
        h_app
    }
}

/// Transit speed for a distance to the stopping point: braking curve `sqrt(2 a (d - R) + V_APP^2)`
/// outside the approach radius, the gentle `0.9 d` law inside. `a` is the braking deceleration.
pub fn transit_speed(d: f64, a: f64, cap: f64) -> f64 {
    use land::*;
    if d > R_APP {
        (2.0 * a * (d - R_APP) + V_APP * V_APP).sqrt().min(cap.max(V_APP))
    } else {
        (0.9 * d).min(V_APP)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Takeoff = 0,
    Hold = 1,
    Circuit = 2,
    Land = 3,
    Rescue = 4,
    Autorot = 5,
    Stunt = 6,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Climb = 0,
    Run = 1,
    Glide = 2,
    Flare = 3,
    Cushion = 4,
}

#[derive(Clone, Debug)]
pub struct Auto {
    pub mode: Mode,
    pub anchor: V3,
    pub alt: f64,
    pub heading: f64,
    /// loop index, -1 until the circuit path has been joined
    pub li: i64,
    pub prog: f64,
    pub laps: u32,
    pub lap_t: f64,
    pub lap_times: Vec<f64>,
    pub descending: bool,
    pub t: f64,
    pub forced: bool,
    pub fs: bool,
    /// surveyed pad position in the estimate's frame (moves with EKF position resets)
    pub pad_ref: V3,
    // LAND transit: start distance, approach height, height cap, flown target, nose-along-track latch
    pub land_d0: f64,
    pub land_h: f64,
    pub land_cap: f64,
    pub tgt: V3,
    pub tgt_init: bool,
    pub track_yaw: bool,
    /// flying the camera target (else GPS)
    pub cam_tgt: bool,
    // autorotation
    pub phase: Phase,
    pub min_rpm: f64,
    pub max_rpm: f64,
    pub entry: V3,
    pub entry_alt: f64,
    pub coll: f64,
    pub cut_alt: f64,
    pub glide_n: u32,
    pub glide_vy: f64,
    pub glide_hs: f64,
    // rescue
    pub why: String,
    pub was_forced: bool,
    pub min_alt: f64,
    /// rescue from inverted: latched axis (true = pitch) and direction of the shortest roll-over
    pub resc_axis: Option<(bool, f64)>,
    // stunt
    pub st: Option<crate::stunt::StuntSt>,
}

impl Auto {
    pub(crate) fn new(mode: Mode, anchor: V3, alt: f64, heading: f64) -> Auto {
        Auto {
            mode,
            anchor,
            alt,
            heading,
            li: -1,
            prog: 0.0,
            laps: 0,
            lap_t: 0.0,
            lap_times: Vec::new(),
            descending: false,
            t: 0.0,
            forced: false,
            fs: false,
            pad_ref: v3(PAD.x, 0.0, PAD.z),
            land_d0: 0.0,
            land_h: 3.0,
            land_cap: 1e9,
            tgt: anchor,
            tgt_init: false,
            track_yaw: false,
            cam_tgt: false,
            phase: Phase::Climb,
            min_rpm: 9.0,
            max_rpm: 0.0,
            entry: V3::ZERO,
            entry_alt: 0.0,
            coll: 0.0,
            cut_alt: 0.0,
            glide_n: 0,
            glide_vy: 0.0,
            glide_hs: 0.0,
            why: String::new(),
            was_forced: false,
            min_alt: 0.0,
            resc_axis: None,
            st: None,
        }
    }
    /// Phases of the autorotation where the motor is cut (protection and failsafe stand back).
    pub fn ar_unpowered(&self) -> bool {
        self.mode == Mode::Autorot && self.phase != Phase::Climb && self.phase != Phase::Run
    }
}

/// Closed racetrack in front of the pilot: rounded rectangle x -10..14, z -20..-4 with 5 m corners,
/// sampled at about 1 m, with tangents, headings and curvature per point.
pub struct Loop {
    pub pts: Vec<(f64, f64)>,
    pub n: usize,
    pub s: Vec<f64>,
    pub len: f64,
    pub tx: Vec<f64>,
    pub tz: Vec<f64>,
    pub hd: Vec<f64>,
    pub kx: Vec<f64>,
    pub kz: Vec<f64>,
    pub kh: Vec<f64>,
}

pub fn loop_path() -> &'static Loop {
    static L: OnceLock<Loop> = OnceLock::new();
    L.get_or_init(|| {
        let (x0, x1, z0, z1, r) = (-10.0f64, 14.0f64, -20.0f64, -4.0f64, 5.0f64);
        let mut pts = Vec::new();
        let line = |pts: &mut Vec<(f64, f64)>, ax: f64, az: f64, bx: f64, bz: f64| {
            let l = (bx - ax).hypot(bz - az);
            let n = l.ceil() as usize;
            for i in 0..n {
                let f = i as f64 / n as f64;
                pts.push((ax + (bx - ax) * f, az + (bz - az) * f));
            }
        };
        let arc = |pts: &mut Vec<(f64, f64)>, cx: f64, cz: f64, a0: f64| {
            let n = (r * PI / 2.0).ceil() as usize;
            for i in 0..n {
                let a = a0 + (PI / 2.0) * i as f64 / n as f64;
                pts.push((cx + r * a.cos(), cz + r * a.sin()));
            }
        };
        line(&mut pts, x0, z1 - r, x0, z0 + r);
        arc(&mut pts, x0 + r, z0 + r, PI);
        line(&mut pts, x0 + r, z0, x1 - r, z0);
        arc(&mut pts, x1 - r, z0 + r, 1.5 * PI);
        line(&mut pts, x1, z0 + r, x1, z1 - r);
        arc(&mut pts, x1 - r, z1 - r, 0.0);
        line(&mut pts, x1 - r, z1, x0 + r, z1);
        arc(&mut pts, x0 + r, z1 - r, 0.5 * PI);
        let n = pts.len();
        let mut s = vec![0.0];
        for i in 1..n {
            s.push(s[i - 1] + (pts[i].0 - pts[i - 1].0).hypot(pts[i].1 - pts[i - 1].1));
        }
        let len = s[n - 1] + (pts[0].0 - pts[n - 1].0).hypot(pts[0].1 - pts[n - 1].1);
        let (mut tx, mut tz, mut hd) = (vec![], vec![], vec![]);
        for i in 0..n {
            let (a, b) = (pts[(i + n - 1) % n], pts[(i + 1) % n]);
            let l = (b.0 - a.0).hypot(b.1 - a.1);
            tx.push((b.0 - a.0) / l);
            tz.push((b.1 - a.1) / l);
            hd.push((-tx[i]).atan2(-tz[i]));
        }
        let (mut kx, mut kz, mut kh) = (vec![], vec![], vec![]);
        for i in 0..n {
            let (a, b) = ((i + n - 1) % n, (i + 1) % n);
            let mut ds = ((s[b] - s[a]) + len) % len;
            if ds == 0.0 {
                ds = 2.0;
            }
            kx.push((tx[b] - tx[a]) / ds);
            kz.push((tz[b] - tz[a]) / ds);
            kh.push(wrap(hd[b] - hd[a]) / ds);
        }
        Loop { pts, n, s, len, tx, tz, hd, kx, kz, kh }
    })
}

struct Follow {
    vx: f64,
    vz: f64,
    ax: f64,
    az: f64,
    yaw_rate: f64,
    heading: f64,
}

/// Transit guidance: desired velocity, feed-forward acceleration, target height, heading.
pub(crate) struct Transit {
    pub vd: (f64, f64),
    pub ff: (f64, f64),
    pub alt_t: f64,
    pub climbing: bool,
}

impl Sim {
    /// Fly toward (tx, tz) and stop there: speed by distance with wind-aware braking, a height
    /// profile `alt_t` (with climb first if more than `CLIMB_DEF` below it), nose along the track.
    pub(crate) fn transit(&self, a: &mut Auto, tx: f64, tz: f64, alt_t: f64) -> Transit {
        use land::*;
        let est = &self.est;
        let (dx, dz) = (tx - est.p.x, tz - est.p.z);
        let d = dx.hypot(dz);
        let (ux, uz) = if d > 1e-6 { (dx / d, dz / d) } else { (0.0, 0.0) };
        let w_along = est.wind.x * ux + est.wind.z * uz; // > 0: tailwind
        let a_eff = A_BRAKE * (1.0 - TAIL_K * w_along.max(0.0)).max(TAIL_MIN);
        let cap = V_MAX.min(AIR_MAX + w_along.min(0.0));
        let mut v = transit_speed(d, a_eff, cap);
        let climbing = d > R_APP && self.est.alt < alt_t - CLIMB_DEF;
        if climbing {
            v = v.min(CLIMB_V);
        }
        // braking along the profile: feed the deceleration forward
        let braking = d > R_APP && v < cap - 0.05 && !climbing && est.v.hyp_xz() > v - 0.5;
        let ff = if braking { (-a_eff * ux, -a_eff * uz) } else { (0.0, 0.0) };
        if v > YAW_V + 0.5 {
            a.track_yaw = true;
        } else if v < YAW_V - 0.5 {
            a.track_yaw = false;
        }
        if a.track_yaw && d > 1.0 {
            a.heading = (-ux).atan2(-uz);
        }
        Transit { vd: (ux * v, uz * v), ff, alt_t, climbing }
    }

    /// Pad position the autopilot flies to: the camera target while it is usable (REAL), else the
    /// surveyed position in the estimate's frame.
    pub(crate) fn pad_target(&self, a: &Auto) -> (V3, bool) {
        let pl = &self.sens.pl;
        if self.set.real && self.sens.cam.enabled && pl.usable(self.est.alt) {
            (pl.pad, true)
        } else {
            (a.pad_ref, false)
        }
    }

    /// Landing heading: the tag heading from the camera when known, else the surveyed pad heading.
    fn land_heading(&self) -> f64 {
        match (self.set.real, self.sens.pl.hdg) {
            (true, Some(h)) if self.sens.pl.locked => h,
            _ => PAD.hdg,
        }
    }

    /// Cue text for LAND.
    pub(crate) fn land_cue(&self, a: &Auto) -> String {
        let cam = if a.cam_tgt { " (camera lock)" } else { "" };
        let d = (a.tgt.x - self.est.p.x).hypot(a.tgt.z - self.est.p.z);
        if a.descending {
            format!("Autopilot: <em>descending</em> onto the pad{cam}.")
        } else if d > land::R_APP && self.est.alt < cruise_height(d, a.land_h).min(a.land_cap).max(a.land_h) - land::CLIMB_DEF {
            format!("Autopilot: <em>climbing</em> to {} m, then to the pad.", to_fixed(cruise_height(d, a.land_h).min(a.land_cap).max(a.land_h), 0))
        } else if d > land::R_APP {
            format!("Autopilot: flying to the pad, <em>{} m</em> at {} m/s.", to_fixed(d, 0), to_fixed(self.est.v.hyp_xz(), 1))
        } else {
            format!("Autopilot: <em>lining up</em> over the pad{cam}.")
        }
    }

    fn loop_follow(&mut self, a: &mut Auto) -> Follow {
        let l = loop_path();
        let n = l.n;
        let (px, pz) = (self.est.p.x, self.est.p.z);
        let d2 = |i: usize| (l.pts[i].0 - px).powi(2) + (l.pts[i].1 - pz).powi(2);
        if a.li < 0 {
            let mut bi = 0;
            for i in 1..n {
                if d2(i) < d2(bi) {
                    bi = i;
                }
            }
            a.li = bi as i64;
            a.prog = 0.0;
            a.laps = 0;
            a.lap_t = self.sim_t;
        } else {
            let li = a.li as usize;
            let mut bi = li;
            for k in 1..=12 {
                let i = (li + k) % n; // forward only, modular across the seam
                if d2(i) < d2(bi) {
                    bi = i;
                }
            }
            a.prog += ((l.s[bi] - l.s[li]) + l.len) % l.len;
            a.li = bi as i64;
            if a.prog >= (a.laps + 1) as f64 * l.len {
                a.laps += 1;
                a.lap_times.push(self.sim_t - a.lap_t);
                a.lap_t = self.sim_t;
                let lt = *a.lap_times.last().unwrap();
                self.toast(
                    format!("Lap {} complete", a.laps),
                    format!("{} s. Tap LAND to come in, or touch a stick to take over.", to_fixed(lt, 1)),
                    "good",
                );
            }
        }
        let i = a.li as usize;
        let (j, ia, il) = ((i + 1) % n, (i + 2) % n, (i + 4) % n);
        let (ex, ez) = (px - l.pts[i].0, pz - l.pts[i].1);
        let along = ex * l.tx[i] + ez * l.tz[i];
        let (mut cx, mut cz) = (-(ex - along * l.tx[i]) * 0.8, -(ez - along * l.tz[i]) * 0.8);
        let cm = cx.hypot(cz);
        if cm > 2.5 {
            cx *= 2.5 / cm;
            cz *= 2.5 / cm;
        }
        let (mut vx, mut vz) = (l.tx[j] * CRUISE + cx, l.tz[j] * CRUISE + cz);
        let vm = vx.hypot(vz);
        vx *= CRUISE / vm;
        vz *= CRUISE / vm;
        Follow {
            vx,
            vz,
            ax: CRUISE * CRUISE * l.kx[ia],
            az: CRUISE * CRUISE * l.kz[ia],
            yaw_rate: CRUISE * l.kh[ia],
            heading: l.hd[il],
        }
    }

    /// Start an autopilot mode. `internal` skips the user-facing guards (failsafe, rescue).
    pub fn start_auto(&mut self, mode: Mode, internal: bool, forced: bool, fs: bool, quiet: bool) {
        if self.h.crashed {
            self.toast("Crashed", "Tap RESET first", "bad");
            return;
        }
        if !internal {
            if let Some(a) = &self.auto {
                if a.forced {
                    self.toast("Forced landing", "Battery critical. The autopilot is landing.", "bad");
                    return;
                }
                if a.mode == Mode::Rescue && a.t < 1.5 {
                    return;
                }
            }
        }
        self.restore_pitch();
        let alt = self.est.alt;
        if mode == Mode::Land && self.h.grounded && self.auto.is_none() {
            self.toast("Already on the ground", "Use TAKEOFF or CIRCUIT first", "");
            return;
        }
        if mode == Mode::Land && !self.h.armed {
            return;
        }
        self.set_hold(false, true);
        let mut a = Auto::new(mode, v3(self.est.p.x, 0.0, self.est.p.z), alt.max(3.0), heading_of(self.est.q));
        a.forced = forced;
        a.fs = fs;
        if mode == Mode::Circuit {
            a.alt = CIRC_ALT;
        }
        a.pad_ref = self.sens.pl.nom;
        if mode == Mode::Land {
            a.anchor = a.pad_ref;
            let d0 = (a.anchor.x - self.est.p.x).hypot(a.anchor.z - self.est.p.z);
            a.land_d0 = d0;
            a.land_h = clamp(alt, if d0 < land::R_APP { 2.0 } else { land::H_APP_MIN }, land::H_APP_MAX);
            a.alt = a.land_h;
            // short hops don't climb above where they started; nor does a forced landing
            a.land_cap = if d0 > land::D_NEAR && !forced { 1e9 } else { alt.max(a.land_h) };
            a.tgt = a.anchor;
            a.tgt_init = false;
            a.track_yaw = false;
        }
        if mode == Mode::Autorot {
            a.phase = Phase::Climb;
            a.min_rpm = 9.0;
            a.max_rpm = 0.0;
            if alt >= 18.0 && !self.h.grounded {
                a.entry = v3(self.est.p.x, 0.0, self.est.p.z);
                a.entry_alt = alt;
            } else {
                // run in and glide toward the pad, into the wind
                a.entry = v3(a.pad_ref.x + ar::ENTRY_D + ar::RUN_D, 0.0, a.pad_ref.z);
                a.entry_alt = ar::ENTRY_ALT;
            }
            a.alt = a.entry_alt;
        }
        let entry_alt = a.entry_alt;
        self.auto = Some(a);
        if !self.h.armed {
            self.sticks.thr = 0.0;
            self.h.armed = true;
            self.h.soft_start = self.h.rpm < 0.5;
        }
        if quiet {
            return;
        }
        let (title, sub) = match mode {
            Mode::Land => ("Autopilot: landing", "Setting down on the pad".into()),
            Mode::Autorot => (
                "Autopilot: practice autorotation",
                format!("Climbing to {} m, then motor cut and glide to the pad", to_fixed(entry_alt, 0)),
            ),
            Mode::Circuit => ("Autopilot: circuit", "Touch any stick to take control".into()),
            _ => ("Autopilot: takeoff", "Touch any stick to take control".into()),
        };
        self.toast(title, sub, "");
    }

    pub fn cancel_auto(&mut self, by_user: bool) {
        let Some(a) = &self.auto else { return };
        if by_user && (a.forced || (a.mode == Mode::Rescue && a.t < 1.5)) {
            return; // hands-off phases
        }
        let was_stunt = a.mode == Mode::Stunt || a.mode == Mode::Rescue;
        self.auto = None;
        self.auto_out = None;
        let mut sub3d = false;
        if was_stunt && self.stunt_p3d.is_some() {
            if by_user && up_of(self.est.q).y < 0.0 && self.set.pitch3d {
                // inverted when the pilot takes over: keep the 3D curve so it can be flown out
                self.stunt_p3d = None;
                sub3d = true;
            } else {
                self.restore_pitch();
            }
        }
        if by_user {
            let sub = if sub3d {
                "You have it inverted. Pitch stays on 3D: push the collective up to climb, roll or flip upright."
            } else if self.h.hold {
                "Throttle hold still ON. Tap THR HOLD to bail out."
            } else {
                "Autopilot off. The throttle stays where the autopilot left it."
            };
            self.toast("Manual control", sub, "");
        }
    }

    fn start_rescue(&mut self, why: String) {
        let was_forced = matches!(&self.auto, Some(a) if a.forced);
        self.rescues += 1;
        self.set_hold(false, true);
        self.h.soft_start = false; // bailout: straight to full power
        let mut a = Auto::new(Mode::Rescue, v3(self.est.p.x, 0.0, self.est.p.z), 3.0, heading_of(self.est.q));
        a.why = why.clone();
        a.was_forced = was_forced;
        a.min_alt = self.est.alt;
        self.auto = Some(a);
        self.toast("Crash protection", why, "bad");
    }

    /// Rescue from a big tilt or inverted: roll upright the shortest way, about the body pitch
    /// or roll axis, whichever needs less rotation. The axis and direction are latched so the
    /// rescue doesn't dither when it starts exactly upside down.
    fn rescue_rate(&self, a: &mut Auto) -> V3 {
        let g = Y.rot(self.est.q.inv()); // world up in the body frame
        let phi = g.z.atan2(g.y); // pitch-axis angle (rotation about body x)
        let psi = (-g.x).atan2(g.y); // roll-axis angle (rotation about body z)
        let (pitch, dir) = *a.resc_axis.get_or_insert_with(|| {
            let pitch = g.z.abs() > g.x.abs();
            let ang = if pitch { phi } else { psi };
            (pitch, if ang >= 0.0 { 1.0 } else { -1.0 })
        });
        let fix = |ang: f64| if ang * dir < 0.0 && ang.abs() > PI / 2.0 { ang + dir * 2.0 * PI } else { ang };
        let (on, off) = if pitch { (fix(phi), psi) } else { (fix(psi), phi) };
        let w_on = clamp(8.0 * on, -5.0, 5.0);
        let w_off = clamp(4.0 * off.sin() * g.y.max(0.0), -2.0, 2.0);
        let yaw = -self.est.w.y * 0.5;
        if pitch {
            v3(w_on, yaw, w_off)
        } else {
            v3(w_off, yaw, w_on)
        }
    }

    pub(crate) fn check_protect(&mut self) {
        let h = &self.h;
        if !self.set.protect || !h.armed || h.crashed || h.grounded || h.air_t < 0.3 {
            return;
        }
        let m = self.auto.as_ref().map(|a| a.mode);
        if matches!(m, Some(Mode::Takeoff | Mode::Land | Mode::Rescue)) {
            return;
        }
        let est = &self.est;
        let up = up_of(est.q);
        let tilt = clamp(up.y, -1.0, 1.0).acos() / DEG;
        let (alt, vy) = (est.alt, est.v.y);
        let pct = |x: f64| round_str(x * 100.0);
        let mut why = None;
        if let Some(s) = self.stunt_window() {
            // stunt-aware: big bank angles and inverted are the point; watch height, headspeed
            // and a manoeuvre that takes too long
            let run = s.t_man + if s.phase == crate::stunt::SPhase::Recover { s.t_rec } else { 0.0 };
            if alt < 6.0 {
                why = Some(format!("Too low in the {}: {} m", s.kind.name().to_lowercase(), to_fixed(alt, 1)));
            } else if est.rpm < 0.75 {
                why = Some(format!("Headspeed {}% in the {}", pct(est.rpm), s.kind.name().to_lowercase()));
            } else if run > s.t_exp + 1.5 {
                why = Some(format!("{} overran by {} s", s.kind.name(), to_fixed(run - s.t_exp, 1)));
            }
        } else if self.auto.as_ref().is_some_and(|a| a.ar_unpowered()) {
            if est.rpm < 0.65 && alt > 4.0 {
                why = Some(format!("Headspeed {}%", pct(est.rpm)));
            } else if tilt > 45.0 {
                why = Some(format!("Over-banked {}°", to_fixed(tilt, 0)));
            }
        } else {
            let vr = est.v - est.wind;
            let vu = vr.dot(up);
            let ip = (vr.length_sq() - vu * vu).max(0.0).sqrt();
            if tilt > 55.0 || (!self.set.pitch3d && up.y < 0.0) {
                why = Some(format!("Over-banked {}°", to_fixed(tilt, 0)));
            } else if alt < 8.0 && vy < -2.2 && alt / -vy < 1.4 {
                why = Some(format!("Sinking {} m/s at {} m", to_fixed(-vy, 1), to_fixed(alt, 0)));
            } else if let Some(w) = self.ground_prox() {
                why = Some(w);
            } else if !h.hold && vrs_of(-vu, ip, h.coll_cmd, est.rpm) > 0.5 {
                why = Some("Settling with power".into());
            } else if est.rpm < 0.72 {
                why = Some(format!("Headspeed {}%", pct(est.rpm)));
            }
        }
        if let Some(w) = why {
            self.start_rescue(w);
        }
    }

    pub(crate) fn check_battery(&mut self, dt: f64) {
        let h = &mut self.h;
        if !h.armed || h.crashed {
            return;
        }
        if h.grounded {
            h.fs15 = false;
            h.fs8 = false;
            h.fs_t15 = 0.0;
            h.fs_t8 = 0.0;
            return;
        }
        // 2 s debounce against momentary sag
        h.fs_t15 = if self.est.bat <= 0.15 { h.fs_t15 + dt } else { 0.0 };
        h.fs_t8 = if self.est.bat <= 0.08 { h.fs_t8 + dt } else { 0.0 };
        if h.air_t < 0.5 {
            return;
        }
        let m = self.auto.as_ref().map(|a| a.mode);
        if m == Some(Mode::Rescue) || self.auto.as_ref().is_some_and(|a| a.ar_unpowered()) {
            return;
        }
        // never mid-manoeuvre: the failsafe waits until the stunt has recovered (timers keep running)
        if self.auto.as_ref().is_some_and(|a| a.mode == Mode::Stunt && a.st.as_ref().is_some_and(|s| s.phase as u8 >= crate::stunt::SPhase::Man as u8)) {
            return;
        }
        if !h.fs8 && h.fs_t8 >= 2.0 {
            h.fs8 = true;
            h.fs15 = true;
            if m == Some(Mode::Land) {
                let a = self.auto.as_mut().unwrap();
                a.forced = true;
                a.fs = true;
            } else {
                self.start_auto(Mode::Land, true, true, true, true);
            }
            if self.auto.is_some() {
                self.toast("Battery critical (8%)", "Forced landing on the pad", "bad");
                self.h.fs_at = Some(FsAt { t: self.sim_t, bat: self.h.bat, est: self.est.bat, stage: 8 });
            }
        } else if !h.fs15 && h.fs_t15 >= 2.0 && m != Some(Mode::Land) {
            h.fs15 = true;
            self.start_auto(Mode::Land, true, false, true, true);
            if self.auto.is_some() {
                self.toast("Battery low (15%)", "Landing on the pad", "bad");
                self.h.fs_at = Some(FsAt { t: self.sim_t, bat: self.h.bat, est: self.est.bat, stage: 15 });
            }
        }
    }

    /// Collective that gives the wanted vertical acceleration, from the estimated state.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn coll_for(&self, vz_des: f64, up: V3, rx: f64, ry: f64, rz: f64, dk: f64, kz: f64, ff_a: f64) -> f64 {
        let sp2 = rx * rx + ry * ry + rz * rz;
        let vu = rx * up.x + ry * up.y + rz * up.z;
        let ip = (sp2 - vu * vu).max(0.0).sqrt();
        let rpm = self.est.rpm.max(0.3);
        let sev = vrs_of(-vu, ip, self.h.coll_cmd, rpm);
        let az_des = kz * (vz_des - self.est.v.y) + ff_a + G + dk * ry + damp_of(rpm, ip, sev) * vu * up.y;
        let t = az_des / up.y.max(0.5);
        clamp(
            t / (TMAX * rpm * rpm * ge_of(self.est.p.y, up.y) * eta_of(ip) * (1.0 - 0.45 * sev)),
            if self.set.pitch3d { -1.0 } else { 0.0 },
            1.0,
        )
    }

    fn low_coll(&self) -> Cmd {
        Cmd { coll: if self.set.pitch3d { -1.0 } else { 0.0 }, ..Cmd::default() }
    }

    fn finish_autorot(&mut self, a: Auto) -> Cmd {
        let vs = self.h.last_impact;
        let d_pad = pad_dist(self.h.p.x, self.h.p.z);
        self.h.auto_result = Some(AutoResult {
            vs,
            d_pad,
            drift: self.h.last_touch_hs,
            min_rpm: a.min_rpm,
            max_rpm: a.max_rpm,
            cut_alt: a.cut_alt,
            glide_n: a.glide_n,
            glide_vy: a.glide_vy,
            glide_hs: a.glide_hs,
        });
        self.sticks.thr = 0.0;
        self.auto = None;
        self.auto_out = None;
        let g = self.h.last_touch.as_ref().map(|t| t.grade).unwrap_or("Hard");
        self.toast(
            format!("{g} autorotation landing"),
            format!(
                "Touchdown {} m/s · {} m from the pad centre · min headspeed {}%",
                to_fixed(vs, 2),
                to_fixed(d_pad, 1),
                round_str(a.min_rpm * 100.0)
            ),
            if vs < 1.5 { "good" } else { "" },
        );
        self.low_coll()
    }

    /// One autopilot step. The active mode is in `self.auto` (must be Some).
    pub(crate) fn auto_step(&mut self, dt: f64) -> Cmd {
        let mut a = self.auto.take().expect("auto_step without autopilot");
        a.t += dt;
        let alt = self.est.alt;
        let out = Cmd::default();
        if a.ar_unpowered() && self.h.grounded {
            return self.finish_autorot(a);
        }
        if self.est.rpm < 0.97 && self.h.grounded {
            self.auto = Some(a);
            return out; // wait for headspeed
        }
        let est_q = self.est.q;
        let up = up_of(est_q);
        let (mut tx, mut tz, mut alt_t, mut vmax) = (a.anchor.x, a.anchor.z, a.alt, 2.5);
        let mut vz_des: Option<f64> = None;
        let mut vd: Option<(f64, f64)> = None;
        let (mut ffx, mut ffz, mut ff_yaw, mut kv) = (0.0, 0.0, 0.0, 1.6);
        let mut coll_ov: Option<f64> = None;
        let mut lim = 24.0 * DEG;
        let mut kp = 0.9;
        if a.mode == Mode::Takeoff && alt > a.alt - 0.3 {
            a.mode = Mode::Hold;
            self.toast(
                "Hover hold",
                format!("Holding {} m. Touch a stick to fly, or tap CIRCUIT / LAND.", to_fixed(a.alt, 0)),
                "good",
            );
        }
        if a.mode == Mode::Circuit && (a.li >= 0 || alt > a.alt - 0.6) {
            let f = self.loop_follow(&mut a);
            vd = Some((f.vx, f.vz));
            ffx = f.ax;
            ffz = f.az;
            ff_yaw = f.yaw_rate;
            a.heading = f.heading;
            lim = 30.0 * DEG;
        }
        if a.mode == Mode::Land {
            use land::*;
            // the flown target: camera while it tracks the tag, GPS otherwise; a switch slews
            let (want, cam) = self.pad_target(&a);
            a.cam_tgt = cam;
            if !a.tgt_init {
                a.tgt = want;
                a.tgt_init = true;
            } else {
                let dd = want - a.tgt;
                let st = TGT_SLEW * dt;
                a.tgt = if dd.length() <= st { want } else { a.tgt + dd * (st / dd.length()) };
            }
            a.anchor = a.tgt;
            tx = a.tgt.x;
            tz = a.tgt.z;
            let e = (a.tgt.x - self.est.p.x).hypot(a.tgt.z - self.est.p.z);
            let hs = self.est.v.hyp_xz();
            if e > R_APP {
                let h = cruise_height(e, a.land_h).min(a.land_cap).max(a.land_h);
                let tr = self.transit(&mut a, tx, tz, h);
                vd = Some(tr.vd);
                ffx = tr.ff.0;
                ffz = tr.ff.1;
                alt_t = tr.alt_t;
                lim = LIM * DEG;
                if tr.climbing {
                    vz_des = Some(clamp(1.2 * (alt_t - alt), -1.5, CLIMB_VZ));
                }
            } else {
                // gentle approach and final hover: nose to the landing heading once slow
                alt_t = a.land_h.min(a.land_cap.max(a.land_h)).max(if a.descending { 0.0 } else { a.land_h });
                if hs < YAW_V - 0.5 {
                    a.track_yaw = false;
                }
                if !a.track_yaw {
                    a.heading = self.land_heading();
                }
                if a.descending && e > E_PAUSE && alt > 1.0 {
                    // target moved (camera lost / reacquired): hold this height and line up again
                    a.descending = false;
                    a.land_h = alt;
                    alt_t = alt;
                }
            }
            let e_ok = if cam { E_DESC_CAM } else { E_DESC };
            let hdg_ok = wrap(a.heading - heading_of(est_q)).abs() < HDG_DESC * DEG;
            if !a.descending && e < e_ok && hs < HS_DESC && hdg_ok && (alt - a.land_h).abs() < 0.6 {
                a.descending = true;
            }
            if a.descending {
                vz_des = Some(-(0.45 * alt).min(1.1).max(0.35));
                // stiffer position hold over the pad in the descent (gusts near the ground)
                kp = KP_DESC;
                vmax = 1.0;
                kv = KV_DESC;
            }
            if self.h.grounded && a.descending {
                // on the skids: lower collective and finish
                self.sticks.thr = 0.0;
                self.auto = None;
                self.auto_out = None;
                return self.low_coll();
            }
        }
        if a.mode == Mode::Rescue {
            a.min_alt = a.min_alt.min(alt);
            a.alt = (a.min_alt + 2.0).max(3.0);
            vd = Some((0.0, 0.0));
            let tilt = clamp(up.y, -1.0, 1.0).acos() / DEG;
            if up.y < 0.5 {
                // roll upright first
                coll_ov = Some(if self.set.pitch3d && up.y < 0.0 {
                    -0.5
                } else if up.y > 0.0 {
                    0.35
                } else {
                    0.0
                });
            } else if self.est.v.y < -0.5 && alt / -self.est.v.y < 0.9 {
                coll_ov = Some(1.0); // about to hit: everything it has
            } else {
                vz_des = Some(clamp(1.5 * (a.alt - alt), 0.0, 2.5));
            }
            if a.t > 0.6 && tilt < 10.0 && self.est.v.length() < 1.5 && alt >= a.alt - 0.3 && alt >= 2.7 {
                self.restore_pitch();
                if self.est.bat <= 0.15 {
                    let forced = a.was_forced || self.est.bat <= 0.08;
                    self.start_auto(Mode::Land, true, forced, true, true);
                    self.toast("Recovered", "Battery low. Landing on the pad", "good");
                } else {
                    a.mode = Mode::Hold;
                    a.anchor = v3(self.est.p.x, 0.0, self.est.p.z);
                    a.alt = alt;
                    a.heading = heading_of(est_q);
                    self.auto = Some(a);
                    self.toast("Recovered. Hover hold.", "Touch a stick to fly.", "good");
                }
                return self.auto_step(0.0);
            }
        }
        if a.mode == Mode::Stunt {
            match self.stunt_step(&mut a, dt) {
                crate::stunt::StuntOut::Direct(mut c) => {
                    if let Some((t0, cc)) = self.stunt_hook.cut_coll {
                        if a.st.as_ref().is_some_and(|s| s.t_man >= t0) {
                            c.coll = cc;
                        }
                    }
                    self.auto = Some(a);
                    return c;
                }
                crate::stunt::StuntOut::Guide(g) => {
                    tx = g.tx;
                    tz = g.tz;
                    alt_t = g.alt_t;
                    vmax = g.vmax;
                    vd = g.vd;
                    lim = g.lim;
                    if let (Some((t0, cc)), Some(s)) = (self.stunt_hook.cut_coll, a.st.as_ref()) {
                        if s.t_man >= t0 && s.phase as u8 >= crate::stunt::SPhase::Man as u8 {
                            coll_ov = Some(cc);
                        }
                    }
                }
            }
        }
        if a.mode == Mode::Autorot {
            let (p, _) = self.pad_target(&a);
            let (dxp, dzp) = (p.x - self.est.p.x, p.z - self.est.p.z);
            let d = dxp.hypot(dzp);
            let hs = self.est.v.hyp_xz();
            let cmin = if self.set.pitch3d { -0.3 } else { 0.0 };
            if a.phase == Phase::Climb || a.phase == Phase::Run {
                a.min_rpm = 9.0;
            }
            if a.phase == Phase::Climb {
                tx = a.entry.x;
                tz = a.entry.z;
                alt_t = a.entry_alt;
                vmax = 5.0;
                let de = (tx - self.est.p.x).hypot(tz - self.est.p.z);
                if de > land::R_APP {
                    // same transit as LAND: climb clear of the trees first, then fast to the entry
                    let h_tr = a.entry_alt.min(land::H_FAR);
                    let tr = self.transit(&mut a, tx, tz, h_tr);
                    vd = Some(tr.vd);
                    ffx = tr.ff.0;
                    ffz = tr.ff.1;
                    lim = land::LIM * DEG;
                    alt_t = a.entry_alt;
                    if tr.climbing {
                        vz_des = Some(clamp(1.2 * (a.entry_alt - alt), -1.5, land::CLIMB_VZ));
                    }
                }
                if de < 10.0 {
                    a.heading = (-(p.x - self.est.p.x)).atan2(-(p.z - self.est.p.z));
                }
                if de < 2.5 && alt > a.entry_alt - 1.5 && hs < 1.5 && wrap(a.heading - heading_of(est_q)).abs() < 0.25 {
                    a.phase = Phase::Run;
                }
            } else if a.phase == Phase::Run {
                // powered run-in at glide speed: autos are entered with airspeed
                let (ux, uz) = (dxp / d, dzp / d);
                alt_t = a.entry_alt;
                vd = Some((ux * ar::RUN_V, uz * ar::RUN_V));
                a.heading = (-ux).atan2(-uz);
                if d < ar::ENTRY_D + 1.0 && hs > ar::RUN_V - 1.5 {
                    a.phase = Phase::Glide;
                    self.set_hold(true, true);
                    a.coll = ar::COLL;
                    a.cut_alt = alt;
                    a.min_rpm = self.est.rpm;
                    a.max_rpm = self.est.rpm;
                    a.glide_n = 0;
                    a.glide_vy = 0.0;
                    a.glide_hs = 0.0;
                    self.toast("Autorotation", "Motor cut. Gliding to the pad.", "");
                }
            }
            if a.ar_unpowered() {
                a.min_rpm = a.min_rpm.min(self.est.rpm);
                a.max_rpm = a.max_rpm.max(self.est.rpm);
                let (ux, uz) = if d > 0.01 { (dxp / d, dzp / d) } else { (0.0, 0.0) };
                let est = &self.est;
                match a.phase {
                    Phase::Glide => {
                        let tgo = ((alt - ar::FLARE_ALT) / (-est.v.y).max(1.5)).max(0.5);
                        let vf = clamp(
                            (d - ar::FLARE_D) / tgo + est.wind.dot(v3(ux, 0.0, uz)),
                            if d > ar::FLARE_D + 2.0 { 6.0 } else { 0.0 },
                            8.0,
                        );
                        vd = Some((ux * vf, uz * vf));
                        if d > 3.0 {
                            a.heading = (-ux).atan2(-uz);
                        }
                        lim = 32.0 * DEG;
                        a.coll = clamp(a.coll + 1.5 * (est.rpm - 0.99) * dt, cmin, 0.45);
                        coll_ov = Some(a.coll);
                        if a.t > 0.0 && alt < a.cut_alt - 6.0 && alt > ar::FLARE_ALT + 2.0 {
                            a.glide_n += 1;
                            a.glide_vy += est.v.y;
                            a.glide_hs += hs;
                        }
                        if alt < ar::FLARE_ALT {
                            a.phase = Phase::Flare;
                        }
                    }
                    Phase::Flare => {
                        vd = Some((clamp(0.4 * dxp, -1.5, 1.5), clamp(0.4 * dzp, -1.5, 1.5)));
                        kv = 1.3;
                        lim = 30.0 * DEG;
                        a.coll = clamp(a.coll + 2.5 * (est.rpm - 1.0) * dt, cmin, 0.6);
                        coll_ov = Some(a.coll);
                        // start the cushion at the stopping distance for a steady deceleration
                        if est.v.y < 0.0 && alt < est.v.y * est.v.y / (2.0 * ar::CUSH_A) + 0.6 {
                            a.phase = Phase::Cushion;
                        }
                    }
                    _ => {
                        let kp = clamp((alt - 0.4) / 2.0, 0.0, 1.0) * 0.5; // steer onto the pad, fading near the ground
                        vd = Some((clamp(kp * dxp, -2.5, 2.5), clamp(kp * dzp, -2.5, 2.5)));
                        lim = (if alt > 1.2 { 15.0 } else { 8.0 }) * DEG;
                        let vz = -(2.0 * ar::CUSH_A * (alt - 0.25).max(0.0)).sqrt().max(0.4);
                        vz_des = Some(vz);
                        let (wx, wz) = (est.wind.x, est.wind.z);
                        let dk = 0.22 + 0.035 * est.v.length();
                        coll_ov = Some(self.coll_for(
                            vz,
                            up,
                            est.v.x - wx,
                            est.v.y,
                            est.v.z - wz,
                            dk,
                            5.0,
                            if vz < -0.5 { ar::CUSH_A } else { 0.0 },
                        ));
                    }
                }
            }
        }
        let est = &self.est;
        // horizontal: position -> velocity -> acceleration -> tilt
        let (vdx, vdz) = vd.unwrap_or_else(|| {
            let (mut x, mut z) = (kp * (tx - est.p.x), kp * (tz - est.p.z));
            let vm = x.hypot(z);
            if vm > vmax {
                x *= vmax / vm;
                z *= vmax / vm;
            }
            (x, z)
        });
        let w = est.wind;
        let (rx, rz, ry) = (est.v.x - w.x, est.v.z - w.z, est.v.y);
        let sp = (rx * rx + ry * ry + rz * rz).sqrt();
        let dk = 0.22 + 0.035 * sp;
        let ax = kv * (vdx - est.v.x) + dk * rx + ffx;
        let az = kv * (vdz - est.v.z) + dk * rz + ffz;
        let (fwd, right) = fwd_right(est_q);
        let max_a = 32.0 * PI / 180.0;
        let pa = clamp(((ax * fwd.x + az * fwd.z) / G).atan(), -lim, lim);
        let ra = clamp(((ax * right.x + az * right.z) / G).atan(), -lim, lim);
        let mut out = Cmd { pit: pa / max_a, rol: ra / max_a, ..out };
        // vertical
        out.coll = match coll_ov {
            Some(c) => c,
            None => {
                let vz = vz_des.unwrap_or_else(|| clamp(1.2 * (alt_t - alt), -1.5, 2.0));
                let c = self.coll_for(vz, up, rx, ry, rz, dk, 2.5, 0.0);
                if a.mode == Mode::Autorot {
                    c.max(a.coll)
                } else {
                    c
                }
            }
        };
        // yaw: hold or turn toward the commanded heading, with the path's turn rate fed forward
        out.yaw = clamp(-(2.0 * wrap(a.heading - heading_of(est_q)) + ff_yaw) / 4.5, -0.8, 0.8);
        if a.mode == Mode::Rescue {
            if up.y < 0.5 {
                out.rate = Some(self.rescue_rate(&mut a));
            } else {
                a.resc_axis = None;
            }
        }
        self.auto = Some(a);
        out
    }
}
