//! World state and the 240 Hz step: battery failsafe, crash protection, autopilot, physics,
//! sensors. Also the user commands (buttons/keys), landing grading, toasts as an event list and
//! the HUD/cue text, so the page only renders what this module decides.

use crate::autopilot::{Auto, Mode, Phase};
use crate::heli::{Heli, PADS, PAD_R};
use crate::jsonw::{round_str, to_fixed};
use crate::math::*;
use crate::rng::Rng;
use crate::sensors::{Est, Sensors};
use crate::units::*;
use crate::wind::WINDS;

pub const DT: f64 = 1.0 / 240.0;
pub const CAMS: [&str; 3] = ["pilot", "chase", "pad"];

#[derive(Clone, Copy, Debug)]
pub struct Settings {
    pub level: bool,
    pub pitch3d: bool,
    pub wind: usize,
    pub cam: usize,
    pub protect: bool,
    pub real: bool,
    /// REAL mode estimator: true = EKF, false = BASIC complementary filter
    pub ekf: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Sticks {
    pub thr: f64,
    pub yaw: f64,
    pub pit: f64,
    pub rol: f64,
}

/// Autopilot output in self-level stick units plus a raw collective.
#[derive(Clone, Copy, Debug, Default)]
pub struct Cmd {
    pub coll: f64,
    pub yaw: f64,
    pub pit: f64,
    pub rol: f64,
    /// body-rate demand (rad/s, body axes) flown directly by the FBL rate loop instead of the
    /// self-level tilt in `pit` / `rol` (stunts)
    pub rate: Option<V3>,
}

#[derive(Clone, Debug)]
pub struct Toast {
    pub title: String,
    pub sub: String,
    pub kind: &'static str,
}

/// One frame of pilot input, gathered by the page (or a test harness).
#[derive(Clone, Copy, Debug, Default)]
pub struct Input {
    /// yaw / pitch / roll from touch sticks and keys, -1..1, before the gamepad
    pub yaw: f64,
    pub pit: f64,
    pub rol: f64,
    /// throttle change from the W/S keys this frame (0.45 per second while held)
    pub thr_delta: f64,
    /// absolute throttle from the touch stick this frame, if it moved
    pub thr_set: Option<f64>,
    /// first gamepad with at least 4 axes: axes 0..3
    pub gp: Option<[f64; 4]>,
}

/// User commands (buttons and keys). Codes are the wasm `sim_cmd` argument.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Arm = 0,
    Hold = 1,
    Protect = 2,
    Sensors = 3,
    Level = 4,
    Pitch = 5,
    Wind = 6,
    Cam = 7,
    Reset = 8,
    Takeoff = 9,
    Circuit = 10,
    Land = 11,
    Autorot = 12,
    /// a stick was touched or a stick key pressed: hand control back unless the phase is hands-off
    Takeover = 13,
    /// toggle the REAL-mode estimator EKF / BASIC
    Estimator = 14,
    /// open / close the stunt picker
    Stunts = 15,
    /// fly a stunt: 16 pirouette, 17 loop, 18 roll, 19 flip, 20 inverted hover
    StuntPirouette = 16,
    StuntLoop = 17,
    StuntRoll = 18,
    StuntFlip = 19,
    StuntInverted = 20,
    /// close the stunt picker
    PickerClose = 21,
}

impl Command {
    pub fn from_code(c: u32) -> Option<Command> {
        use Command::*;
        Some(match c {
            0 => Arm,
            1 => Hold,
            2 => Protect,
            3 => Sensors,
            4 => Level,
            5 => Pitch,
            6 => Wind,
            7 => Cam,
            8 => Reset,
            9 => Takeoff,
            10 => Circuit,
            11 => Land,
            12 => Autorot,
            13 => Takeover,
            14 => Estimator,
            15 => Stunts,
            16 => StuntPirouette,
            17 => StuntLoop,
            18 => StuntRoll,
            19 => StuntFlip,
            20 => StuntInverted,
            21 => PickerClose,
            _ => return None,
        })
    }
}

#[derive(Clone, Debug)]
pub struct LastTouch {
    pub vs: f64,
    pub grade: &'static str,
    pub pad: Option<usize>,
}

#[derive(Clone, Debug)]
pub struct AutoResult {
    pub vs: f64,
    pub d_pad: f64,
    pub drift: f64,
    pub min_rpm: f64,
    pub max_rpm: f64,
    pub cut_alt: f64,
    pub glide_n: u32,
    pub glide_vy: f64,
    pub glide_hs: f64,
}

#[derive(Clone, Debug)]
pub struct FsAt {
    pub t: f64,
    pub bat: f64,
    pub est: f64,
    pub stage: u32,
}

pub struct Sim {
    pub h: Heli,
    pub est: Est,
    pub sens: Sensors,
    pub set: Settings,
    pub sticks: Sticks,
    pub auto: Option<Auto>,
    pub auto_out: Option<Cmd>,
    pub sim_t: f64,
    pub rescues: u32,
    pub landings: u32,
    pub best: Option<f64>,
    pub goal_b: bool,
    pub gp_seen: bool,
    pub rng: Rng,
    /// toasts raised since the page last read a frame
    pub toasts: Vec<Toast>,
    pub toast_clear: bool,
    /// most recent toast, for tests
    pub last_toast: Option<Toast>,
    pub toast_count: u64,
    /// wind vector used by the last physics step
    pub wind: V3,
    /// stunt picker open on the page
    pub picker: bool,
    /// the user's pitch curve while a stunt has switched to 3D (restored afterwards)
    pub stunt_p3d: Option<bool>,
    /// test hooks for deliberately broken stunts
    pub stunt_hook: crate::stunt::StuntHook,
    pub stunt_best: [Option<u32>; 5],
    pub stunt_last: Option<crate::stunt::StuntRes>,
    pub stunt_count: u32,
}

impl Sim {
    pub fn new(seed: u64) -> Sim {
        let mut s = Sim {
            h: Heli::new(),
            est: Est::new(),
            sens: Sensors::new(),
            set: Settings { level: true, pitch3d: false, wind: 1, cam: 0, protect: true, real: false, ekf: true },
            sticks: Sticks::default(),
            auto: None,
            auto_out: None,
            sim_t: 0.0,
            rescues: 0,
            landings: 0,
            best: None,
            goal_b: false,
            gp_seen: false,
            rng: Rng::new(seed),
            toasts: Vec::new(),
            toast_clear: false,
            last_toast: None,
            toast_count: 0,
            wind: V3::ZERO,
            picker: false,
            stunt_p3d: None,
            stunt_hook: crate::stunt::StuntHook::default(),
            stunt_best: [None; 5],
            stunt_last: None,
            stunt_count: 0,
        };
        s.reset_heli();
        s
    }

    pub fn reset_heli(&mut self) {
        self.h = Heli::new();
        self.sticks.thr = 0.0;
        self.sensors_reset();
    }

    pub fn toast(&mut self, title: impl Into<String>, sub: impl Into<String>, kind: &'static str) {
        let t = Toast { title: title.into(), sub: sub.into(), kind };
        self.toasts.push(t.clone());
        self.last_toast = Some(t);
        self.toast_count += 1;
    }

    // ---------- input ----------
    /// Port of readInputs: keys ramp the throttle, touch sets it, a gamepad overrides both.
    pub fn input(&mut self, inp: &Input) {
        if let Some(t) = inp.thr_set {
            self.sticks.thr = clamp(t, 0.0, 1.0);
        }
        self.sticks.thr = clamp(self.sticks.thr + inp.thr_delta, 0.0, 1.0);
        let (mut yaw, mut rol, mut pit) = (inp.yaw, inp.rol, inp.pit);
        if let Some(ax) = inp.gp {
            let dz = |a: f64| if a.abs() < 0.06 { 0.0 } else { a };
            if !self.gp_seen && ax.iter().any(|a| a.abs() > 0.2) {
                self.gp_seen = true;
                self.toast("Controller connected", "Left stick: throttle/yaw · right stick: cyclic", "");
            }
            if self.gp_seen && self.auto.is_some() && dz(ax[0]).abs() + dz(ax[2]).abs() + dz(ax[3]).abs() > 0.25 {
                self.cancel_auto(true);
            }
            if self.gp_seen && self.auto.is_none() {
                let or = |a: f64, b: f64| if a != 0.0 { a } else { b };
                yaw = or(dz(ax[0]), yaw);
                self.sticks.thr = (1.0 - ax[1]) / 2.0;
                rol = or(dz(ax[2]), rol);
                pit = or(-dz(ax[3]), pit);
            }
        }
        self.sticks.yaw = yaw;
        self.sticks.rol = rol;
        self.sticks.pit = pit;
    }

    /// What the stick gimbals on screen should show: (left x, left y, right x, right y).
    pub fn stick_display(&self) -> (f64, f64, f64, f64) {
        let (yaw, rol, pit) = match (&self.auto, self.auto_out) {
            (Some(_), Some(o)) => (o.yaw, o.rol, o.pit),
            _ => (self.sticks.yaw, self.sticks.rol, self.sticks.pit),
        };
        (yaw, self.sticks.thr * 2.0 - 1.0, rol, pit)
    }

    // ---------- commands ----------
    pub fn command(&mut self, c: Command) {
        use Command::*;
        match c {
            Arm => self.toggle_arm(),
            Hold => {
                if !self.h.crashed {
                    self.cancel_auto(false);
                    let on = !self.h.hold;
                    self.set_hold(on, false);
                }
            }
            Protect => self.set.protect = !self.set.protect,
            Sensors => {
                self.set.real = !self.set.real;
                if self.set.real {
                    self.toast(
                        "Real sensors",
                        "The flight controller now flies on noisy, delayed sensor data. The blue ghost shows what it believes.",
                        "",
                    );
                }
            }
            Level => self.set.level = !self.set.level,
            Pitch => {
                if self.h.armed && !self.h.grounded {
                    self.toast("Land first", "Change the pitch curve on the ground", "");
                } else {
                    self.set.pitch3d = !self.set.pitch3d;
                    if self.set.pitch3d {
                        self.toast("3D pitch curve", "Mid-stick is zero thrust. Hover is about 73%. Low stick pushes down.", "");
                    }
                }
            }
            Wind => self.set.wind = (self.set.wind + 1) % WINDS.len(),
            Cam => self.set.cam = (self.set.cam + 1) % CAMS.len(),
            Reset => {
                self.cancel_auto(false);
                if let Some(p) = self.stunt_p3d.take() {
                    self.set.pitch3d = p;
                }
                self.picker = false;
                self.reset_heli();
                self.goal_b = false;
                self.toast_clear = true;
            }
            Takeoff => self.start_auto(Mode::Takeoff, false, false, false, false),
            Circuit => self.start_auto(Mode::Circuit, false, false, false, false),
            Land => self.start_auto(Mode::Land, false, false, false, false),
            Autorot => self.start_auto(Mode::Autorot, false, false, false, false),
            Takeover => {
                self.picker = false;
                self.cancel_auto(true)
            }
            Stunts => self.picker = !self.picker,
            PickerClose => self.picker = false,
            StuntPirouette | StuntLoop | StuntRoll | StuntFlip | StuntInverted => {
                if let Some(k) = crate::stunt::Kind::from_idx(c as usize - StuntPirouette as usize) {
                    self.start_stunt(k);
                }
            }
            Estimator => {
                self.set.ekf = !self.set.ekf;
                if self.set.real {
                    self.toast(
                        if self.set.ekf { "Estimator: EKF" } else { "Estimator: BASIC" },
                        if self.set.ekf {
                            "Kalman filter: weighs every sensor by its noise and rejects GPS glitches."
                        } else {
                            "Complementary filter: fixed blending gains, soft GPS gate."
                        },
                        "",
                    );
                }
            }
        }
    }

    fn toggle_arm(&mut self) {
        if self.h.crashed {
            self.toast("Crashed", "Tap RESET to put a fresh heli on Pad A", "bad");
            return;
        }
        if self.h.armed {
            self.h.armed = false;
            self.cancel_auto(false);
        } else {
            if self.sticks.thr > 0.08 {
                self.toast("Throttle not low", "Pull the throttle stick fully down before arming", "bad");
                return;
            }
            if self.h.bat <= 0.0 {
                self.toast("Battery flat", "Tap RESET for a fresh pack", "bad");
                return;
            }
            self.h.armed = true;
            self.h.soft_start = self.h.rpm < 0.5;
        }
    }

    pub fn set_hold(&mut self, on: bool, quiet: bool) {
        if on == self.h.hold {
            return;
        }
        self.h.hold = on;
        if !on {
            // released in the air: bailout at full power
            self.h.soft_start = (self.h.grounded || self.h.rpm <= 0.3) && self.h.rpm < 0.5;
        }
        if !quiet {
            let g = self.h.grounded;
            let sub = if on {
                if g { "Motor off. The rotor coasts down." } else { "Motor off. Lower collective to keep headspeed." }
            } else if g {
                "Governor back on"
            } else {
                "Bailout: governor back on at full power"
            };
            self.toast(if on { "Throttle hold ON" } else { "Throttle hold off" }, sub, if on { "bad" } else { "" });
        }
    }

    // ---------- the 240 Hz step ----------
    pub fn step(&mut self, dt: f64) {
        self.sim_t += dt;
        if self.h.crashed && self.auto.is_some() {
            self.cancel_auto(false);
        }
        self.check_battery(dt);
        self.check_protect();
        let mut cmd = None;
        if self.auto.is_some() {
            let c = self.auto_step(dt);
            self.auto_out = if self.auto.is_some() { Some(c) } else { None };
            if self.auto.is_some() {
                self.sticks.thr = if self.set.pitch3d { (c.coll + 1.0) / 2.0 } else { c.coll };
            }
            cmd = Some(c);
        }
        let wind = self.physics(cmd, dt);
        self.wind = wind;
        self.sensors_update(dt, wind);
    }

    pub fn steps(&mut self, n: u32) {
        for _ in 0..n {
            self.step(DT);
        }
    }

    // ---------- landing record ----------
    pub fn landed(&mut self, vs: f64) {
        self.landings += 1;
        let mut pad = None;
        for (i, p) in PADS.iter().enumerate() {
            if (self.h.p.x - p.x).hypot(self.h.p.z - p.z) <= PAD_R {
                pad = Some(i);
            }
        }
        let grade = grade_of(vs);
        if pad.is_some() && self.best.map_or(true, |b| vs < b) {
            self.best = Some(vs);
        }
        let wher = match pad {
            Some(i) => format!("on Pad {}", PADS[i].name),
            None => format!("off the pads ({})", self.nearest_text()),
        };
        self.h.last_touch = Some(LastTouch { vs, grade, pad });
        let good = pad.is_some() && vs < 1.0;
        self.toast(format!("{grade} landing {wher}"), format!("Touchdown {} m/s", to_fixed(vs, 2)), if good { "good" } else { "" });
        if pad == Some(1) && vs < 1.0 {
            self.goal_b = true;
        }
    }

    fn nearest_text(&self) -> String {
        let (mut d, mut n) = (f64::INFINITY, "");
        for p in PADS.iter() {
            let dd = (self.h.p.x - p.x).hypot(self.h.p.z - p.z);
            if dd < d {
                d = dd;
                n = p.name;
            }
        }
        format!("{} m from Pad {}", to_fixed(d - PAD_R, 1), n)
    }

    // ---------- HUD ----------
    pub fn hud(&self) -> Hud {
        let h = &self.h;
        let est = &self.est;
        let r = est.rpm;
        let rpm_cls = if h.armed {
            if r < 0.7 {
                "bad"
            } else if r < 0.85 || r > 1.08 {
                "warn"
            } else {
                "ok"
            }
        } else {
            ""
        };
        let alt = est.alt.max(0.0);
        let vy = est.v.y;
        let vs_cls = if alt < 4.0 && vy < -3.0 {
            "bad"
        } else if alt < 4.0 && vy < -1.0 {
            "warn"
        } else {
            ""
        };
        let mut bat = format!("{}%", round_str(est.bat * 100.0));
        if self.set.real {
            bat.push_str(&format!(" · {} V", to_fixed(est.bat_v, 1)));
        }
        let bat_cls = if est.bat < 0.2 {
            "bad"
        } else if est.bat < 0.35 {
            "warn"
        } else {
            ""
        };
        let ft = h.flight_t;
        let secs = (ft % 60.0).floor() as u64;
        let time = format!("{}:{}{}", round_str((ft / 60.0).floor()), if secs < 10 { "0" } else { "" }, secs);
        Hud {
            thr: format!("{}%", round_str(self.sticks.thr * 100.0)),
            rpm: format!("{} rpm", round_str(r * RPM_FULL)),
            rpm_cls,
            alt: format!("{} m", to_fixed(alt, 1)),
            vs: format!("{}{} m/s", if vy >= 0.0 { "+" } else { "" }, to_fixed(vy, 1)),
            vs_cls,
            spd: format!("{} m/s", to_fixed(est.v.hyp_xz(), 1)),
            bat,
            bat_cls,
            land: format!(
                "{} · best {}",
                self.landings,
                match self.best {
                    None => "—".to_string(),
                    Some(b) => format!("{} m/s", to_fixed(b, 2)),
                }
            ),
            time,
            resc: self.rescues.to_string(),
            cue: self.cue(),
            sens: if self.set.real { self.sens_html() } else { String::new() },
        }
    }

    fn sens_html(&self) -> String {
        let st = &self.sens.st;
        let (h, est) = (&self.h, &self.est);
        let up_t = up_of(h.q);
        let up_e = up_of(est.q);
        let tilt_err = deg(clamp(up_t.dot(up_e), -1.0, 1.0).acos());
        let hdg_err = deg(wrap(heading_of(est.q) - heading_of(h.q)).abs());
        let dy = est.p.y - h.p.y;
        let mut s = String::new();
        s += &format!("<span>POS ERR <b>{} m</b></span>", to_fixed((est.p.x - h.p.x).hypot(est.p.z - h.p.z), 2));
        s += &format!("<span>ALT ERR <b>{}{} m</b></span>", if dy >= 0.0 { "+" } else { "" }, to_fixed(dy, 2));
        s += &format!("<span>TILT ERR <b>{}°</b></span><span>HDG ERR <b>{}°</b></span>", to_fixed(tilt_err, 1), to_fixed(hdg_err, 1));
        s += &if st.gps_lost {
            if up_t.y < 0.0 { "<span class=\"bad\">GPS LOST (inverted)</span>".to_string() } else { "<span class=\"bad\">GPS LOST</span>".to_string() }
        } else if st.gps_reacq {
            format!("<span class=\"warn\">GPS REACQUIRING {} sats</span>", st.sats)
        } else if st.glitch {
            "<span class=\"bad\">GPS GLITCH</span>".to_string()
        } else {
            format!("<span>GPS <b>{} sats</b></span>", st.sats)
        };
        s += &format!("<span>RANGE <b>{}</b></span>", if st.rf { "yes" } else { "no" });
        if st.vib > 1.25 {
            s += &format!("<span class=\"warn\">VIB {}x</span>", to_fixed(st.vib, 1));
        }
        s += &format!("<span>RPM TLM {}</span>", if st.esc { "<b>ok</b>" } else { "<span class=\"bad\">dropped</span>" });
        s += &format!("<span>PACK <b>{} V · {} A</b></span>", to_fixed(st.v, 1), to_fixed(st.a, 0));
        if self.set.ekf {
            s += &self.ekf_html();
        } else {
            s += "<span>EST <b>BASIC</b></span>";
        }
        s
    }

    /// EKF health for the sensor panel: status, innovation test ratios (bars, full = gate),
    /// 1-sigma horizontal position and heading, rejected GPS samples.
    fn ekf_html(&self) -> String {
        use crate::ekf::Status;
        let k = &self.sens.ekf;
        let st = k.status();
        let cls = match st {
            Status::GpsReset => " class=\"bad\"",
            Status::GpsRejected | Status::BaroOnly | Status::Init => " class=\"warn\"",
            _ => "",
        };
        let mut s = format!("<span>EKF <b{cls}>{}</b></span>", st.name());
        let bar = |name: &str, tr: f64| {
            let c = if tr > 1.0 {
                " bad"
            } else if tr > 0.5 {
                " warn"
            } else {
                ""
            };
            let w = clamp(tr, 0.0, 1.0) * 100.0;
            format!("<span>{name}<span class=\"tr{c}\"><i style=\"width:{}%\"></i></span></span>", round_str(w))
        };
        s += "<span class=\"trs\">";
        s += &bar("GPS", k.h.tr_gps_pos.max(k.h.tr_gps_vel));
        s += &bar("BARO", k.h.tr_baro);
        s += &bar("RNG", k.h.tr_rng);
        s += &bar("MAG", k.h.tr_mag);
        s += "</span>";
        s += &format!(
            "<span>1σ <b>{} m · {}°</b></span><span>GPS REJ <b>{}</b></span>",
            to_fixed(k.sig_pos(), 2),
            to_fixed(deg(k.sig_hdg()), 1),
            k.h.rej_gps
        );
        s
    }

    /// Coaching line under the buttons (HTML with <em> highlights).
    pub fn cue(&self) -> String {
        let h = &self.h;
        let est = &self.est;
        let pct = |x: f64| round_str(x * 100.0);
        if h.crashed {
            return "Crashed. Tap <em>RESET</em> to put a fresh heli on Pad A.".into();
        }
        if let Some(a) = &self.auto {
            let take = " Touch any stick to take control.";
            let padn = a.pad.map(|i| PADS[i].name).unwrap_or("A");
            if a.mode == Mode::Rescue {
                return "Crash protection <em>recovering</em>. Hands off for a moment.".into();
            }
            if let (Mode::Stunt, Some(st)) = (a.mode, a.st.as_ref()) {
                return self.stunt_cue(st);
            }
            if a.mode == Mode::Land && a.forced {
                return format!("Battery critical: <em>forced landing</em> on Pad {padn}.");
            }
            if a.mode == Mode::Land && a.fs {
                return format!("Battery low: autopilot <em>landing</em> on Pad {padn}. Touch a stick to take over.");
            }
            if est.rpm < 0.97 && h.grounded {
                return format!("Autopilot: spooling up, <em>{}%</em> headspeed.", pct(est.rpm));
            }
            match a.mode {
                Mode::Takeoff => return format!("Autopilot: <em>climbing</em> to {} m.{take}", to_fixed(a.alt, 0)),
                Mode::Hold => return format!("Autopilot: <em>hover hold</em> at {} m.{take}", to_fixed(a.alt, 0)),
                Mode::Circuit => {
                    return if a.li < 0 {
                        format!("Autopilot: climbing to <em>5 m</em> for the circuit.{take}")
                    } else {
                        format!(
                            "Autopilot: circuit, lap <em>{}</em> at 5 m / 5 m/s. Tap LAND to come in, or touch a stick to take over.",
                            a.laps + 1
                        )
                    }
                }
                Mode::Land => {
                    return format!(
                        "{} Pad {padn}.{take}",
                        if a.descending { "Autopilot: <em>descending</em> onto" } else { "Autopilot: lining up over" }
                    )
                }
                Mode::Autorot => {
                    return match a.phase {
                        Phase::Climb => "Autorotation: <em>climbing</em> to entry height.".into(),
                        Phase::Run => "Autorotation: running in at entry height.".into(),
                        Phase::Glide => format!("Autorotation: motor cut, gliding, headspeed <em>{}%</em>.", pct(est.rpm)),
                        Phase::Flare => "Autorotation: <em>flare</em>.".into(),
                        Phase::Cushion => "Autorotation: <em>cushion</em>.".into(),
                    }
                }
                Mode::Rescue | Mode::Stunt => unreachable!(),
            }
        }
        if !h.armed {
            return if self.sticks.thr > 0.08 {
                "Pull the <em>throttle fully down</em>, then ARM.".into()
            } else if h.bat <= 0.0 {
                "Battery flat. Tap <em>RESET</em>.".into()
            } else {
                "Tap <em>ARM</em> (Space) to spool up the rotor head.".into()
            };
        }
        if h.hold && !h.grounded {
            return format!(
                "Throttle hold: autorotate. Lower collective to keep headspeed (<em>{}%</em>), flare at about 3 m.",
                pct(est.rpm)
            );
        }
        if h.hold {
            return "Throttle hold is on. Tap <em>THR HOLD</em> to spool the motor back up.".into();
        }
        if h.vrs > 0.2 {
            return "Settling with power. <em>Push forward</em> to fly out of your own downwash.".into();
        }
        if est.rpm < 0.97 && h.grounded {
            return format!("Spooling up: <em>{}%</em> headspeed. Keep the throttle low.", pct(est.rpm));
        }
        let hov = if self.set.pitch3d { "73%" } else { "45%" };
        if h.grounded && !h.airborne {
            return format!("Raise throttle slowly. It lifts off at about <em>{hov}</em>.");
        }
        if est.bat < 0.2 {
            return "<em>Low battery</em>. Land now.".into();
        }
        let alt = est.alt;
        if !h.grounded && alt < 3.0 && est.v.y < -1.2 {
            return "Descending fast near the ground. <em>Add throttle</em>.".into();
        }
        if self.goal_b {
            return if self.set.level {
                "Pad B done. Next, try <em>GYRO: RATE</em> for real flybarless handling.".into()
            } else {
                "Rate mode landing done. Try a <em>nose-in</em> hover next.".into()
            };
        }
        if h.grounded {
            return "On the ground. Lift off and fly over to <em style=\"color:var(--cool)\">Pad B</em>.".into();
        }
        "Hover tail-in, then move over to <em style=\"color:var(--cool)\">Pad B</em> and land under 1 m/s.".into()
    }
}

pub fn grade_of(vs: f64) -> &'static str {
    if vs < 0.5 {
        "Butter-soft"
    } else if vs < 1.0 {
        "Soft"
    } else if vs < 2.0 {
        "Firm"
    } else {
        "Hard"
    }
}

pub struct Hud {
    pub thr: String,
    pub rpm: String,
    pub rpm_cls: &'static str,
    pub alt: String,
    pub vs: String,
    pub vs_cls: &'static str,
    pub spd: String,
    pub bat: String,
    pub bat_cls: &'static str,
    pub land: String,
    pub time: String,
    pub resc: String,
    pub cue: String,
    pub sens: String,
}
