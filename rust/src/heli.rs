//! Helicopter rigid body: rotor aerodynamics (thrust, translational lift, settling with power,
//! ground effect, disc damping), powertrain (governor, soft start, rotor inertia, autorotative
//! drive, battery sag), body drag, skid contact with static/kinetic friction, crash detection.
//! Attitude dynamics are delegated to `fbl` (the flybarless gyro).

use crate::math::*;
use crate::sim::{Cmd, LastTouch, Sim};
use crate::units::G;
use crate::wind::{wind_at, WINDS};

/// Max collective thrust per unit mass.
pub const TMAX: f64 = 2.2 * G;
pub const ROTOR_R: f64 = 0.68;
pub const MAST_H: f64 = 0.29;
/// Rotor inertia: divides the rpm rate; governor and soft start scale with it.
pub const ROTOR_I: f64 = 2.0;
pub const PAD_R: f64 = 1.0;
/// Height of the body origin above the ground when sitting on the skids.
pub const REST_Y: f64 = 0.29;

pub struct Pad {
    pub name: &'static str,
    pub x: f64,
    pub z: f64,
}
pub const PADS: [Pad; 2] = [Pad { name: "A", x: 0.0, z: 0.0 }, Pad { name: "B", x: 6.0, z: -6.0 }];
pub const SKIDS: [V3; 4] = [v3(0.14, -0.29, -0.24), v3(-0.14, -0.29, -0.24), v3(0.14, -0.29, 0.26), v3(-0.14, -0.29, 0.26)];

pub fn nearest_pad(x: f64, z: f64) -> usize {
    let (mut best, mut d) = (0, f64::INFINITY);
    for (i, p) in PADS.iter().enumerate() {
        let dd = (x - p.x).hypot(z - p.z);
        if dd < d {
            d = dd;
            best = i;
        }
    }
    best
}

// ---------- rotor aerodynamics shared by physics and flight controller ----------
/// Translational lift: up to +15% thrust once edgewise airspeed passes a walk.
pub fn eta_of(inplane: f64) -> f64 {
    1.0 + 0.15 * smooth01((inplane - 1.5) / 4.5)
}
/// Vortex ring state severity: sinking into your own downwash with pitch pulled and little
/// airspeed. Fades again above ~6.5 m/s of descent (windmill-brake state).
pub fn vrs_of(inflow: f64, inplane: f64, coll: f64, rpm: f64) -> f64 {
    if coll * rpm * rpm <= 0.25 {
        return 0.0;
    }
    clamp((inflow - 2.0) / 2.0, 0.0, 1.0) * clamp((6.5 - inflow) / 2.0, 0.0, 1.0) * clamp(1.0 - inplane / 2.5, 0.0, 1.0)
}
/// Disc damping along the shaft: the inflow term that holds the heli up in autorotation.
pub fn damp_of(rpm: f64, inplane: f64, sev: f64) -> f64 {
    0.55 * rpm * (1.0 - sev) * (1.0 + 0.1 * inplane.min(10.0))
}
/// Ground effect: up to +18% thrust with the disc on the ground, gone at 1.4 m.
pub fn ge_of(py: f64, up_y: f64) -> f64 {
    let r = py + MAST_H * up_y;
    if r < 1.4 {
        1.0 + 0.18 * (1.0 - r.max(0.0) / 1.4)
    } else {
        1.0
    }
}
pub fn rot_drag(r: f64, coll: f64) -> f64 {
    0.4 * r * r * (0.22 + 2.0 * coll * coll) + 0.04 * r
}

#[derive(Clone, Debug)]
pub struct Heli {
    pub p: V3,
    pub v: V3,
    pub q: Quat,
    pub w: V3,
    pub rpm: f64,
    pub armed: bool,
    pub crashed: bool,
    pub grounded: bool,
    pub airborne: bool,
    pub air_t: f64,
    pub flight_t: f64,
    pub bat: f64,
    pub crash_q: Option<Quat>,
    pub hold: bool,
    pub soft_start: bool,
    pub motor: f64,
    pub yaw_i: f64,
    pub eta: f64,
    pub vrs: f64,
    pub vrs_lag: f64,
    pub inflow: f64,
    pub inplane: f64,
    pub vrs_shown: bool,
    pub coll_cmd: f64,
    pub fs15: bool,
    pub fs8: bool,
    pub fs_t15: f64,
    pub fs_t8: f64,
    pub v_prev: V3,
    pub last_impact: f64,
    pub last_touch_hs: f64,
    pub ground_load: f64,
    pub last_touch: Option<LastTouch>,
    pub auto_result: Option<crate::sim::AutoResult>,
    pub fs_at: Option<crate::sim::FsAt>,
    /// rotor disc flapping, swashplate servos and the FBL rate-loop integrators
    pub rotor: crate::fbl::Rotor,
    /// thrust per unit mass along the disc normal, in g (negative = pushing down the shaft)
    pub thrust_g: f64,
}

impl Heli {
    pub fn new() -> Heli {
        Heli {
            p: v3(0.0, REST_Y, 0.0),
            v: V3::ZERO,
            q: Quat::IDENTITY,
            w: V3::ZERO,
            rpm: 0.0,
            armed: false,
            crashed: false,
            grounded: true,
            airborne: false,
            air_t: 0.0,
            flight_t: 0.0,
            bat: 1.0,
            crash_q: None,
            hold: false,
            soft_start: false,
            motor: 0.0,
            yaw_i: 0.0,
            eta: 1.0,
            vrs: 0.0,
            vrs_lag: 0.0,
            inflow: 0.0,
            inplane: 0.0,
            vrs_shown: false,
            coll_cmd: 0.0,
            fs15: false,
            fs8: false,
            fs_t15: 0.0,
            fs_t8: 0.0,
            v_prev: V3::ZERO,
            last_impact: 0.0,
            last_touch_hs: 0.0,
            ground_load: 0.0,
            last_touch: None,
            auto_result: None,
            fs_at: None,
            rotor: crate::fbl::Rotor::default(),
            thrust_g: 0.0,
        }
    }
    pub fn alt(&self) -> f64 {
        self.p.y - REST_Y
    }
    pub fn tilt_deg(&self) -> f64 {
        crate::units::deg(clamp(up_of(self.q).y, -1.0, 1.0).acos())
    }
}

impl Default for Heli {
    fn default() -> Self {
        Heli::new()
    }
}

impl Sim {
    fn crash(&mut self, reason: String) {
        if self.h.crashed {
            return;
        }
        self.h.crashed = true;
        self.h.armed = false;
        let (fwd, _) = fwd_right(self.h.q);
        let heading = (-fwd.x).atan2(-fwd.z);
        let side = if self.rng.f() < 0.5 { 1.0 } else { -1.0 };
        self.h.crash_q = Some(Quat::axis_angle(Y, heading) * Quat::axis_angle(v3(0.0, 0.0, 1.0), side * 1.35));
        let w = v3((self.rng.f() - 0.5) * 6.0, (self.rng.f() - 0.5) * 6.0, (self.rng.f() - 0.5) * 6.0);
        self.h.w = w;
        self.toast("Crashed", reason, "bad");
    }

    /// One physics step on the true state. Returns the wind used (fed to the sensors).
    pub(crate) fn physics(&mut self, cmd: Option<Cmd>, dt: f64) -> V3 {
        let coll = if self.h.crashed {
            0.0
        } else if let Some(c) = cmd {
            c.coll
        } else {
            self.manual_coll(dt)
        };
        let h = &mut self.h;
        h.coll_cmd = coll;
        let up = up_of(h.q);
        let ge = ge_of(h.p.y, up.y);
        let w = wind_at(self.sim_t, self.set.wind);
        let v_rel = h.v - w;
        let sp = v_rel.length();
        let powered = h.armed && !h.hold && h.bat > 0.0 && !h.crashed;
        // flow through the disc: inflow > 0 when air comes up through it, inplane = edgewise airspeed
        let v_up = v_rel.dot(up);
        let inflow = -v_up;
        let inplane = (sp * sp - v_up * v_up).max(0.0).sqrt();
        // the vortex ring takes a moment to build (and to wash away): severity lags the instantaneous value
        let sev0 = if h.crashed || h.grounded || h.hold || !h.armed { 0.0 } else { vrs_of(inflow, inplane, coll, h.rpm) };
        h.vrs_lag += (sev0 - h.vrs_lag) * (dt / if sev0 > h.vrs_lag { 1.2 } else { 0.35 }).min(1.0);
        let sev = if h.vrs_lag < 0.002 { 0.0 } else { h.vrs_lag };
        let eta = if h.grounded { 1.0 } else { eta_of(inplane) };
        h.inflow = inflow;
        h.inplane = inplane;
        h.vrs = sev;
        h.eta = eta;
        let mut vrs_toast = false;
        if sev > 0.2 && !h.vrs_shown {
            h.vrs_shown = true;
            vrs_toast = true;
        } else if sev == 0.0 {
            h.vrs_shown = false;
        }

        // powertrain: governor on measured headspeed, rotor drag, autorotative drive from the airflow
        let drag = rot_drag(h.rpm, coll);
        let auto_drive = (0.02 * inflow.max(0.0) + 0.004 * inplane * (h.rpm * 2.0).min(1.0)) * (1.0 - 0.6 * sev);
        let mut motor = 0.0;
        if powered {
            let bat_f = 0.72 + 0.28 * (h.bat / 0.3).min(1.0);
            let pmax = 0.85 * bat_f;
            let er = self.est.rpm;
            motor = clamp(3.0 * ROTOR_I * (1.0 - er) + rot_drag(er, coll) - auto_drive, 0.0, pmax);
            if h.soft_start {
                motor = motor.min(0.33 * ROTOR_I + drag);
            }
        }
        if h.soft_start && h.rpm > 0.97 {
            h.soft_start = false;
        }
        h.motor = motor;
        h.rpm = clamp(h.rpm + (motor + auto_drive - drag) * dt / ROTOR_I, 0.0, 1.18);
        if !powered && h.rpm < 0.002 {
            h.rpm = 0.0;
        }
        if h.armed {
            h.bat = (h.bat - dt * (0.0003 + 0.0079 * motor * h.rpm)).max(0.0);
        }

        let t = coll * TMAX * h.rpm * h.rpm * ge * eta * (1.0 - 0.45 * sev);
        h.thrust_g = t / G;
        // thrust acts along the disc normal (tip-path plane), not the shaft
        let n_disc = h.rotor.normal().rot(h.q);
        let mut acc = n_disc * t;
        acc.y -= G;
        acc += v_rel * (-(0.22 + 0.035 * sp) * if h.grounded { 0.3 } else { 1.0 });
        acc += up * (-damp_of(h.rpm, inplane, sev) * v_up); // rotor disc damping along the shaft
        if h.grounded {
            // skid friction: grip depends on how much weight is still on the skids
            let n = (-acc.y).max(0.0);
            let a_h = acc.x.hypot(acc.z);
            let v_h = h.v.x.hypot(h.v.z);
            let (mu_s, mu_k) = (1.0, 0.8);
            if v_h < 0.05 && a_h <= mu_s * n {
                acc.x = 0.0;
                acc.z = 0.0;
                h.v.x = 0.0;
                h.v.z = 0.0;
            } else if v_h > 1e-6 {
                let f = (mu_k * n).min(v_h / dt);
                acc.x -= f * h.v.x / v_h;
                acc.z -= f * h.v.z / v_h;
            }
            h.ground_load = n / G;
        } else {
            h.ground_load = 0.0;
        }
        h.v += acc * dt;
        h.p += h.v * dt;
        if vrs_toast {
            self.toast("Settling with power", "Push forward to fly out of your own downwash", "bad");
        }

        // attitude: rotor flapping, hub moment and the flybarless gyro
        let tg = self.h.thrust_g;
        self.fbl_step(cmd, v_rel, motor, tg, dt);
        let wind_lvl = WINDS[self.set.wind].v;
        let h = &mut self.h;
        if !h.grounded && h.rpm > 0.5 && !h.crashed {
            let shud = if inplane > 2.0 && inplane < 4.5 { (crate::units::PI * (inplane - 2.0) / 2.5).sin() } else { 0.0 };
            let n = (wind_lvl * 1.2 + shud + sev * 5.0) * dt * 10.0;
            h.w.x += (self.rng.f() - 0.5) * n;
            h.w.z += (self.rng.f() - 0.5) * n;
        }
        let ang = h.w.length() * dt;
        if ang > 1e-9 {
            h.q = (h.q * Quat::axis_angle(h.w.normalize(), ang)).normalize();
        }

        self.ground_contact(up, dt);
        let h = &mut self.h;
        if h.p.y > 120.0 {
            h.p.y = 120.0;
            h.v.y = h.v.y.min(0.0);
        }
        let lim = 140.0;
        h.p.x = clamp(h.p.x, -lim, lim);
        h.p.z = clamp(h.p.z, -lim, lim);
        w
    }

    /// Ground contact through the four skid points; `up` is the shaft direction from the start of the step.
    fn ground_contact(&mut self, up: V3, dt: f64) {
        let h = &mut self.h;
        let mut min_y = f64::INFINITY;
        for s in SKIDS.iter() {
            let y = (s.rot(h.q) + h.p).y;
            if y < min_y {
                min_y = y;
            }
        }
        let body_low = min_y.min(h.p.y + MAST_H * up.y - 0.05).min(h.p.y - 0.15);
        let contact = if h.crashed { body_low } else { min_y };
        let mut crash_reason = None;
        let mut landed_vs = None;
        if contact < 0.0 {
            h.p.y -= contact;
            let impact = -h.v.y;
            if h.v.y < 0.0 {
                h.v.y = 0.0;
            }
            let was_air = !h.grounded;
            h.grounded = true;
            let tilt = crate::units::deg(clamp(up_of(h.q).y, -1.0, 1.0).acos());
            let hs = h.v.x.hypot(h.v.z);
            if was_air && h.airborne {
                h.last_impact = impact;
                h.last_touch_hs = hs;
            }
            if !h.crashed {
                use crate::jsonw::to_fixed;
                if was_air && h.airborne && impact > 3.0 {
                    crash_reason = Some(format!(
                        "Hit the ground at {} m/s. Keep descents under 1 m/s near the ground.",
                        to_fixed(impact, 1)
                    ));
                } else if tilt > 35.0 && h.rpm > 0.2 {
                    crash_reason = Some(format!("Blade strike: the disc touched down at {}° of tilt.", to_fixed(tilt, 0)));
                } else if hs > 3.2 {
                    crash_reason = Some("Dynamic rollover: the skids caught while sliding sideways.".into());
                } else if was_air && h.airborne && h.air_t > 0.8 {
                    landed_vs = Some(impact);
                }
            }
        } else if contact > 0.004 {
            h.grounded = false;
        }
        if let Some(r) = crash_reason {
            self.crash(r);
        }
        if let Some(vs) = landed_vs {
            self.landed(vs);
        }
        let h = &mut self.h;
        if contact < 0.0 {
            let tilt = crate::units::deg(clamp(up_of(h.q).y, -1.0, 1.0).acos());
            if h.crashed {
                let k = (1.0 - 6.0 * dt).max(0.0);
                h.v *= k;
                h.w *= k;
                if let Some(cq) = h.crash_q {
                    h.q = h.q.slerp(cq, (dt * 4.0).min(1.0));
                }
            } else if tilt < 20.0 {
                let mut fwd = v3(0.0, 0.0, -1.0).rot(h.q);
                fwd.y = 0.0;
                if fwd.length_sq() > 1e-4 {
                    let fwd = fwd.normalize();
                    let lvl = Quat::axis_angle(Y, (-fwd.x).atan2(-fwd.z));
                    h.q = h.q.slerp(lvl, (dt * 10.0).min(1.0));
                }
                let load = h.ground_load.min(1.0);
                h.w.x *= (1.0 - 12.0 * dt).max(0.0);
                h.w.z *= (1.0 - 12.0 * dt).max(0.0);
                if load > 0.6 {
                    h.w.y = 0.0; // skids planted: no pirouetting on the grass
                } else {
                    h.w.y *= (1.0 - 30.0 * load * dt).max(0.0);
                }
            }
        }
        if !h.grounded {
            h.air_t += dt;
            if min_y > 0.06 {
                h.airborne = true;
            }
        } else {
            h.air_t = 0.0;
        }
        if h.armed && h.airborne {
            h.flight_t += dt;
        }
    }
}
