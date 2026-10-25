//! Rotor head and flybarless gyro.
//!
//! Rotor: the disc has its own flapping state. The swashplate servos are rate limited with a
//! little slop; the tip-path plane follows the swashplate cyclic with a first-order flapping lag
//! (tau = 16 / (gamma * Omega), slower at low headspeed), lags the shaft when the body rotates
//! (rotor damping) with a small gyroscopic cross term, blows back away from the relative wind
//! (flapback, scaled by collective) and tilts sideways in forward flight (lateral flapping from
//! coning and inflow). The tilted disc puts a moment on the body through the hub spring (stiff
//! flybarless head, scales with rpm^2) and the thrust offset at the hub (scales with thrust, and
//! reverses with negative thrust), and it tilts the thrust vector. A small lightly damped
//! "head bob" mode is excited by fast cyclic, scaled by thrust.
//!
//! Gyro: self-level (and every autopilot mode) commands a tilt that becomes a body-rate demand;
//! rate mode commands rates directly. The rate loop is P + I + feed-forward on the measured
//! (or estimated) rates and outputs swashplate cyclic. Tail: heading-hold gyro (P + I) against
//! motor torque reaction, weathervaning and fin damping.

use crate::math::*;
use crate::sim::{Cmd, Sim};

/// Max commanded tilt at full cyclic in self-level, radians.
pub const MAX_TILT: f64 = 32.0 * core::f64::consts::PI / 180.0;

/// Rotor head constants (see README "Blade flapping").
pub mod rotor {
    /// Effective Lock number of a heavy RC blade
    pub const LOCK: f64 = 2.0;
    /// Rotor speed at the governed 2350 rpm, rad/s
    pub const OMEGA: f64 = 246.1;
    /// Flapping time constant at full headspeed: 16 / (gamma * Omega) = 32.5 ms; scales 1 / rpm
    pub const TAU_F: f64 = 16.0 / (LOCK * OMEGA);
    /// Hub spring: pitch / roll acceleration per radian of disc tilt at full headspeed (scales rpm^2)
    pub const K_HUB: f64 = 420.0;
    /// Thrust offset (thrust x hub height): acceleration per radian of disc tilt per g of thrust
    pub const K_THR: f64 = 130.0;
    /// Gyroscopic cross term of the disc lag (gamma / 16 of the on-axis lag)
    pub const CROSS: f64 = LOCK / 16.0 * 0.65;
    /// Max swashplate cyclic, radians of disc tilt
    pub const MAX_CYC: f64 = 0.2;
    /// Servo slew, radians of disc tilt per second (0.08 s / 60 deg servos: full cyclic in about 60 ms)
    pub const SERVO_RATE: f64 = 3.2;
    /// Swashplate slop / servo deadband, radians of disc tilt
    pub const SLOP: f64 = 0.0015;
    /// Flapback: disc tilt per m/s of edgewise airspeed, times (collective + 0.15)
    pub const K_BB: f64 = 0.0011;
    /// Lateral flapping: disc tilt per m/s, times thrust in g (coning)
    pub const K_LAT: f64 = 0.00035;
    /// Body aerodynamic damping of pitch / roll rates, 1/s
    pub const BODY_DAMP: f64 = 0.5;
    /// Head bob mode: 7 Hz, damping ratio 0.25, excitation per rad/s of swash rate per g
    pub const BOB_W: f64 = 2.0 * core::f64::consts::PI * 7.0;
    pub const BOB_Z: f64 = 0.25;
    pub const BOB_K: f64 = 10.0;
}

/// Flybarless rate-loop gains (cyclic radians per rad/s of rate error).
pub mod gains {
    pub const KP: f64 = 0.02;
    pub const KI: f64 = 0.3;
    /// I-term limit, radians of cyclic
    pub const I_LIM: f64 = 0.06;
    /// feed-forward multiplier on tau_f * w_des (1 = exact steady-state inverse of the disc lag)
    pub const FF: f64 = 1.0;
    /// self-level attitude gain (tilt error -> rate demand), and in a rescue
    pub const LEVEL: f64 = 7.0;
    pub const LEVEL_RESCUE: f64 = 12.0;
}

/// Stick expo used for manual input.
pub fn expo(x: f64) -> f64 {
    let e = 0.35;
    x * (1.0 - e) + x * x * x * e
}

/// Rotor flapping state carried on the heli.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rotor {
    /// swashplate cyclic at the servos (x: pitch axis, z: roll axis), radians of disc tilt
    pub sw_x: f64,
    pub sw_z: f64,
    /// disc tilt relative to the shaft (x: tilt that pitches the nose up, z: rolls left)
    pub fx: f64,
    pub fz: f64,
    /// disc tilt rate (rad/s), for vibration
    pub f_rate: f64,
    /// head bob mode state
    pub bx: f64,
    pub bz: f64,
    pub bvx: f64,
    pub bvz: f64,
    /// FBL rate-loop integrators
    pub ix: f64,
    pub iz: f64,
    /// FBL reference model: the rate the disc can deliver for the demand (and its derivative)
    pub rx: f64,
    pub rz: f64,
    pub rvx: f64,
    pub rvz: f64,
    /// last flapback / lateral flapping (for tests and the README)
    pub bb: f64,
}

impl Rotor {
    /// Disc tilt including the bob mode.
    pub fn tilt(&self) -> (f64, f64) {
        (self.fx + self.bx, self.fz + self.bz)
    }
    /// Thrust direction in the body frame (disc normal).
    pub fn normal(&self) -> V3 {
        let (fx, fz) = self.tilt();
        v3(-fz, 1.0, fx).normalize()
    }
}

/// Flapping time constant for a headspeed (rpm fraction).
pub fn tau_f(rpm: f64) -> f64 {
    rotor::TAU_F / rpm.max(0.3)
}

impl Sim {
    pub(crate) fn fbl_step(&mut self, cmd: Option<Cmd>, v_rel: V3, motor: f64, thrust_g: f64, dt: f64) {
        use rotor::*;
        let (yaw_cmd, pit_cmd, rol_cmd) = match cmd {
            Some(c) => (c.yaw, c.pit, c.rol),
            None => (expo(self.sticks.yaw), expo(self.sticks.pit), expo(self.sticks.rol)),
        };
        let rescue = matches!(&self.auto, Some(a) if a.mode == crate::autopilot::Mode::Rescue);
        // a stunt flies body rates directly through the rate loop (Cmd::rate)
        let rate_cmd = cmd.and_then(|c| c.rate);
        // manual-flight assists: AUTO-YAW rate and the RATE tail (autopilot modes fly HOLD)
        let manual = cmd.is_none();
        let ay_rate = if manual { self.auto_yaw_rate(self.sticks.yaw) } else { 0.0 };
        let tail_rate = manual && self.set.tail_rate;
        let est_q = self.est.q;
        let wm = self.est.w;
        let est_rpm = self.est.rpm;
        let h = &mut self.h;
        let mut w_des = V3::ZERO;
        let mut active = true;
        if h.crashed {
            h.w = h.w.lerp(V3::ZERO, (dt / 0.6).min(1.0));
            return;
        } else if h.rpm < 0.25 {
            active = false;
        } else if let Some(r) = rate_cmd {
            w_des = r;
        } else if self.set.level || cmd.is_some() {
            let up_e = up_of(est_q);
            let (fwd, right) = fwd_right(est_q);
            let des_up = (Y + fwd * (pit_cmd * MAX_TILT).tan() + right * (rol_cmd * MAX_TILT).tan()).normalize();
            let mut err = up_e.cross(des_up);
            if up_e.dot(des_up) < 0.0 {
                err = err.normalize() * 2.0; // recover from inverted
            }
            let err = err.rot(est_q.inv());
            w_des = err * if rescue { gains::LEVEL_RESCUE } else { gains::LEVEL };
            w_des.y = -yaw_cmd * 4.5 + ay_rate;
        } else {
            let rate = 4.2; // about 240 deg/s
            w_des = v3(-pit_cmd * rate, -yaw_cmd * 5.2 + ay_rate, -rol_cmd * rate);
        }
        let vb = v_rel.rot(h.q.inv()); // airflow in the body frame
        let gl = if h.grounded { (h.ground_load / 0.6).min(1.0) } else { 0.0 }; // weight on the skids
        let r = &mut h.rotor;

        // ---- FBL rate loop -> swashplate cyclic
        let tau_n = tau_f(est_rpm.max(0.5));
        let (mut cx, mut cz) = (0.0, 0.0);
        if active {
            // reference model: the second-order response of the disc + body to a rate demand
            // (w'' + w'/tau + L w = L w_des), so P and I only act on what the head can't follow
            let l_est = (K_HUB * est_rpm * est_rpm + K_THR * h.thrust_g).max(0.2 * K_HUB * est_rpm * est_rpm).max(40.0);
            let wn2 = l_est;
            let c1 = 1.0 / tau_n;
            let n = 4;
            let ddt = dt / n as f64;
            for _ in 0..n {
                r.rvx += (wn2 * (w_des.x - r.rx) - c1 * r.rvx) * ddt;
                r.rvz += (wn2 * (w_des.z - r.rz) - c1 * r.rvz) * ddt;
                r.rx += r.rvx * ddt;
                r.rz += r.rvz * ddt;
            }
            let (ex, ez) = (r.rx - wm.x, r.rz - wm.z);
            if h.grounded {
                // no I-term wind-up on the skids (the classic FBL tip-over)
                let k = (1.0 - 4.0 * dt).max(0.0);
                r.ix *= k;
                r.iz *= k;
            } else {
                let il = gains::I_LIM / gains::KI;
                r.ix = clamp(r.ix + ex * dt, -il, il);
                r.iz = clamp(r.iz + ez * dt, -il, il);
            }
            // feed-forward: the steady-state cyclic for the demand (disc lag tau * w) plus the
            // body damping it must hold against
            let ffk = gains::FF * (tau_n + BODY_DAMP / l_est);
            cx = ffk * w_des.x + gains::KP * ex + gains::KI * r.ix;
            cz = ffk * w_des.z + gains::KP * ez + gains::KI * r.iz;
        } else {
            r.ix = 0.0;
            r.iz = 0.0;
            r.rx = wm.x;
            r.rz = wm.z;
            r.rvx = 0.0;
            r.rvz = 0.0;
        }
        cx = clamp(cx, -MAX_CYC, MAX_CYC);
        cz = clamp(cz, -MAX_CYC, MAX_CYC);
        // ---- servos: rate limited, with slop
        let step = SERVO_RATE * dt;
        let (sx0, sz0) = (r.sw_x, r.sw_z);
        let servo = |s: f64, c: f64| -> f64 {
            let d = c - s;
            if d.abs() <= SLOP {
                s
            } else {
                s + clamp(d - SLOP * d.signum(), -step, step)
            }
        };
        r.sw_x = servo(r.sw_x, cx);
        r.sw_z = servo(r.sw_z, cz);
        let (dsx, dsz) = ((r.sw_x - sx0) / dt, (r.sw_z - sz0) / dt);

        // ---- flapping: first-order tip-path plane relative to the shaft
        let rpm = h.rpm;
        let tau = tau_f(rpm);
        let spin = clamp((rpm - 0.1) / 0.3, 0.0, 1.0); // no flapping response on a stopped rotor
        // flapback away from the relative wind (scaled by collective) and lateral flapping (coning)
        let coll = h.coll_cmd;
        let (u, v) = (-vb.z, vb.x); // forward and rightward airspeed in the body frame
        let kb = K_BB * (coll + 0.15) * rpm;
        let kl = K_LAT * thrust_g;
        let (bbx, bbz) = (kb * u + kl * v, kb * v - kl * u);
        r.bb = bbx;
        let tx = r.sw_x * spin + bbx - tau * (h.w.x + CROSS * h.w.z);
        let tz = r.sw_z * spin + bbz - tau * (h.w.z - CROSS * h.w.x);
        let (fx0, fz0) = (r.fx, r.fz);
        let k = (dt / tau).min(1.0);
        r.fx += (tx - r.fx) * k;
        r.fz += (tz - r.fz) * k;
        r.f_rate = ((r.fx - fx0).hypot(r.fz - fz0)) / dt;
        // head bob: lightly damped flap mode driven by fast cyclic, scaled by thrust
        let drive = BOB_K * thrust_g.abs().min(2.0) * spin;
        let (w2, zw) = (BOB_W * BOB_W, 2.0 * BOB_Z * BOB_W);
        r.bvx += (-w2 * r.bx - zw * r.bvx + drive * dsx) * dt;
        r.bvz += (-w2 * r.bz - zw * r.bvz + drive * dsz) * dt;
        r.bx += r.bvx * dt;
        r.bz += r.bvz * dt;

        // ---- hub moment -> body pitch / roll acceleration
        let (fx, fz) = r.tilt();
        let l_hub = K_HUB * rpm * rpm + K_THR * thrust_g;
        let l_hub = l_hub.max(0.2 * K_HUB * rpm * rpm);
        let on_ground = 1.0 - 0.7 * gl;
        h.w.x += (l_hub * fx * on_ground - BODY_DAMP * h.w.x) * dt;
        h.w.z += (l_hub * fz * on_ground - BODY_DAMP * h.w.z) * dt;

        // ---- tail: heading-hold gyro
        let err_y = w_des.y - wm.y;
        if tail_rate {
            // rate gyro: no heading lock, the integrator bleeds off
            h.yaw_i *= (1.0 - 8.0 * dt).max(0.0);
        } else if gl >= 1.0 {
            h.yaw_i *= (1.0 - 2.0 * dt).max(0.0);
        } else {
            h.yaw_i = clamp(h.yaw_i + err_y * dt, -0.6, 0.6);
        }
        let tl = 45.0 * h.rpm * h.rpm;
        // RATE tail: torque precompensation (tail pitch mixed from motor load) instead of the lock
        let precomp = if tail_rate { -8.0 * motor * (1.0 - gl) } else { 0.0 };
        let tail = clamp(22.0 * err_y + 30.0 * h.yaw_i + precomp, -tl, tl);
        h.w.y += (tail + (8.0 * motor - 0.5 * vb.x) * (1.0 - gl) - 0.15 * vb.z.abs() * h.w.y) * dt;
    }
}
