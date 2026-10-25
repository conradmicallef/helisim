//! Diagnostic for the REAL-mode inverted hover (and any other stunt): per seed, flies the stunt
//! the way `tests/suites.rs` does and logs true vs estimated gyro bias, accelerometer bias,
//! world-frame tilt error (x / z), velocity and position error, the gravity-aiding and drag
//! fusion counts, baro use and the covariance while GPS is lost, plus the true deviation before
//! and after GPS reacquire.
//! `cargo run --release --example inv_diag -- [seed0] [n] [kind 0..4] [trace] [ideal] [basic]`
use helisim::autopilot::Mode;
use helisim::math::*;
use helisim::sim::{Command, Input, Sim, DT};
use helisim::stunt::Kind;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let s0: u64 = a.get(1).and_then(|x| x.parse().ok()).unwrap_or(800);
    let n: u64 = a.get(2).and_then(|x| x.parse().ok()).unwrap_or(5);
    let k = Kind::from_idx(a.get(3).and_then(|x| x.parse().ok()).unwrap_or(4)).unwrap();
    let flag = |f: &str| a.iter().any(|x| x == f);
    let (mut sc_sum, mut dev_sum, mut dev_ok) = (0.0, 0.0, 0);
    for seed in s0..s0 + n {
        let mut s = Sim::new(seed);
        s.set.thr_curve = false;
        s.set.protect = true;
        if !flag("ideal") {
            s.command(Command::Sensors);
        }
        if flag("basic") {
            s.set.ekf = false;
        }
        s.command(Command::Takeoff);
        let mut i = 0u64;
        let step = |s: &mut Sim, i: &mut u64| {
            if *i % 4 == 0 {
                s.input(&Input::default());
            }
            s.step(DT);
            *i += 1;
        };
        while !s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold) && i < 240 * 25 {
            step(&mut s, &mut i);
        }
        for _ in 0..480 {
            step(&mut s, &mut i);
        }
        s.command(Command::Stunts);
        s.command(match k {
            Kind::Pirouette => Command::StuntPirouette,
            Kind::Loop => Command::StuntLoop,
            Kind::Roll => Command::StuntRoll,
            Kind::Flip => Command::StuntFlip,
            Kind::Inverted => Command::StuntInverted,
        });
        let (r0, mut in_man, mut lost_t, mut back) = (s.rescues, false, None::<f64>, false);
        let (mut maxv, mut maxp, mut maxtx, mut maxtz) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let (mut dev_out, mut dev_after, mut e_p) = (0.0f64, 0.0f64, V3::ZERO);
        let (mut grav0, mut drag0, mut drag_lost) = (0, 0, 0);
        let mut at_loss: Option<(V3, V3, V3, f64, f64)> = None; // tilt err, bg err, ve, sig_v, sig_wind
        let mut at_back: Option<(V3, V3, V3, f64, f64)> = None;
        for j in 0..240 * 60u64 {
            step(&mut s, &mut i);
            let ph = s.auto.as_ref().and_then(|a| a.st.as_ref()).map(|x| x.phase as i64).unwrap_or(-1);
            let ek = &s.sens.ekf;
            let h = &s.h;
            if ph == 2 && !in_man {
                in_man = true;
                e_p = h.p;
                grav0 = ek.h.fused_grav;
                drag0 = ek.h.fused_drag;

            }
            let dth = rotvec(h.q * ek.q.inv());
            let (ve, pe) = (ek.v - h.v, ek.p - h.p);
            let bge = ek.bg - s.sens.g_bias;
            let snap = (dth, bge, ve, ek.sig_vel(), ek.sig_wind());
            if in_man {
                maxv = maxv.max(ve.hyp_xz());
                maxp = maxp.max(pe.hyp_xz());
                maxtx = maxtx.max(dth.x.abs().to_degrees());
                maxtz = maxtz.max(dth.z.abs().to_degrees());
                let d = (h.p.x - e_p.x).hypot(h.p.z - e_p.z);
                if back {
                    dev_after = dev_after.max(d);
                } else {
                    dev_out = dev_out.max(d);
                }
            }
            if s.sens.st.gps_lost && lost_t.is_none() {
                lost_t = Some(s.sim_t);
                at_loss = Some(snap);
                drag_lost = ek.h.fused_drag;

            }
            if lost_t.is_some() && !s.sens.st.gps_lost && !back {
                back = true;
                at_back = Some(snap);
                drag_lost = ek.h.fused_drag - drag_lost;

            }
            let tr_every = if flag("fine") { 6 } else { 24 };
            if flag("trace") && in_man && j % tr_every == 0 {
                println!(
                    "t {:6.2} dev {:4.2} vt ({:+5.2} {:+5.2}) ph {} upy {:5.2} |w| {:4.2} st {:<12} tilt x {:+5.2} z {:+5.2} deg | ve ({:+5.2} {:+5.2}) pe ({:+5.2} {:+5.2}) | bg_err ({:+5.2} {:+5.2} {:+5.2}) deg/s | ba ({:+5.3} {:+5.3} {:+5.3}) | wind est ({:+4.1} {:+4.1}) true ({:+4.1} {:+4.1}) | sig v {:4.2} wind {:4.2} | tr drag {:4.2} grav {:4.2} baro {:4.2}",
                    s.sim_t, (h.p.x - e_p.x).hypot(h.p.z - e_p.z), h.v.x, h.v.z, ph, up_of(h.q).y, h.w.length(), ek.status().name(), dth.x.to_degrees(), dth.z.to_degrees(), ve.x, ve.z, pe.x, pe.z,
                    bge.x.to_degrees(), bge.y.to_degrees(), bge.z.to_degrees(), ek.ba.x, ek.ba.y, ek.ba.z, ek.wind.x, ek.wind.z, s.wind.x, s.wind.z,
                    ek.sig_vel(), ek.sig_wind(), ek.h.tr_drag, ek.h.tr_grav, ek.h.tr_baro
                );
            }
            if s.h.crashed {
                println!("seed {seed} CRASHED");
                break;
            }
            if s.stunt_count > 0 && s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold) {
                break;
            }
        }
        let ek = &s.sens.ekf;
        let r = s.stunt_last.clone();
        let f3 = |v: V3, d: bool| {
            let m = if d { 1.0f64.to_degrees() } else { 1.0 };
            format!("({:+.2} {:+.2})", v.x * m, v.z * m)
        };
        if let (Some(l), Some(b)) = (at_loss, at_back) {
            println!(
                "  GPS lost {:.1} s | at loss: tilt err {} deg, gyro bias err {} deg/s, vel err {} m/s, sig v {:.2} | at reacquire: tilt err {} deg, gyro bias err {} deg/s, vel err {} m/s, sig v {:.2}, sig wind {:.2} | drag fused while lost {} | grav fused in manoeuvre {}",
                s.sim_t - lost_t.unwrap(), f3(l.0, true), f3(l.1, true), f3(l.2, false), l.3, f3(b.0, true), f3(b.1, true), f3(b.2, false), b.3, b.4, drag_lost, ek.h.fused_grav - grav0
            );
        }
        let sc = r.as_ref().map_or(0, |r| r.score);
        let dev = r.as_ref().map_or(0.0, |r| r.max_dev);
        sc_sum += sc as f64;
        dev_sum += dev;
        dev_ok += (dev <= 1.5) as u32;
        println!(
            "seed {seed} {:?}: score {} dev {:.2} lost {:.2} | true dev until GPS back {:.2} m, after {:.2} m | max est err vel {:.2} m/s pos {:.2} m tilt x {:.2} z {:.2} deg | drag fused {} | rescues {}",
            k,
            sc,
            dev,
            r.as_ref().map_or(0.0, |r| r.height_lost),
            dev_out,
            dev_after,
            maxv,
            maxp,
            maxtx,
            maxtz,
            ek.h.fused_drag - drag0,
            s.rescues - r0
        );
    }
    println!("MEAN score {:.1} dev {:.2} | dev <= 1.5 m: {}/{}", sc_sum / n as f64, dev_sum / n as f64, dev_ok, n);
}
