//! Trace one LAND from far: `cargo run --release --example land_trace -- [x z] [real] [wind]`
use helisim::autopilot::Mode;
use helisim::sim::{Command, Sim, DT};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let px: f64 = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(-38.0);
    let pz: f64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(-24.0);
    let mut s = Sim::new(1000);
    if a.iter().any(|x| x == "real") { s.command(Command::Sensors); }
    if a.iter().any(|x| x == "gusty") { s.set.wind = 2; }
    if a.iter().any(|x| x == "nocam") { s.sens.cam.enabled = false; }
    s.command(Command::Takeoff);
    run(&mut s, 25.0, |s| s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold));
    { let au = s.auto.as_mut().unwrap(); au.anchor.x = px; au.anchor.z = pz; au.alt = 5.0; }
    run(&mut s, 60.0, |s| (s.h.p.x - px).hypot(s.h.p.z - pz) < 0.5 && s.h.v.x.hypot(s.h.v.z) < 0.3);
    run(&mut s, 3.0, |_| false);
    s.command(Command::Land);
    let t0 = s.sim_t;
    let mut k = 0u64;
    run(&mut s, 120.0, |s| {
        k += 1;
        if k % 120 == 0 {
            let au = s.auto.as_ref();
            // error of the flown target relative to the true pad, as seen from the true heli
            let tgt = au.map(|a| a.tgt).unwrap_or_default();
            let te = ((tgt.x - s.est.p.x) - (0.0 - s.h.p.x)).hypot((tgt.z - s.est.p.z) - (0.0 - s.h.p.z));
            let pe = ((s.sens.pl.pad.x - s.est.p.x) + s.h.p.x).hypot((s.sens.pl.pad.z - s.est.p.z) + s.h.p.z);
            println!("t {:5.1} d {:5.1} alt {:5.2} hs {:4.1} vy {:5.2} hdg {:6.1} desc {} cam {} pl {} tgtErr {:.2} plErr {:.2} sd {:.2} vErr {:.2}", s.sim_t - t0, s.h.p.x.hypot(s.h.p.z), s.h.alt(), s.h.v.x.hypot(s.h.v.z), s.h.v.y,
              helisim::math::heading_of(s.h.q).to_degrees(), au.map_or(false, |a| a.descending), au.map_or(false, |a| a.cam_tgt), s.sens.pl.status(true), te, pe, s.sens.pl.var.sqrt(), (s.est.v.x - s.h.v.x).hypot(s.est.v.z - s.h.v.z));
        }
        (s.h.grounded && s.h.airborne) || s.h.crashed
    });
    println!("touchdown {:.2} m/s at {:.2} m after {:.1} s, rescues {}, cam dets {} fakes {} fused {} rej {}", s.h.last_impact, s.h.p.x.hypot(s.h.p.z), s.sim_t - t0, s.rescues, s.sens.cam.n_det, s.sens.cam.n_fake, s.sens.pl.n_fused, s.sens.pl.n_rej);
}
fn run(s: &mut Sim, sec: f64, mut f: impl FnMut(&Sim) -> bool) { for _ in 0..(sec / DT).round() as u64 { s.step(DT); if f(s) { return; } } }
