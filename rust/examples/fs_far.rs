//! Battery failsafe far from the pad, and autorotate from far: `cargo run --release --example fs_far`
use helisim::autopilot::Mode;
use helisim::sim::{Command, Sim, DT};
fn main() {
    for (bat, real) in [(0.155, false), (0.155, true), (0.085, false), (0.085, true)] {
        let mut s = Sim::new(77);
        if real { s.command(Command::Sensors); }
        s.command(Command::Takeoff);
        run(&mut s, 25.0, |s| s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold));
        { let a = s.auto.as_mut().unwrap(); a.anchor.x = -30.0; a.anchor.z = -25.0; a.alt = 6.0; }
        run(&mut s, 60.0, |s| (s.h.p.x + 30.0).hypot(s.h.p.z + 25.0) < 0.5 && s.h.v.x.hypot(s.h.v.z) < 0.3);
        s.h.bat = bat;
        let t0 = s.sim_t;
        let (mut hmax, mut vmax) = (0.0f64, 0.0f64);
        run(&mut s, 120.0, |s| { hmax = hmax.max(s.h.alt()); vmax = vmax.max(s.h.v.x.hypot(s.h.v.z)); (s.h.grounded && s.h.airborne) || s.h.crashed });
        println!("failsafe bat {bat} real {real}: touchdown {:.2} m/s at {:.2} m after {:.1} s, max alt {hmax:.1}, max speed {vmax:.1}, rescues {}, crashed {} | {}",
            s.h.last_impact, s.h.p.x.hypot(s.h.p.z), s.sim_t - t0, s.rescues, s.h.crashed, s.last_toast.as_ref().map(|t| t.title.clone()).unwrap_or_default());
    }
}
fn run(s: &mut Sim, sec: f64, mut f: impl FnMut(&Sim) -> bool) { for _ in 0..(sec / DT).round() as u64 { s.step(DT); if f(s) { return; } } }
