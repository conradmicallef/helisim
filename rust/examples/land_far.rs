//! LAND from far away: time to touchdown, transit heights, touchdown speed and accuracy.
//! `cargo run --release --example land_far -- [real] [nocam]`
//! (The before-version copy had the camera hook line removed.)
use helisim::autopilot::Mode;
use helisim::sim::{Command, Sim, DT};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let real = args.iter().any(|a| a == "real");
    let nocam = args.iter().any(|a| a == "nocam");
    for &(px, pz, name) in &[(-26.0, -15.0, "30 m"), (-38.0, -24.0, "45 m")] {
        for seed in 1000u64..1005 {
            let mut s = Sim::new(seed);
            if real {
                s.command(Command::Sensors);
            }
            if nocam {
                s.sens.cam.enabled = false;
            }
            s.command(Command::Takeoff);
            run(&mut s, 25.0, |s| s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold));
            {
                let a = s.auto.as_mut().unwrap();
                a.anchor.x = px;
                a.anchor.z = pz;
                a.alt = 5.0;
            }
            run(&mut s, 60.0, |s| (s.h.p.x - px).hypot(s.h.p.z - pz) < 0.5 && s.h.v.x.hypot(s.h.v.z) < 0.3 && (s.h.alt() - 5.0).abs() < 0.3);
            run(&mut s, 3.0, |_| false);
            let r0 = s.rescues;
            let t0 = s.sim_t;
            s.command(Command::Land);
            let (mut hmax, mut hmin, mut vmax) = (0.0f64, 1e9f64, 0.0f64);
            run(&mut s, 120.0, |s| {
                let d = s.h.p.x.hypot(s.h.p.z);
                if d > 5.0 {
                    hmax = hmax.max(s.h.alt());
                    hmin = hmin.min(s.h.alt());
                }
                vmax = vmax.max(s.h.v.x.hypot(s.h.v.z));
                (s.h.grounded && s.h.airborne) || s.h.crashed
            });
            println!(
                "{name} {} seed {seed}: touchdown after {:.1} s | transit alt {:.2}-{:.2} m | max ground speed {:.1} m/s | touchdown {:.2} m/s, {:.2} m from pad | rescues {} | crashed {}",
                if real { "REAL" } else { "IDEAL" },
                s.sim_t - t0,
                hmin,
                hmax,
                vmax,
                s.h.last_impact,
                s.h.p.x.hypot(s.h.p.z),
                s.rescues - r0,
                s.h.crashed
            );
        }
    }
}

fn run(s: &mut Sim, sec: f64, mut f: impl FnMut(&Sim) -> bool) {
    for _ in 0..(sec / DT).round() as u64 {
        s.step(DT);
        if f(s) {
            return;
        }
    }
}
