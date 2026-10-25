//! Native runner: flies takeoff -> 3 laps of the circuit -> land and prints a summary.
//! `helisim [seed] [real]`
use helisim::autopilot::Mode;
use helisim::sim::{Command, Sim, DT};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let seed = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    let mut s = Sim::new(seed);
    if args.iter().any(|a| a == "real") {
        s.command(Command::Sensors);
    }
    s.command(Command::Takeoff);
    run_until(&mut s, 25.0, |s| s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold));
    println!("hold: alt {:.2} m at t {:.1} s", s.h.alt(), s.sim_t);
    s.command(Command::Circuit);
    run_until(&mut s, 200.0, |s| s.auto.as_ref().map_or(true, |a| a.laps >= 3));
    if let Some(a) = &s.auto {
        let lt: Vec<String> = a.lap_times.iter().map(|x| format!("{x:.1}")).collect();
        println!("circuit: {} laps, {} s", a.laps, lt.join(", "));
    }
    s.command(Command::Land);
    run_until(&mut s, 60.0, |s| s.auto.is_none() && s.h.grounded);
    println!(
        "land: touchdown {:.2} m/s, {} | crashed {} | battery {:.0}%",
        s.h.last_impact,
        s.last_toast.as_ref().map(|t| format!("{} {}", t.title, t.sub)).unwrap_or_default(),
        s.h.crashed,
        s.h.bat * 100.0
    );
}

fn run_until(s: &mut Sim, sec: f64, f: impl Fn(&Sim) -> bool) {
    let n = (sec / DT).round() as u64;
    for _ in 0..n {
        s.step(DT);
        if f(s) {
            return;
        }
    }
}
