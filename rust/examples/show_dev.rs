//! Development trace: fly the SHOW autopilot demo and print each manoeuvre's judging.
//! `cargo run --release --example show_dev -- [seed] [real] [hover|aero] [trace <man idx>]`
use helisim::autopilot::Mode;
use helisim::show::{Part, Ph};
use helisim::sim::{Command, Sim, DT};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let seed = a.get(1).and_then(|x| x.parse().ok()).unwrap_or(1);
    let mut s = Sim::new(seed);
    if a.iter().any(|x| x == "real") {
        s.command(Command::Sensors);
    }
    s.show_ui.demo = true;
    s.show_ui.part = if a.iter().any(|x| x == "hover") { Part::Hover } else if a.iter().any(|x| x == "aero") { Part::Aero } else { Part::Full };
    let trace: Option<usize> = a.iter().position(|x| x == "trace").and_then(|i| a.get(i + 1)).and_then(|x| x.parse().ok());
    s.show_start();
    let mut last = (99usize, 99u8);
    for i in 0..240 * 1200 {
        s.step(DT);
        if s.h.crashed {
            println!("CRASHED {:?}", s.last_toast);
            break;
        }
        let Some(sh) = s.show.as_ref() else { break };
        let key = (sh.i, sh.ph as u8);
        if key != last {
            println!("t {:7.2} man {} {:?} ph {:?} mode {:?} p ({:.1} {:.1} {:.1}) bat {:.0}%", s.sim_t, sh.i, sh.man(), sh.ph, s.auto.as_ref().map(|a| a.mode), s.h.p.x, s.h.alt(), s.h.p.z, s.h.bat * 100.0);
            if sh.ph == Ph::Judged {
                let r = sh.res.last().unwrap();
                println!("    base {:.2} marks {:?} score {:.2} x K {} = {:.2} | {:?}", r.base, r.marks, r.score, r.k, r.ks, r.why);
            }
            last = key;
        }
        if trace == Some(sh.i) && sh.ph == Ph::Fly && i % 24 == 0 {
            let r = sh.track.at(sh.t_ph);
            let h = &s.h;
            let up = helisim::math::up_of(h.q);
            println!(
                "  t {:5.2} est ({:6.1} {:5.1} {:6.1}) ref ({:6.1} {:5.1} {:6.1}) p ({:6.1} {:5.1} {:6.1}) e {:4.2} v ({:5.1} {:5.1} {:5.1}) up ({:5.2} {:5.2} {:5.2}) rup ({:5.2} {:5.2} {:5.2}) hd {:6.1} coll {:5.2} rpm {:.2}",
                sh.t_ph, s.est.p.x, s.est.alt, s.est.p.z, r.p.x, r.p.y, r.p.z, h.p.x, h.alt(), h.p.z, (helisim::math::v3(h.p.x, h.alt(), h.p.z) - r.p).length(), h.v.x, h.v.y, h.v.z, up.x, up.y, up.z, r.up.x, r.up.y, r.up.z,
                helisim::math::heading_of(h.q).to_degrees(), h.coll_cmd, h.rpm
            );
        }
        let _ = Mode::Show;
    }
    if let Some(r) = &s.show_last {
        println!("TOTAL {:.1} / {:.0} = {:.1}% complete {} rescues {} bat {:.0}%", r.total, r.max, r.pct, r.complete, s.rescues, s.h.bat * 100.0);
    }
}
