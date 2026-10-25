//! Development trace: fly one stunt and print the state every 0.1 s.
//! `cargo run --release --example stunt_dev -- <kind 0..4> [seed] [real] [basic] [quiet]`
use helisim::autopilot::Mode;
use helisim::math::*;
use helisim::sim::{Command, Sim, DT};
use helisim::stunt::Kind;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let k = Kind::from_idx(a.get(1).and_then(|x| x.parse().ok()).unwrap_or(0)).unwrap();
    let seed = a.get(2).and_then(|x| x.parse().ok()).unwrap_or(1);
    let quiet = a.iter().any(|x| x == "quiet");
    let mut s = Sim::new(seed);
    if a.iter().any(|x| x == "real") {
        s.command(Command::Sensors);
    }
    if a.iter().any(|x| x == "basic") {
        s.set.ekf = false;
    }
    s.command(Command::Takeoff);
    for _ in 0..240 * 20 {
        s.step(DT);
        if s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold) {
            break;
        }
    }
    s.start_stunt(k);
    let mut last_ph = -1i64;
    for i in 0..240 * 60 {
        s.step(DT);
        let ph = s.auto.as_ref().and_then(|a| a.st.as_ref()).map(|x| x.phase as i64).unwrap_or(-1);
        let man = ph == 2 || ph == 3;
        if (!quiet && i % if man { 12 } else { 120 } == 0) || ph != last_ph {
            let h = &s.h;
            let up = up_of(h.q);
            println!(
                "t {:6.2} ph {:2} est ({:5.1} {:5.1} {:5.1}) eup {:5.2} gps {} {} mode {:?} p ({:6.1} {:5.1} {:6.1}) v ({:5.1} {:5.1} {:5.1}) up ({:5.2} {:5.2} {:5.2}) hd {:6.1} w ({:5.2} {:5.2} {:5.2}) coll {:5.2} rpm {:.2} fl ({:.3} {:.3}) p3d {} r {}",
                s.sim_t, ph, s.est.p.x, s.est.alt, s.est.p.z, up_of(s.est.q).y, s.sens.st.sats, s.sens.ekf.status().name(), s.auto.as_ref().map(|a| a.mode), h.p.x, h.alt(), h.p.z, h.v.x, h.v.y, h.v.z, up.x, up.y, up.z,
                heading_of(h.q).to_degrees(), h.w.x, h.w.y, h.w.z, h.coll_cmd, h.rpm, h.rotor.fx, h.rotor.fz, s.set.pitch3d, s.rescues
            );
            last_ph = ph;
        }
        if s.h.crashed {
            println!("CRASHED {:?}", s.last_toast);
            break;
        }
        if s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold) && s.stunt_count > 0 {
            break;
        }
    }
    println!("last toast {:?}", s.last_toast);
    println!("result {:?}", s.stunt_last);
}
