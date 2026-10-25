//! Attitude step responses (calm air, IDEAL sensors, hover at 20 m), for the README table.
//! `cargo test --release --test step -- --nocapture`
use helisim::autopilot::Mode;
use helisim::math::*;
use helisim::sim::{Command, Input, Sim, DT};

fn hover20(seed: u64) -> Sim {
    let mut s = Sim::new(seed);
    s.set.wind = 0;
    s.command(Command::Takeoff);
    for _ in 0..(240 * 25) {
        s.step(DT);
        if s.auto.as_ref().is_some_and(|a| a.mode == Mode::Hold) {
            break;
        }
    }
    s.auto.as_mut().unwrap().alt = 20.0;
    for _ in 0..(240 * 25) {
        s.step(DT);
    }
    s.cancel_auto(false);
    s
}
fn expo_inv(y: f64) -> f64 {
    let (mut lo, mut hi) = (0.0, 1.0);
    for _ in 0..60 {
        let m = 0.5 * (lo + hi);
        if m * 0.65 + 0.35 * m * m * m < y {
            lo = m
        } else {
            hi = m
        }
    }
    0.5 * (lo + hi)
}
/// (rise 10-90 % s, overshoot %, settle 5 % s, final)
fn metrics(tr: &[(f64, f64)], target: f64) -> (f64, f64, f64, f64) {
    let (mut t10, mut t90) = (None, None);
    let mut pk = 0.0f64;
    for &(t, v) in tr {
        if t10.is_none() && v >= 0.1 * target {
            t10 = Some(t);
        }
        if t90.is_none() && v >= 0.9 * target {
            t90 = Some(t);
        }
        pk = pk.max(v);
    }
    let mut settle = 0.0;
    for &(t, v) in tr {
        if (v - target).abs() > 0.05 * target {
            settle = t;
        }
    }
    let fin = tr.last().unwrap().1;
    (t90.unwrap_or(9.0) - t10.unwrap_or(0.0), 100.0 * (pk / target - 1.0).max(0.0), settle, fin)
}

#[test]
fn step_response() {
    // 30 deg/s roll-rate step in rate mode
    let mut s = hover20(900);
    s.command(Command::Level);
    let target = 30.0f64.to_radians();
    let x = expo_inv(target / 4.2);
    let thr = s.sticks.thr;
    let mut tr = vec![];
    for i in 0..(240 * 1) {
        s.input(&Input { rol: x, ..Input::default() });
        s.sticks.thr = thr;
        s.step(DT);
        tr.push((i as f64 * DT, -s.h.w.z));
    }
    let (r, o, st, f) = metrics(&tr, target);
    println!("STEP roll rate 30 deg/s: rise {:.3} s overshoot {:.1}% settle(5%) {:.3} s final {:.1} deg/s", r, o, st, f.to_degrees());
    // 20 deg self-level step
    let mut s = hover20(901);
    let target = 20.0f64.to_radians();
    let x = expo_inv(20.0 / 32.0);
    let thr = s.sticks.thr;
    let mut tr = vec![];
    for i in 0..(240 * 2) {
        s.input(&Input { rol: x, ..Input::default() });
        s.sticks.thr = thr;
        s.step(DT);
        let up = up_of(s.h.q);
        let (_, right) = fwd_right(s.h.q);
        tr.push((i as f64 * DT, up.dot(right).atan2(up.y)));
    }
    let (r, o, st, f) = metrics(&tr, target);
    println!("STEP self-level 20 deg: rise {:.3} s overshoot {:.1}% settle(5%) {:.3} s final {:.1} deg", r, o, st, f.to_degrees());
    assert!(r < 0.3 && o < 10.0 && (f.to_degrees() - 20.0).abs() < 1.0);
}
