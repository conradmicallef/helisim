//! Native ports of the JS harnesses test.js .. test5.js. Every scenario prints its key numbers
//! (prefixed `RUST`) for the side-by-side table in README.md and asserts the expected behaviour.
//! Run with `cargo test --release -- --nocapture --test-threads=1` to see the numbers in order.

use helisim::autopilot::Mode;
use helisim::math::*;
use helisim::sim::{Command, Input, Sim, DT};
use std::collections::HashSet;

/// Mirror of lib.js: fast-forward with readInputs every 4 steps, keys, stick touch.
struct T {
    s: Sim,
    t: f64,
    keys: HashSet<&'static str>,
}

impl T {
    fn new(seed: u64) -> T {
        let mut s = Sim::new(seed);
        // the JS-parity suites set raw stick positions: keep the original linear throttle curve
        s.set.thr_curve = false;
        T { s, t: 0.0, keys: HashSet::new() }
    }
    fn fresh(seed: u64, protect: bool, real: bool) -> T {
        let mut t = T::new(seed);
        t.s.set.protect = protect;
        if real {
            t.s.command(Command::Sensors);
        }
        t
    }
    fn read_inputs(&mut self, dt: f64) {
        let k = |n| self.keys.contains(n);
        let mut inp = Input::default();
        if k("w") {
            inp.thr_delta += 0.45 * dt;
        }
        if k("s") {
            inp.thr_delta -= 0.45 * dt;
        }
        if k("a") {
            inp.yaw = -1.0;
        }
        if k("d") {
            inp.yaw = 1.0;
        }
        if k("arrowleft") {
            inp.rol = -1.0;
        }
        if k("arrowright") {
            inp.rol = 1.0;
        }
        if k("arrowup") {
            inp.pit = 1.0;
        }
        if k("arrowdown") {
            inp.pit = -1.0;
        }
        self.s.input(&inp);
    }
    /// Run up to `sec` seconds; stop early (returning true) when `f` returns true.
    fn run(&mut self, sec: f64, mut f: impl FnMut(&mut T) -> bool) -> bool {
        let n = (sec / DT).round() as u64;
        for i in 0..n {
            if i % 4 == 0 {
                self.read_inputs(DT * 4.0);
            }
            self.s.step(DT);
            self.t += DT;
            if f(self) {
                return true;
            }
        }
        false
    }
    fn wait(&mut self, sec: f64) {
        self.run(sec, |_| false);
    }
    fn key(&mut self, k: &'static str, down: bool) {
        if down {
            self.keys.insert(k);
            if ["w", "s", "a", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].contains(&k) {
                self.s.command(Command::Takeover);
            }
        } else {
            self.keys.remove(k);
        }
    }
    fn stick_touch(&mut self) {
        self.s.command(Command::Takeover);
    }
    fn alt(&self) -> f64 {
        self.s.h.alt()
    }
    fn hs(&self) -> f64 {
        self.s.h.v.hyp_xz()
    }
    fn mode(&self) -> Option<Mode> {
        self.s.auto.as_ref().map(|a| a.mode)
    }
    fn last_toast(&self) -> String {
        self.s.last_toast.as_ref().map(|t| format!("{} | {}", t.title, t.sub)).unwrap_or_default()
    }
    fn cue(&self) -> String {
        strip_tags(&self.s.cue())
    }
    fn takeoff_hold(&mut self) {
        self.s.command(Command::Takeoff);
        self.run(25.0, |t| t.mode() == Some(Mode::Hold));
    }
}

fn strip_tags(s: &str) -> String {
    let mut o = String::new();
    let mut tag = false;
    for c in s.chars() {
        match c {
            '<' => tag = true,
            '>' => tag = false,
            c if !tag => o.push(c),
            _ => {}
        }
    }
    o
}

#[derive(Default)]
struct Stat {
    n: f64,
    s: f64,
    s2: f64,
    mx: f64,
    mn: f64,
}
impl Stat {
    fn new() -> Stat {
        Stat { mx: -1e9, mn: 1e9, ..Default::default() }
    }
    fn add(&mut self, x: f64) {
        self.n += 1.0;
        self.s += x;
        self.s2 += x * x;
        self.mx = self.mx.max(x);
        self.mn = self.mn.min(x);
    }
    fn mean(&self) -> f64 {
        self.s / self.n
    }
    fn sd(&self) -> f64 {
        (self.s2 / self.n - self.mean().powi(2)).max(0.0).sqrt()
    }
}

// =====================================================================================
// test.js: takeoff, 6 circuit laps, land, circuit until the 15% failsafe, failsafe landing
// =====================================================================================
#[test]
fn t1_takeoff_circuit_land_failsafe() {
    let mut t = T::new(11);
    t.wait(0.2);
    t.s.command(Command::Takeoff);
    t.wait(15.0);
    let cue = t.cue();
    println!("RUST TAKEOFF alt {:.2} m vs {:.2} | {}", t.alt(), t.s.h.v.y, cue);
    assert_eq!(t.mode(), Some(Mode::Hold));
    assert!((t.alt() - 3.0).abs() < 0.15, "hold altitude {}", t.alt());
    assert!(cue.contains("hover hold at 3 m"), "{cue}");

    t.s.command(Command::Circuit);
    let t0 = t.t;
    let (mut mn, mut mx, mut amn, mut amx) = (1e9f64, 0f64, 1e9f64, 0f64);
    let lap_toasts0 = t.s.toast_count;
    let mut lap_toasts = 0;
    let mut seen = t.s.toast_count;
    t.run(400.0, |t| {
        if t.s.toast_count != seen {
            seen = t.s.toast_count;
            if t.s.last_toast.as_ref().is_some_and(|x| x.title.starts_with("Lap ")) {
                lap_toasts += 1;
            }
        }
        let Some(a) = &t.s.auto else { return true };
        if a.mode != Mode::Circuit {
            return true;
        }
        if a.li >= 0 && t.t - t0 > 10.0 {
            let s = t.hs();
            mn = mn.min(s);
            mx = mx.max(s);
            amn = amn.min(t.alt());
            amx = amx.max(t.alt());
        }
        a.laps >= 6
    });
    let _ = lap_toasts0;
    let a = t.s.auto.as_ref().unwrap();
    let lt: Vec<String> = a.lap_times.iter().map(|x| format!("{x:.1}")).collect();
    println!(
        "RUST CIRCUIT laps {} lap times {} s | ground speed {:.2}-{:.2} m/s | alt {:.2}-{:.2} m | bat {:.0}% | lap toasts {}",
        a.laps,
        lt.join(", "),
        mn,
        mx,
        amn,
        amx,
        t.s.h.bat * 100.0,
        lap_toasts
    );
    assert_eq!(a.laps, 6);
    assert_eq!(lap_toasts, 6);
    for &x in &a.lap_times[1..] {
        assert!((13.5..15.0).contains(&x), "lap time {x}");
    }
    assert!((13.5..15.5).contains(&a.lap_times[0]));
    assert!(mn > 4.2 && mx < 5.3, "speed {mn}..{mx}");
    assert!(amn > 4.9 && amx < 5.1, "alt {amn}..{amx}");

    t.s.command(Command::Land);
    let landings0 = t.s.landings;
    t.run(60.0, |t| t.s.landings > landings0);
    println!("RUST LAND touchdown {:.2} m/s | {} | crashed {}", t.s.h.last_impact, t.last_toast(), t.s.h.crashed);
    assert!(!t.s.h.crashed);
    assert!(t.s.h.last_impact < 0.5);
    assert!(t.last_toast().contains("on the pad"));

    // circuit until the low-battery failsafe takes over
    t.wait(3.0);
    t.s.command(Command::Takeoff);
    t.wait(15.0);
    t.s.command(Command::Circuit);
    let tc = t.t;
    let mut laps_before = 0;
    let mut fired = None;
    t.run(900.0, |t| {
        if let Some(a) = &t.s.auto {
            if a.mode == Mode::Land && a.fs {
                fired = Some((t.t - tc, laps_before, t.s.h.bat));
                return true;
            }
            laps_before = a.laps;
        }
        t.s.h.crashed
    });
    let (ft, fl, fb) = fired.expect("failsafe never fired");
    println!(
        "RUST FAILSAFE fired after {:.0} s of circuit ({:.1} min), {} laps, true charge {:.1}% | cue: {}",
        ft,
        ft / 60.0,
        fl,
        fb * 100.0,
        t.cue()
    );
    assert!((0.135..0.155).contains(&fb));
    assert!(t.cue().contains("Battery low: autopilot landing on the pad"));
    let landings0 = t.s.landings;
    t.run(60.0, |t| t.s.landings > landings0);
    println!("RUST FAILSAFE LAND touchdown {:.2} m/s | {} | crashed {}", t.s.h.last_impact, t.last_toast(), t.s.h.crashed);
    assert!(!t.s.h.crashed && t.s.h.last_impact < 0.6);
}

// =====================================================================================
// test2.js: the heli stays put on the pad disarmed, at idle and at 30% throttle in wind
// =====================================================================================
fn ground_hold(wind: usize) -> Vec<(f64, f64, f64)> {
    let mut t = T::new(5);
    t.s.set.wind = wind;
    let pos = |t: &T| (t.s.h.p.x, t.s.h.p.z, deg(heading_of(t.s.h.q)));
    let mut out = vec![];
    t.wait(30.0);
    out.push(pos(&t));
    t.s.command(Command::Arm);
    t.wait(30.0);
    out.push(pos(&t));
    t.s.sticks.thr = 0.3;
    t.wait(20.0);
    out.push(pos(&t));
    t.s.sticks.thr = 0.4;
    t.wait(20.0);
    out.push(pos(&t));
    out
}
fn deg(r: f64) -> f64 {
    r * 57.29578
}

#[test]
fn t2_ground_stays_put() {
    for (w, name) in [(1, "breeze"), (2, "gusty")] {
        let r = ground_hold(w);
        for (lbl, (x, z, hd)) in ["disarmed 30s", "armed idle 30s", "thr 30% 20s", "thr 40% (light) 20s"].iter().zip(r.iter()) {
            println!("RUST GROUND {name} {lbl}: x {x:.3} z {z:.3} hdg {hd:.1}°");
        }
        for &(x, z, hd) in &r[..3] {
            assert!(x == 0.0 && z == 0.0, "{name}: moved to {x},{z}");
            assert!(hd.abs() < 3.0);
        }
    }
}

// =====================================================================================
// test3.js: headspeed sag, steady autorotation, scripted flares, yaw kick, bailout
// =====================================================================================
fn climb_to(t: &mut T, alt: f64) {
    t.s.command(Command::Takeoff);
    t.run(20.0, |t| t.mode() == Some(Mode::Hold));
    t.wait(2.0);
    t.s.cancel_auto(false);
    t.s.sticks.thr = 0.62;
    t.run(30.0, |t| t.alt() > alt - 2.0);
    t.s.sticks.thr = 0.45;
    t.wait(3.0);
}

fn sag(bat: f64) -> (f64, f64) {
    let mut t = T::fresh(21, false, false);
    climb_to(&mut t, 15.0);
    t.s.h.bat = bat;
    t.s.h.fs15 = true;
    t.s.h.fs8 = true;
    let meas = |t: &mut T, c: f64| {
        t.s.sticks.thr = c;
        t.wait(1.5);
        let (mut s, mut n) = (0.0, 0.0);
        t.run(1.0, |t| {
            s += t.s.h.rpm;
            n += 1.0;
            false
        });
        s / n
    };
    (meas(&mut t, 0.45), meas(&mut t, 1.0))
}

#[test]
fn t3_headspeed_sag() {
    let (h1, f1) = sag(1.0);
    let (h2, f2) = sag(0.15);
    println!("RUST SAG battery 100%: hover-collective rpm {h1:.2}, full-collective rpm {f1:.2}");
    println!("RUST SAG battery 15%: hover-collective rpm {h2:.2}, full-collective rpm {f2:.2}");
    assert!(h1 > 0.98 && h2 > 0.98);
    assert!((0.93..0.99).contains(&f1), "{f1}");
    assert!((0.86..0.93).contains(&f2), "{f2}");
}

fn steady_auto(c: f64) -> (f64, f64, f64) {
    let mut t = T::fresh(31, false, false);
    climb_to(&mut t, 60.0);
    t.s.command(Command::Hold);
    t.s.sticks.thr = c;
    t.wait(4.0);
    let (mut s, mut r, mut n) = (0.0, 0.0, 0.0);
    t.run(1.5, |t| {
        s += t.s.h.v.y;
        r += t.s.h.rpm;
        n += 1.0;
        false
    });
    (-s / n, r / n, t.alt())
}

#[test]
fn t3_steady_autorotation() {
    for c in [0.08, 0.0, 0.12, 0.18] {
        let (sink, rpm, alt) = steady_auto(c);
        println!("RUST AUTOROT coll {c}: steady sink {sink:.2} m/s, rpm {rpm:.2} at {alt:.1} m");
        if c == 0.08 {
            assert!((7.0..8.0).contains(&sink), "{sink}");
            assert!((0.97..1.03).contains(&rpm), "{rpm}");
        }
    }
}

struct ArRes {
    crashed: bool,
    vs: f64,
    rpm: f64,
    flare_dur: f64,
    vmax: f64,
}

fn auto_run(seed: u64, coll_auto: f64, flare_alt: f64, flare_coll: f64) -> ArRes {
    let mut t = T::fresh(seed, false, false);
    climb_to(&mut t, 25.0);
    t.s.command(Command::Hold);
    t.s.sticks.thr = coll_auto;
    let (mut flared, mut t_flare, mut vmax) = (false, 0.0, 0.0f64);
    let mut res = None;
    t.run(40.0, |t| {
        let a = t.alt();
        vmax = vmax.max(-t.s.h.v.y);
        if !flared && flare_alt > 0.0 && a < flare_alt {
            flared = true;
            t_flare = t.t;
            t.s.sticks.thr = flare_coll;
        }
        if t.s.h.crashed || (t.s.h.grounded && t.s.h.airborne) {
            res = Some(ArRes {
                crashed: t.s.h.crashed,
                vs: t.s.h.last_impact,
                rpm: t.s.h.rpm,
                flare_dur: if flared { t.t - t_flare } else { 0.0 },
                vmax,
            });
            return true;
        }
        false
    });
    res.expect("no touchdown")
}

#[test]
fn t3_scripted_flares() {
    let cases = [
        ("scripted flare", 0.08, 3.0, 0.75),
        ("scripted flare 2", 0.08, 3.0, 0.75),
        ("late flare at 2 m", 0.08, 2.0, 0.75),
        ("no flare", 0.08, 0.0, 0.0),
        ("hover collective held after cut", 0.45, 3.0, 0.75),
    ];
    for (i, (name, c, fa, fc)) in cases.iter().enumerate() {
        let r = auto_run(40 + i as u64, *c, *fa, *fc);
        println!(
            "RUST FLARE [{name}] {} touchdown {:.2} m/s, rpm at touchdown {:.2}, flare-to-ground {:.2} s, max sink {:.1} m/s",
            if r.crashed { "CRASH" } else { "landed" },
            r.vs,
            r.rpm,
            r.flare_dur,
            r.vmax
        );
        match i {
            0 | 1 => assert!(!r.crashed && r.vs < 2.5, "{name}: {}", r.vs),
            _ => assert!(r.crashed, "{name} should crash"),
        }
    }
}

#[test]
fn t3_yaw_kick_and_bailout() {
    let mut t = T::fresh(51, false, false);
    climb_to(&mut t, 25.0);
    let h0 = deg(heading_of(t.s.h.q));
    let mut max_d = 0.0f64;
    t.s.command(Command::Hold);
    t.s.sticks.thr = 0.08;
    t.run(2.0, |t| {
        let d = deg(wrap((deg(heading_of(t.s.h.q)) - h0) / 57.29578));
        if d.abs() > max_d.abs() {
            max_d = d;
        }
        false
    });
    t.s.command(Command::Hold);
    t.s.sticks.thr = 0.6;
    let a0 = t.alt();
    t.wait(3.0);
    println!("RUST YAWKICK max heading change {max_d:.1}° in 2 s");
    println!(
        "RUST BAILOUT after 2 s of auto: rpm {:.2} alt {:.1} -> {:.1}, softStart {}",
        t.s.h.rpm,
        a0,
        t.alt(),
        t.s.h.soft_start
    );
    assert!(max_d.abs() > 0.5 && max_d.abs() < 10.0);
    assert!(t.s.h.rpm > 0.97 && !t.s.h.soft_start);
}

// =====================================================================================
// test4.js: crash protection scenarios and the battery failsafe takeover (IDEAL and REAL)
// =====================================================================================
fn hover_at(t: &mut T, alt: f64) {
    t.s.command(Command::Takeoff);
    t.run(25.0, |t| t.mode() == Some(Mode::Hold));
    t.wait(2.0);
    if alt > 3.5 {
        t.s.auto.as_mut().unwrap().alt = alt;
        t.run(30.0, |t| (t.alt() - alt).abs() < 0.3 && t.s.h.v.y.abs() < 0.3);
        t.wait(2.0);
    }
    t.s.cancel_auto(false);
    t.wait(0.5);
}

#[derive(Debug)]
struct Watch {
    trig: Option<(f64, String)>,
    rec: Option<f64>,
    min_alt: f64,
    crashed: bool,
}

fn watch(t: &mut T, sec: f64) -> Watch {
    let before = t.s.rescues;
    let t0 = t.t;
    let mut w = Watch { trig: None, rec: None, min_alt: 1e9, crashed: false };
    t.run(sec, |t| {
        w.min_alt = w.min_alt.min(t.alt());
        if w.trig.is_none() && t.s.rescues > before {
            w.trig = Some((t.t - t0, t.last_toast()));
        }
        if (w.trig.is_some() || t.s.rescues > 0) && w.rec.is_none() && t.mode() == Some(Mode::Hold) {
            w.rec = Some(t.t - t0);
            return true;
        }
        if t.s.h.crashed {
            w.crashed = true;
            return true;
        }
        false
    });
    w
}

fn report(tag: &str, name: &str, r: &Watch) {
    let trig = match &r.trig {
        Some((tt, why)) => format!("TRIGGERED at +{tt:.2} s ({why})"),
        None => "no trigger".into(),
    };
    let end = if r.crashed {
        "CRASHED".to_string()
    } else if let Some(rec) = r.rec {
        format!("recovered to hover hold at +{rec:.2} s ({:.2} s after trigger)", rec - r.trig.as_ref().map_or(0.0, |x| x.0))
    } else {
        "no recovery".into()
    };
    println!("RUST PROTECT {tag} {name}: {trig} | min alt {:.2} m | {end}", r.min_alt);
}

fn slam(seed: u64, protect: bool, real: bool) -> Watch {
    let mut t = T::fresh(seed, protect, real);
    hover_at(&mut t, 8.0);
    t.s.command(Command::Level); // rate mode: full roll actually rolls the heli over
    t.key("arrowright", true);
    let r0 = watch(&mut t, 1.2);
    t.key("arrowright", false);
    let mut r = watch(&mut t, 12.0);
    if r0.trig.is_some() {
        r.trig = r0.trig;
        r.min_alt = r.min_alt.min(r0.min_alt);
        r.rec = r.rec.map(|x| x + 1.2);
    }
    r.crashed |= r0.crashed;
    r
}

fn protect_suite(real: bool, seed: u64) {
    let tag = if real { "REAL" } else { "IDEAL" };
    // (1) full roll right for 1.2 s at 8 m in rate mode
    let r = slam(seed, true, real);
    report(tag, "(1) full roll right 1.2 s at 8 m", &r);
    assert!(r.trig.as_ref().is_some_and(|x| x.1.contains("Over-banked")), "{r:?}");
    assert!(!r.crashed && r.rec.is_some());

    // (2) throttle hold at 20 m with hover collective held
    {
        let mut t = T::fresh(seed + 1, true, real);
        hover_at(&mut t, 20.0);
        let c = t.s.sticks.thr;
        let a0 = t.alt();
        t.s.command(Command::Hold);
        t.s.sticks.thr = c;
        let r = watch(&mut t, 15.0);
        report(tag, &format!("(2) THR HOLD at {a0:.1} m, hover collective {c:.2} held"), &r);
        println!("RUST PROTECT {tag}     hold after rescue: {}, rpm {:.2}", t.s.h.hold, t.s.h.rpm);
        assert!(r.trig.as_ref().is_some_and(|x| x.1.contains("Headspeed")), "{r:?}");
        assert!(!r.crashed && r.rec.is_some() && !t.s.h.hold && t.s.h.rpm > 0.95);
    }
    // (3) collective dropped to 0 at 4 m
    {
        let mut t = T::fresh(seed + 2, true, real);
        hover_at(&mut t, 4.0);
        t.s.sticks.thr = 0.0;
        t.key("s", true);
        let r = watch(&mut t, 12.0);
        t.key("s", false);
        report(tag, "(3) collective dropped to 0 at 4 m", &r);
        assert!(r.trig.as_ref().is_some_and(|x| x.1.contains("Sinking")), "{r:?}");
        assert!(!r.crashed && r.rec.is_some() && r.min_alt > 2.0);
    }
    // (4a) autopilot land: no trigger
    {
        let mut t = T::fresh(seed + 3, true, real);
        hover_at(&mut t, 5.0);
        t.s.command(Command::Land);
        let r = watch(&mut t, 40.0);
        println!(
            "RUST PROTECT {tag} (4a) autopilot LAND: {}, touchdown {:.2} m/s, crashed {}",
            if r.trig.is_some() { "FALSE TRIGGER" } else { "no trigger" },
            t.s.h.last_impact,
            t.s.h.crashed
        );
        assert!(r.trig.is_none() && !t.s.h.crashed && t.s.h.last_impact < 0.6);
    }
    // (4b) manual gentle landing
    {
        let mut t = T::fresh(seed + 4, true, real);
        hover_at(&mut t, 3.0);
        let _ = watch(&mut t, 30.0);
        let c0 = t.s.sticks.thr;
        t.s.sticks.thr = c0 - 0.03;
        let rr = watch(&mut t, 25.0);
        println!(
            "RUST PROTECT {tag} (4b) manual gentle landing: {}, touchdown {:.2} m/s, crashed {}",
            if rr.trig.is_some() { "FALSE TRIGGER" } else { "no trigger" },
            t.s.h.last_impact,
            t.s.h.crashed
        );
        assert!(rr.trig.is_none() && !t.s.h.crashed && t.s.h.last_impact < 1.0);
    }
    // (4c) manual autorotation with the headspeed kept up and a flare
    {
        let mut t = T::fresh(seed + 5, true, real);
        hover_at(&mut t, 25.0);
        t.s.command(Command::Hold);
        t.s.sticks.thr = 0.08;
        let before = t.s.rescues;
        let (mut flare, mut trig_alt, mut min_a) = (false, None, 99.0f64);
        t.run(30.0, |t| {
            if !flare && t.alt() < 3.0 {
                flare = true;
                t.s.sticks.thr = 0.75;
            }
            if trig_alt.is_none() && t.s.rescues > before {
                trig_alt = Some(t.alt());
            }
            min_a = min_a.min(t.alt());
            if trig_alt.is_some() && t.mode() == Some(Mode::Hold) {
                return true;
            }
            t.s.h.crashed || (t.s.h.grounded && t.s.h.airborne)
        });
        println!(
            "RUST PROTECT {tag} (4c) manual autorotation, rpm kept up: {}, touchdown {:.2} m/s, crashed {}",
            match trig_alt {
                None => "no trigger".to_string(),
                Some(a) => format!(
                    "trigger at {a:.2} m, bailout min alt {min_a:.2} m, ends in {}",
                    t.mode().map(|m| format!("{m:?}").to_lowercase()).unwrap_or("manual".into())
                ),
            },
            t.s.h.last_impact,
            t.s.h.crashed
        );
        assert!(!t.s.h.crashed);
    }
    // (5) protection off: the same slam crashes
    let r = slam(seed + 6, false, real);
    report(tag, "(5) PROTECT OFF, full roll right 1.2 s at 8 m", &r);
    assert!(r.trig.is_none() && r.crashed);
}

fn battery_takeover(real: bool, seed: u64) {
    let tag = if real { "REAL" } else { "IDEAL" };
    let mut t = T::fresh(seed, true, real);
    hover_at(&mut t, 5.0);
    t.s.h.bat = 0.18;
    let mut fired15 = None;
    t.run(120.0, |t| {
        if t.s.auto.as_ref().is_some_and(|a| a.mode == Mode::Land && a.fs) {
            fired15 = Some((t.s.h.bat, t.s.est.bat));
            return true;
        }
        false
    });
    let (b15, e15) = fired15.expect("15% failsafe not fired");
    println!("RUST BATTERY {tag} 15% failsafe: fired, true charge {:.1}%, est {:.1}% | {}", b15 * 100.0, e15 * 100.0, t.cue());
    if real {
        assert!((0.135..0.17).contains(&b15));
    } else {
        assert!((0.14..0.15).contains(&b15));
    }
    assert!((0.14..0.15).contains(&e15));
    t.stick_touch();
    println!("RUST BATTERY {tag}   stick touch -> autopilot {}", if t.s.auto.is_some() { "on" } else { "off" });
    assert!(t.s.auto.is_none());
    // hover manually with the collective the autopilot left
    t.s.start_auto(Mode::Takeoff, false, false, false, false);
    t.wait(0.1);
    {
        let a = t.s.auto.as_mut().unwrap();
        a.mode = Mode::Hold;
        a.alt = 5.0;
    }
    t.wait(3.0);
    t.s.cancel_auto(false);
    let (mut fired8, mut ignored, mut tries) = (None, 0, 0);
    t.run(300.0, |t| {
        let forced = t.s.auto.as_ref().is_some_and(|a| a.forced);
        if fired8.is_none() && forced {
            fired8 = Some((t.s.h.bat, t.s.est.bat, t.cue()));
            println!("RUST BATTERY {tag}   8% fired at alt {:.2} m, x {:.1}", t.alt(), t.s.h.p.x);
        }
        if fired8.is_some() && helisim::jsonw::js_round(t.t * 4.0) as i64 % 4 == 0 && tries < 20 {
            tries += 1;
            t.stick_touch();
            t.key("w", true);
            t.key("w", false);
            if t.s.auto.as_ref().is_some_and(|a| a.forced) {
                ignored += 1;
            }
        }
        t.s.h.grounded && t.s.h.airborne && fired8.is_some()
    });
    let (b8, e8, cue8) = fired8.expect("8% not fired");
    println!(
        "RUST BATTERY {tag} 8% forced landing: fired, true {:.1}%, est {:.1}% | {} | stick/key inputs ignored {}/{}",
        b8 * 100.0,
        e8 * 100.0,
        cue8,
        ignored,
        tries
    );
    println!("RUST BATTERY {tag}   touchdown {:.2} m/s, crashed {} | {}", t.s.h.last_impact, t.s.h.crashed, t.last_toast());
    assert!(cue8.contains("forced landing"));
    assert_eq!((ignored, tries), (20, 20));
    assert!(!t.s.h.crashed && t.s.h.last_impact < 3.0);
}

#[test]
fn t4_protect_ideal() {
    protect_suite(false, 100);
}
#[test]
fn t4_protect_real() {
    protect_suite(true, 200);
}
#[test]
fn t4_battery_ideal() {
    let seeds: Vec<u64> = std::env::var("SEEDS").ok().map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![300]);
    for s in seeds {
        battery_takeover(false, s);
    }
}
#[test]
fn t4_battery_real() {
    battery_takeover(true, 400);
}

// =====================================================================================
// test5.js: REAL sensor stats, GPS glitch, battery telemetry, AUTOROTATE autopilot
// =====================================================================================
struct Errs {
    pos: f64,
    alt: f64,
    tilt: f64,
    hdg: f64,
}
fn errs(s: &Sim) -> Errs {
    let (h, e) = (&s.h, &s.est);
    let (ut, ue) = (up_of(h.q), up_of(e.q));
    Errs {
        pos: (e.p.x - h.p.x).hypot(e.p.z - h.p.z),
        alt: e.p.y - h.p.y,
        tilt: deg(ut.dot(ue).min(1.0).acos()),
        hdg: deg(wrap(heading_of(e.q) - heading_of(h.q)).abs()),
    }
}

#[test]
fn t5_real_hover() {
    let mut t = T::fresh(500, true, true);
    t.takeoff_hold();
    t.wait(3.0);
    let a = t.s.auto.as_ref().unwrap();
    let (ax, az) = (a.anchor.x, a.anchor.z);
    let (mut dx, mut dz, mut dist, mut ep, mut ea, mut et, mut eh, mut rpm, mut alt) =
        (Stat::new(), Stat::new(), Stat::new(), Stat::new(), Stat::new(), Stat::new(), Stat::new(), Stat::new(), Stat::new());
    t.run(30.0, |t| {
        let h = &t.s.h;
        dx.add(h.p.x);
        dz.add(h.p.z);
        dist.add((h.p.x - ax).hypot(h.p.z - az));
        let e = errs(&t.s);
        ep.add(e.pos);
        ea.add(e.alt);
        et.add(e.tilt);
        eh.add(e.hdg);
        rpm.add(t.s.est.rpm);
        alt.add(t.alt());
        false
    });
    println!(
        "RUST REAL hover: true pos std x {:.2} z {:.2} m, max dist from anchor {:.2} m, true alt {:.2}-{:.2} m",
        dx.sd(),
        dz.sd(),
        dist.mx,
        alt.mn,
        alt.mx
    );
    println!(
        "RUST REAL hover est error: pos mean {:.2} max {:.2} m | alt mean {:.2} range {:.2}..{:.2} m | tilt mean {:.2} max {:.2}° | heading mean {:.2} max {:.2}°",
        ep.mean(),
        ep.mx,
        ea.mean(),
        ea.mn,
        ea.mx,
        et.mean(),
        et.mx,
        eh.mean(),
        eh.mx
    );
    println!("RUST REAL hover rpm telemetry {:.2}-{:.2}", rpm.mn, rpm.mx);
    assert!(dx.sd() < 0.4 && dz.sd() < 0.4);
    assert!(dist.mx < 1.5);
    assert!(alt.mn > 2.75 && alt.mx < 3.25);
    assert!(ep.mean() < 0.6 && ea.mean().abs() < 0.1 && et.mx < 3.0 && eh.mx < 4.0);
    assert!(rpm.mn > 0.95 && rpm.mx < 1.05);
}

#[test]
fn t5_real_circuit() {
    let mut t = T::fresh(510, true, true);
    t.takeoff_hold();
    t.s.command(Command::Circuit);
    let (mut ep, mut et, mut sp, mut rpm, mut al) = (Stat::new(), Stat::new(), Stat::new(), Stat::new(), Stat::new());
    let t0 = t.t;
    t.run(120.0, |t| {
        let Some(a) = &t.s.auto else { return true };
        if a.mode != Mode::Circuit {
            return true;
        }
        if a.li >= 0 && t.t - t0 > 10.0 {
            let e = errs(&t.s);
            ep.add(e.pos);
            sp.add(t.hs());
            rpm.add(t.s.h.rpm);
            al.add(t.alt());
            if t.s.h.w.y.abs() > 0.25 {
                et.add(e.tilt);
            }
        }
        a.laps >= 3
    });
    let a = t.s.auto.as_ref().unwrap();
    let lt: Vec<String> = a.lap_times.iter().map(|x| format!("{x:.1}")).collect();
    println!(
        "RUST REAL circuit: laps {}, times {} s | est pos err mean {:.2} max {:.2} m | tilt err in corners mean {:.2} max {:.2}°",
        a.laps,
        lt.join(", "),
        ep.mean(),
        ep.mx,
        et.mean(),
        et.mx
    );
    println!(
        "RUST REAL circuit: true ground speed {:.2}-{:.2} m/s, true alt {:.2}-{:.2} m, true rpm {:.2}-{:.2}, crashed {}",
        sp.mn,
        sp.mx,
        al.mn,
        al.mx,
        rpm.mn,
        rpm.mx,
        t.s.h.crashed
    );
    assert_eq!(a.laps, 3);
    assert!(!t.s.h.crashed);
    assert!(ep.mean() < 1.0 && al.mn > 4.3 && al.mx < 5.5);
}

#[test]
fn t5_real_land_runs() {
    for i in 0..5 {
        let mut t = T::fresh(520 + i, true, true);
        t.takeoff_hold();
        t.wait(5.0);
        t.s.command(Command::Land);
        t.run(40.0, |t| (t.s.h.grounded && t.s.h.airborne) || t.s.h.crashed);
        let d = t.s.h.p.x.hypot(t.s.h.p.z);
        println!(
            "RUST REAL land run {}: touchdown {:.2} m/s, {:.2} m from the pad centre, crashed {}",
            i + 1,
            t.s.h.last_impact,
            d,
            t.s.h.crashed
        );
        assert!(!t.s.h.crashed && t.s.h.last_impact < 0.6 && d < 1.0);
    }
}

#[test]
fn t5_real_gps_glitch() {
    let mut t = T::fresh(530, true, true);
    t.takeoff_hold();
    t.wait(5.0);
    let p0 = t.s.h.p;
    t.s.force_glitch(3.5, 2.5);
    let mut mx = 0.0f64;
    t.run(15.0, |t| {
        mx = mx.max((t.s.h.p.x - p0.x).hypot(t.s.h.p.z - p0.z));
        false
    });
    let back = (t.s.h.p.x - p0.x).hypot(t.s.h.p.z - p0.z);
    println!(
        "RUST REAL glitch 3.5 m / 2.5 s: true heli moved up to {:.2} m, back to {:.2} m after 15 s, crashed {}, mode {:?}",
        mx,
        back,
        t.s.h.crashed,
        t.mode()
    );
    assert!(mx < 4.5 && back < 1.0 && !t.s.h.crashed && t.mode() == Some(Mode::Hold));
}

#[test]
fn t5_real_battery_telemetry() {
    let mut t = T::fresh(540, true, true);
    t.takeoff_hold();
    t.wait(2.0);
    t.s.cancel_auto(false);
    let c = t.s.sticks.thr;
    t.s.h.bat = 0.2;
    t.wait(3.0);
    let e0 = t.s.est.bat;
    let (mut mn, mut fs) = (1.0f64, false);
    t.s.sticks.thr = 1.0;
    t.run(3.0, |t| {
        mn = mn.min(t.s.est.bat);
        fs |= t.s.auto.is_some();
        false
    });
    t.s.sticks.thr = 0.4;
    let mut mx = 0.0f64;
    t.run(3.0, |t| {
        mx = mx.max(t.s.est.bat);
        fs |= t.s.auto.is_some();
        false
    });
    println!(
        "RUST REAL battery: true charge 20%: est {:.1}% in hover, dips to {:.1}% in a 3 s full-collective climb, recovers to {:.1}% | failsafe fired: {} | pack {:.1} V",
        e0 * 100.0,
        mn * 100.0,
        mx * 100.0,
        fs,
        t.s.sens.st.v
    );
    assert!(!fs, "momentary sag must not trigger the failsafe");
    assert!(mn < e0 && mx > mn);
    t.s.sticks.thr = c;
    t.s.start_auto(Mode::Takeoff, false, false, false, false);
    t.wait(0.1);
    let alt = t.alt();
    {
        let a = t.s.auto.as_mut().unwrap();
        a.mode = Mode::Hold;
        a.alt = alt;
    }
    t.s.h.bat = 0.19;
    let mut fired = None;
    t.run(200.0, |t| {
        if t.s.auto.as_ref().is_some_and(|a| a.fs) {
            fired = Some((t.s.h.bat, t.s.est.bat));
            return true;
        }
        false
    });
    let (b, e) = fired.expect("15% failsafe in hover not fired");
    println!("RUST REAL battery: 15% failsafe in hover: true charge {:.1}% (estimate {:.1}%)", b * 100.0, e * 100.0);
    assert!((0.14..0.15).contains(&e) && (0.135..0.17).contains(&b));
}

fn autorotate_runs(real: bool) {
    let tag = if real { "REAL" } else { "IDEAL" };
    for i in 0..5 {
        let mut t = T::fresh(600 + i + if real { 10 } else { 0 }, true, real);
        t.takeoff_hold();
        t.wait(2.0);
        let r0 = t.s.rescues;
        t.s.command(Command::Autorot);
        let mut phases: Vec<helisim::autopilot::Phase> = vec![];
        t.run(150.0, |t| {
            if let Some(a) = &t.s.auto {
                if a.mode == Mode::Autorot && phases.last() != Some(&a.phase) {
                    phases.push(a.phase);
                }
            }
            t.s.h.crashed || (t.s.auto.is_none() && t.s.h.grounded)
        });
        let r = t.s.h.auto_result.clone().unwrap_or_else(|| panic!("{tag} run {}: no result, phases {phases:?}", i + 1));
        println!(
            "RUST AUTOROTATE {tag} run {}: cut {:.1} m | glide sink {:.2} m/s fwd {:.2} m/s | rpm {:.2}-{:.2} | touchdown {:.2} m/s drift {:.2} m/s | {:.2} m from pad | rescues {} | crashed {} | hold {}",
            i + 1,
            r.cut_alt,
            -r.glide_vy / r.glide_n as f64,
            r.glide_hs / r.glide_n as f64,
            r.min_rpm,
            r.max_rpm,
            r.vs,
            r.drift,
            r.d_pad,
            t.s.rescues - r0,
            t.s.h.crashed,
            t.s.h.hold
        );
        assert!(!t.s.h.crashed);
        assert_eq!(t.s.rescues, r0);
        assert!(r.vs < if real { 2.5 } else { 1.5 }, "touchdown {}", r.vs);
        assert_eq!(phases.len(), 5, "{phases:?}");
    }
}

#[test]
fn t5_autorotate_ideal() {
    autorotate_runs(false);
}
#[test]
fn t5_autorotate_real() {
    autorotate_runs(true);
}

// =====================================================================================
// determinism and the wasm frame
// =====================================================================================
#[test]
fn t6_deterministic_with_seed() {
    let run = |seed| {
        let mut s = Sim::new(seed);
        s.command(Command::Sensors);
        s.command(Command::Takeoff);
        s.steps(240 * 20);
        (s.h.p, s.est.p)
    };
    assert_eq!(run(7), run(7));
    assert_ne!(run(7).1, run(8).1);
}

#[test]
fn t6_frame_json() {
    use helisim::wasm::*;
    sim_init(3.0);
    sim_cmd(Command::Takeoff as u32);
    sim_input(0.0, 0.0, 0.0, 0.0, -1.0, 0, 0.0, 0.0, 0.0, 0.0);
    sim_step(240 * 6);
    let n = sim_frame() as usize;
    let s = unsafe { std::str::from_utf8(std::slice::from_raw_parts(sim_out_ptr(), n)).unwrap() };
    assert!(s.starts_with('{') && s.ends_with('}'));
    assert!(s.contains("\"mode\":1") && s.contains("Hover hold"), "{s}");
    // toasts are drained after each frame
    let n = sim_frame() as usize;
    let s2 = unsafe { std::str::from_utf8(std::slice::from_raw_parts(sim_out_ptr(), n)).unwrap() };
    assert!(s2.contains("\"toasts\":[]"));
}

// =====================================================================================
// EKF vs BASIC (REAL sensors), same seeds for both estimators
// =====================================================================================
const EST_SEEDS: [u64; 5] = [700, 701, 702, 703, 704];

fn fresh_est(seed: u64, ekf: bool) -> T {
    let mut t = T::fresh(seed, true, true);
    t.s.set.ekf = ekf;
    t
}
fn est_name(ekf: bool) -> &'static str {
    if ekf {
        "EKF"
    } else {
        "BASIC"
    }
}

/// Estimation error accumulators: horizontal position, |altitude|, tilt, heading.
struct ErrAcc {
    pos: Stat,
    alt: Stat,
    tilt: Stat,
    hdg: Stat,
}
impl ErrAcc {
    fn new() -> ErrAcc {
        ErrAcc { pos: Stat::new(), alt: Stat::new(), tilt: Stat::new(), hdg: Stat::new() }
    }
    fn add(&mut self, s: &Sim) {
        let e = errs(s);
        self.pos.add(e.pos);
        self.alt.add(e.alt.abs());
        self.tilt.add(e.tilt);
        self.hdg.add(e.hdg);
    }
    fn merge(&mut self, o: &ErrAcc) {
        for (a, b) in [(&mut self.pos, &o.pos), (&mut self.alt, &o.alt), (&mut self.tilt, &o.tilt), (&mut self.hdg, &o.hdg)] {
            a.n += b.n;
            a.s += b.s;
            a.s2 += b.s2;
            a.mx = a.mx.max(b.mx);
            a.mn = a.mn.min(b.mn);
        }
    }
    fn line(&self) -> String {
        format!(
            "pos {:.2} (max {:.2}) m | alt {:.3} (max {:.2}) m | tilt {:.2} (max {:.2})° | hdg {:.2} (max {:.2})°",
            self.pos.mean(),
            self.pos.mx,
            self.alt.mean(),
            self.alt.mx,
            self.tilt.mean(),
            self.tilt.mx,
            self.hdg.mean(),
            self.hdg.mx
        )
    }
}

fn check_ekf_health(s: &Sim) {
    let k = &s.sens.ekf;
    assert!(k.init && k.is_finite(), "EKF not finite");
    let (mn, mx, asym) = k.cov_stats();
    assert!(mn > 0.0 && mx < 1e4 && asym < 1e-9, "EKF covariance diag {mn}..{mx} asym {asym}");
}

fn hover_case(ekf: bool) -> (ErrAcc, Stat, f64) {
    let mut all = ErrAcc::new();
    let mut wander = Stat::new();
    let mut wmax = 0.0f64;
    for &seed in &EST_SEEDS {
        let mut t = fresh_est(seed, ekf);
        t.takeoff_hold();
        t.wait(3.0);
        let a = t.s.auto.as_ref().unwrap();
        let (ax, az) = (a.anchor.x, a.anchor.z);
        let mut acc = ErrAcc::new();
        t.run(60.0, |t| {
            acc.add(&t.s);
            let d = (t.s.h.p.x - ax).hypot(t.s.h.p.z - az);
            wander.add(d);
            wmax = wmax.max(d);
            false
        });
        assert!(!t.s.h.crashed && t.mode() == Some(Mode::Hold));
        if ekf {
            check_ekf_health(&t.s);
        }
        all.merge(&acc);
    }
    (all, wander, wmax)
}

#[test]
fn t7_ekf_hover() {
    let mut res = vec![];
    for ekf in [true, false] {
        let (e, w, wmax) = hover_case(ekf);
        println!("RUST EST {} hover 60 s x5: {} | true wander mean {:.2} max {:.2} m", est_name(ekf), e.line(), w.mean(), wmax);
        res.push((e, wmax));
    }
    let (ek, bs) = (&res[0].0, &res[1].0);
    assert!(ek.pos.mean() < 0.8 && ek.alt.mean() < 0.1 && ek.tilt.mx < 3.0 && ek.hdg.mx < 4.0);
    assert!(ek.tilt.mean() < bs.tilt.mean() * 1.1, "EKF tilt {} vs BASIC {}", ek.tilt.mean(), bs.tilt.mean());
    assert!(res[0].1 < 2.0);
}

fn circuit_case(ekf: bool) -> (ErrAcc, Stat, Stat, Stat) {
    let (mut all, mut corner, mut sp, mut al) = (ErrAcc::new(), Stat::new(), Stat::new(), Stat::new());
    for &seed in &EST_SEEDS {
        let mut t = fresh_est(seed, ekf);
        t.takeoff_hold();
        t.s.command(Command::Circuit);
        let t0 = t.t;
        let mut acc = ErrAcc::new();
        t.run(150.0, |t| {
            let Some(a) = &t.s.auto else { return true };
            if a.mode != Mode::Circuit {
                return true;
            }
            if a.li >= 0 && t.t - t0 > 10.0 {
                acc.add(&t.s);
                sp.add(t.hs());
                al.add(t.alt());
                if t.s.h.w.y.abs() > 0.25 {
                    corner.add(errs(&t.s).tilt);
                }
            }
            a.laps >= 3
        });
        let a = t.s.auto.as_ref().expect("circuit ended");
        assert_eq!(a.laps, 3);
        assert!(!t.s.h.crashed);
        if ekf {
            check_ekf_health(&t.s);
        }
        all.merge(&acc);
    }
    (all, corner, sp, al)
}

#[test]
fn t7_ekf_circuit() {
    let mut res = vec![];
    for ekf in [true, false] {
        let (e, c, sp, al) = circuit_case(ekf);
        println!(
            "RUST EST {} circuit 3 laps x5: {} | corner tilt {:.2} (max {:.2})° | speed {:.2}-{:.2} m/s | alt {:.2}-{:.2} m",
            est_name(ekf),
            e.line(),
            c.mean(),
            c.mx,
            sp.mn,
            sp.mx,
            al.mn,
            al.mx
        );
        res.push((e, c, al));
    }
    let (ek, ec, eal) = &res[0];
    assert!(ek.pos.mean() < 1.0 && eal.mn > 4.3 && eal.mx < 5.6);
    assert!(ec.mean() < res[1].1.mean(), "EKF corner tilt should beat BASIC");
}

/// Forced GPS glitch during hover: how far the true heli moves.
fn glitch_case(ekf: bool, m: f64, dur: f64, settle: f64) -> (Stat, Stat, u32, u32, u32) {
    let (mut moved, mut back, mut rej, mut resets, mut rej_in) = (Stat::new(), Stat::new(), 0, 0, 0);
    for &seed in &EST_SEEDS {
        let mut t = fresh_est(seed, ekf);
        t.takeoff_hold();
        t.wait(10.0);
        let p0 = t.s.h.p;
        let (r0, n0) = (t.s.sens.ekf.h.rej_gps, t.s.sens.ekf.h.resets);
        t.s.force_glitch(m, dur);
        let t0g = t.t;
        let mut mx = 0.0f64;
        let mut rin = None;
        t.run(dur + settle, |t| {
            mx = mx.max((t.s.h.p.x - p0.x).hypot(t.s.h.p.z - p0.z));
            // glitched samples are still arriving for 150 ms after the glitch ends
            if rin.is_none() && t.t > t0g + dur + 0.2 {
                rin = Some(t.s.sens.ekf.h.rej_gps - r0);
            }
            false
        });
        rej_in += rin.unwrap_or(0);
        let b = (t.s.h.p.x - p0.x).hypot(t.s.h.p.z - p0.z);
        assert!(!t.s.h.crashed && t.mode() == Some(Mode::Hold), "glitch {m} m: crashed {} mode {:?}", t.s.h.crashed, t.mode());
        if ekf {
            check_ekf_health(&t.s);
            rej += t.s.sens.ekf.h.rej_gps - r0;
            resets += t.s.sens.ekf.h.resets - n0;
        }
        moved.add(mx);
        back.add(b);
    }
    (moved, back, rej, resets, rej_in)
}

#[test]
fn t7_ekf_gps_glitch() {
    let mut res = vec![];
    for ekf in [true, false] {
        let (mv, bk, rej, rs, rin) = glitch_case(ekf, 3.5, 2.5, 12.0);
        println!(
            "RUST EST {} glitch 3.5 m / 2.5 s x5: true heli moved mean {:.2} max {:.2} m | back to {:.2} (max {:.2}) m{}",
            est_name(ekf),
            mv.mean(),
            mv.mx,
            bk.mean(),
            bk.mx,
            if ekf {
                format!(" | GPS rejected {rin} of 125 glitched samples (25 x 5), {rej} in all, resets {rs} (into the glitch after 2 s and back out)")
            } else {
                String::new()
            }
        );
        res.push(mv);
    }
    assert!(res[0].mean() < res[1].mean() * 0.6, "EKF should reject most of the glitch");
    assert!(res[0].mx < 1.5);
}

#[test]
fn t7_ekf_long_glitch_reset() {
    for ekf in [true, false] {
        let (mv, bk, rej, rs, rin) = glitch_case(ekf, 6.0, 8.0, 20.0);
        println!(
            "RUST EST {} glitch 6 m / 8 s x5: true heli moved mean {:.2} max {:.2} m | back to {:.2} (max {:.2}) m after 20 s{}",
            est_name(ekf),
            mv.mean(),
            mv.mx,
            bk.mean(),
            bk.mx,
            if ekf { format!(" | GPS rejected {rin} of 400 glitched samples, {rej} in all, resets {rs}") } else { String::new() }
        );
        if ekf {
            assert!(rs >= 5, "every run should take the reset path");
        }
        assert!(bk.mx < 1.5, "must recover");
    }
}

#[test]
fn t7_ekf_land_autorotate() {
    for ekf in [true, false] {
        let (mut vs, mut dd) = (Stat::new(), Stat::new());
        for &seed in &EST_SEEDS {
            let mut t = fresh_est(seed, ekf);
            t.takeoff_hold();
            t.wait(5.0);
            t.s.command(Command::Land);
            t.run(40.0, |t| (t.s.h.grounded && t.s.h.airborne) || t.s.h.crashed);
            let d = t.s.h.p.x.hypot(t.s.h.p.z);
            assert!(!t.s.h.crashed && t.s.h.last_impact < 0.6 && d < 1.2, "{} land {seed}: {} m/s {d} m", est_name(ekf), t.s.h.last_impact);
            vs.add(t.s.h.last_impact);
            dd.add(d);
        }
        println!(
            "RUST EST {} LAND x5: touchdown {:.2}-{:.2} m/s (mean {:.2}) | {:.2}-{:.2} m from pad centre (mean {:.2})",
            est_name(ekf),
            vs.mn,
            vs.mx,
            vs.mean(),
            dd.mn,
            dd.mx,
            dd.mean()
        );
        let (mut vs, mut dd) = (Stat::new(), Stat::new());
        for &seed in &EST_SEEDS {
            let mut t = fresh_est(seed, ekf);
            t.takeoff_hold();
            t.wait(2.0);
            let r0 = t.s.rescues;
            t.s.command(Command::Autorot);
            t.run(150.0, |t| t.s.h.crashed || (t.s.auto.is_none() && t.s.h.grounded));
            let r = t.s.h.auto_result.clone().unwrap_or_else(|| panic!("{} autorotate {seed}: no result", est_name(ekf)));
            assert!(!t.s.h.crashed && t.s.rescues == r0 && r.vs < 2.5, "{} autorotate {seed}: {} m/s", est_name(ekf), r.vs);
            vs.add(r.vs);
            dd.add(r.d_pad);
        }
        println!(
            "RUST EST {} AUTOROTATE x5: touchdown {:.2}-{:.2} m/s (mean {:.2}) | {:.2}-{:.2} m from pad (mean {:.2}) | 0 rescues, 0 crashes",
            est_name(ekf),
            vs.mn,
            vs.mx,
            vs.mean(),
            dd.mn,
            dd.mx,
            dd.mean()
        );
    }
}

#[test]
fn t7_ekf_protect() {
    for ekf in [true, false] {
        for &seed in &EST_SEEDS {
            let mut t = fresh_est(seed, ekf);
            hover_at(&mut t, 8.0);
            t.s.command(Command::Level);
            t.key("arrowright", true);
            let r0 = watch(&mut t, 1.2);
            t.key("arrowright", false);
            let mut r = watch(&mut t, 12.0);
            if r0.trig.is_some() {
                r.trig = r0.trig;
                r.min_alt = r.min_alt.min(r0.min_alt);
                r.rec = r.rec.map(|x| x + 1.2);
            }
            r.crashed |= r0.crashed;
            report(est_name(ekf), &format!("seed {seed} full roll right 1.2 s at 8 m"), &r);
            assert!(r.trig.as_ref().is_some_and(|x| x.1.contains("Over-banked")), "{r:?}");
            assert!(!r.crashed && r.rec.is_some());
            if ekf {
                check_ekf_health(&t.s);
            }
        }
    }
}

/// 10 minutes: 60 s hover then circuit laps; consistency of the EKF covariance against the truth.
#[test]
fn t7_ekf_ten_minutes() {
    let mut tot = [0.0f64; 5];
    let mut n = 0.0;
    let mut nees = Stat::new();
    let mut sig = [Stat::new(), Stat::new(), Stat::new(), Stat::new()];
    let mut acc = ErrAcc::new();
    for &seed in &EST_SEEDS {
        let mut t = fresh_est(seed, true);
        t.s.h.bat = 1.0;
        t.takeoff_hold();
        let mut phase = 0;
        let t0 = t.t;
        t.run(600.0, |t| {
            t.s.h.bat = t.s.h.bat.max(0.6); // keep the pack up: this test is about the filter
            if phase == 0 && t.t - t0 > 60.0 {
                t.s.command(Command::Circuit);
                phase = 1;
            }
            let (h, k) = (&t.s.h, &t.s.sens.ekf);
            let (ex, ey, ez) = (k.p.x - h.p.x, k.p.y - h.p.y, k.p.z - h.p.z);
            let e = errs(&t.s);
            let ux = (ex / k.sig_x()).abs() < 3.0;
            let uz = (ez / k.sig_z()).abs() < 3.0;
            let uy = (ey / k.sig_hgt()).abs() < 3.0;
            let ut = (e.tilt / 57.29578) < 3.0 * k.sig_tilt();
            let uh = (e.hdg / 57.29578) < 3.0 * k.sig_hdg();
            for (i, ok) in [ux, uz, uy, ut, uh].iter().enumerate() {
                if *ok {
                    tot[i] += 1.0;
                }
            }
            n += 1.0;
            // horizontal position NEES (2 DOF, expected mean 2)
            nees.add((ex / k.sig_x()).powi(2) + (ez / k.sig_z()).powi(2));
            sig[0].add(k.sig_pos());
            sig[1].add(k.sig_hgt());
            sig[2].add(deg(k.sig_tilt()));
            sig[3].add(deg(k.sig_hdg()));
            acc.add(&t.s);
            if helisim::jsonw::js_round(t.t * 240.0) as i64 % 240 == 0 {
                check_ekf_health(&t.s);
            }
            t.s.h.crashed
        });
        assert!(!t.s.h.crashed);
        assert_eq!(t.mode(), Some(Mode::Circuit));
        check_ekf_health(&t.s);
    }
    let pct = |x: f64| 100.0 * x / n;
    println!(
        "RUST EST EKF 10 min (60 s hover + circuit) x5: |err| < 3 sigma: x {:.1}% z {:.1}% height {:.1}% tilt {:.1}% heading {:.1}% | horizontal NEES mean {:.2} (2 DOF)",
        pct(tot[0]),
        pct(tot[1]),
        pct(tot[2]),
        pct(tot[3]),
        pct(tot[4]),
        nees.mean()
    );
    println!(
        "RUST EST EKF 10 min sigmas: pos {:.2}-{:.2} m | height {:.3}-{:.3} m | tilt {:.2}-{:.2}° | heading {:.2}-{:.2}° | errors {}",
        sig[0].mn,
        sig[0].mx,
        sig[1].mn,
        sig[1].mx,
        sig[2].mn,
        sig[2].mx,
        sig[3].mn,
        sig[3].mx,
        acc.line()
    );
    for (i, v) in tot.iter().enumerate() {
        assert!(pct(*v) > 90.0, "state {i}: only {:.1}% within 3 sigma", pct(*v));
    }
    assert!(sig[0].mx < 3.0 && sig[2].mx < 3.0 && sig[3].mx < 5.0);
}

// =====================================================================================
// STUNTS: each stunt 5x IDEAL, 5x REAL (EKF), 5x REAL BASIC for loop / roll / flip;
// stunt-aware crash protection; sensor effects
// =====================================================================================
use helisim::stunt::{Kind, StuntHook, KINDS};

#[derive(Debug, Default, Clone)]
struct SRun {
    done: bool,
    crashed: bool,
    rescues: u32,
    min_alt: f64,
    lost: f64,
    dev: f64,
    hdg: f64,
    rec: f64,
    score: f64,
    att_err: f64,
    pos_err: f64,
    gps_lost: bool,
    reconv: Option<f64>,
    sig_loss: f64,
    sig_back: f64,
    p3d_after: bool,
}

#[derive(Clone, Copy, PartialEq)]
enum SMode {
    Ideal,
    Ekf,
    Basic,
}
impl SMode {
    fn name(self) -> &'static str {
        match self {
            SMode::Ideal => "IDEAL",
            SMode::Ekf => "REAL EKF",
            SMode::Basic => "REAL BASIC",
        }
    }
}

fn stunt_run(seed: u64, m: SMode, k: Kind, hook: StuntHook, protect: bool) -> SRun {
    let mut t = T::fresh(seed, protect, m != SMode::Ideal);
    if m == SMode::Basic {
        t.s.set.ekf = false;
    }
    t.takeoff_hold();
    t.wait(2.0);
    t.s.stunt_hook = hook;
    let r0 = t.s.rescues;
    let n0 = t.s.stunt_count;
    t.s.command(Command::Stunts);
    t.s.command(match k {
        Kind::Pirouette => Command::StuntPirouette,
        Kind::Loop => Command::StuntLoop,
        Kind::Roll => Command::StuntRoll,
        Kind::Flip => Command::StuntFlip,
        Kind::Inverted => Command::StuntInverted,
    });
    assert!(!t.s.picker, "picking a stunt closes the picker");
    let mut r = SRun { min_alt: 1e9, ..SRun::default() };
    let mut in_win = false;
    let mut lost_seen = false;
    t.run(90.0, |t| {
        let w = t.s.stunt_window().is_some();
        if w {
            in_win = true;
            let e = errs(&t.s);
            r.att_err = r.att_err.max(e.tilt);
            r.pos_err = r.pos_err.max(e.pos);
        }
        if in_win {
            r.min_alt = r.min_alt.min(t.alt());
        }
        let st = &t.s.sens.st;
        if st.gps_lost && !lost_seen {
            lost_seen = true;
            r.gps_lost = true;
            r.sig_loss = t.s.sens.ekf.sig_pos();
        }
        if let Some(b) = t.s.sens.gps_back_t {
            if lost_seen && r.reconv.is_none() {
                if r.sig_back == 0.0 {
                    r.sig_back = t.s.sens.ekf.sig_pos();
                }
                if errs(&t.s).pos < 1.0 {
                    r.reconv = Some(t.s.sim_t - b);
                }
            }
        }
        if t.s.h.crashed {
            r.crashed = true;
            return true;
        }
        let rec_done = t.s.rescues > r0 && t.mode() == Some(Mode::Hold);
        let stunt_done = t.s.stunt_count > n0 && t.mode() == Some(Mode::Hold);
        if stunt_done || rec_done {
            // let the estimate settle after a GPS outage, for the re-convergence time
            return !lost_seen || r.reconv.is_some() || t.s.sens.gps_back_t.is_some_and(|b| t.s.sim_t - b > 15.0);
        }
        false
    });
    r.rescues = t.s.rescues - r0;
    r.p3d_after = t.s.set.pitch3d;
    if t.s.stunt_count > n0 {
        let x = t.s.stunt_last.clone().unwrap();
        r.done = true;
        r.lost = x.height_lost;
        r.dev = x.max_dev;
        r.hdg = x.hdg_err;
        r.rec = x.rec_t;
        r.score = x.score as f64;
        r.min_alt = x.min_alt;
    }
    r
}

#[derive(Default)]
struct SAgg {
    n: u32,
    done: u32,
    crashes: u32,
    rescues: u32,
    min_alt: Option<Stat>,
    lost: Option<Stat>,
    dev: Option<Stat>,
    hdg: Option<Stat>,
    score: Option<Stat>,
    rec: Option<Stat>,
    att: Option<Stat>,
    pos: Option<Stat>,
    reconv: Option<Stat>,
    gps_lost: u32,
}
fn st_add(s: &mut Option<Stat>, x: f64) {
    s.get_or_insert_with(Stat::new).add(x);
}
fn rng_s(s: &Option<Stat>, d: usize) -> String {
    match s {
        Some(s) => format!("{:.*}-{:.*}", d, s.mn, d, s.mx),
        None => "-".into(),
    }
}

fn stunt_batch(m: SMode, k: Kind, seeds: std::ops::Range<u64>) -> SAgg {
    let mut a = SAgg::default();
    for seed in seeds {
        let r = stunt_run(seed, m, k, StuntHook::default(), true);
        a.n += 1;
        a.done += r.done as u32;
        a.crashes += r.crashed as u32;
        a.rescues += r.rescues;
        if r.done {
            st_add(&mut a.min_alt, r.min_alt);
            st_add(&mut a.lost, r.lost);
            st_add(&mut a.dev, r.dev);
            st_add(&mut a.hdg, r.hdg.abs());
            st_add(&mut a.score, r.score);
            st_add(&mut a.rec, r.rec);
        }
        if m != SMode::Ideal {
            st_add(&mut a.att, r.att_err);
            st_add(&mut a.pos, r.pos_err);
        }
        if r.gps_lost {
            a.gps_lost += 1;
            if m != SMode::Ideal {
                // the filter coasts on the IMU while GPS is lost: its uncertainty must grow
                assert!(r.sig_back > r.sig_loss, "{} {}: sigma {} -> {}", m.name(), k.name(), r.sig_loss, r.sig_back);
            }
            if let Some(c) = r.reconv {
                st_add(&mut a.reconv, c);
            }
        }
        assert!(!r.p3d_after || r.rescues > 0, "pitch curve restored after the stunt");
    }
    let real = m != SMode::Ideal;
    println!(
        "RUST STUNT {} {:<14} | done {}/{} crashes {} rescues {} | min alt {} m | lost {} m | dev {} m (mean {:.2}) | hdg {} deg | rec {} s | score {} (mean {:.0}){}",
        m.name(),
        k.name(),
        a.done,
        a.n,
        a.crashes,
        a.rescues,
        rng_s(&a.min_alt, 1),
        rng_s(&a.lost, 1),
        rng_s(&a.dev, 1),
        a.dev.as_ref().map_or(0.0, |s| s.mean()),
        rng_s(&a.hdg, 1),
        rng_s(&a.rec, 1),
        rng_s(&a.score, 0),
        a.score.as_ref().map_or(0.0, |s| s.mean()),
        if real {
            format!(
                " | max est err att {} deg pos {} m | GPS lost in {}/{} runs, re-converged (<1 m) after {} s",
                rng_s(&a.att, 1),
                rng_s(&a.pos, 2),
                a.gps_lost,
                a.n,
                rng_s(&a.reconv, 1)
            )
        } else {
            String::new()
        }
    );
    a
}

/// Seeds of the full stunt matrices (5 per stunt and mode).
const MATRIX_SEEDS: std::ops::Range<u64> = 810..815;

/// Per-stunt bounds for the full matrices: (min score, max position deviation m, max height lost m).
/// Observed values with margin; see README "Stunt results".
fn matrix_bounds(m: SMode, k: Kind) -> (f64, f64, f64) {
    match (m, k) {
        (_, Kind::Pirouette) => (95.0, 0.8, 0.5),
        (SMode::Ideal, Kind::Loop) => (88.0, 0.6, 1.8),
        (SMode::Ideal, Kind::Roll) => (90.0, 1.4, 1.2),
        (SMode::Ideal, Kind::Flip) => (80.0, 1.9, 1.3),
        (SMode::Ideal, Kind::Inverted) => (88.0, 1.4, 1.0),
        (SMode::Ekf, Kind::Loop) => (85.0, 1.7, 1.8),
        (SMode::Ekf, Kind::Roll) => (87.0, 1.8, 1.3),
        (SMode::Ekf, Kind::Flip) => (80.0, 2.1, 1.3),
        (SMode::Ekf, Kind::Inverted) => (70.0, 3.0, 1.0),
        (SMode::Basic, Kind::Loop) => (72.0, 2.6, 2.0),
        (SMode::Basic, Kind::Roll) => (55.0, 5.0, 1.5),
        (SMode::Basic, Kind::Flip) => (68.0, 3.3, 1.3),
        (SMode::Basic, Kind::Inverted) => (0.0, 99.0, 99.0),
    }
}

fn check_matrix(m: SMode, k: Kind, a: &SAgg) {
    let (smin, dmax, lmax) = matrix_bounds(m, k);
    let n = MATRIX_SEEDS.end - MATRIX_SEEDS.start;
    assert_eq!((a.done as u64, a.crashes, a.rescues), (n, 0, 0), "{} {}: completes, no crash, no false rescue", m.name(), k.name());
    let (sc, dev, lost) = (a.score.as_ref().unwrap(), a.dev.as_ref().unwrap(), a.lost.as_ref().unwrap());
    assert!(sc.mn >= smin, "{} {}: score {} < {smin}", m.name(), k.name(), sc.mn);
    assert!(dev.mx <= dmax, "{} {}: deviation {} > {dmax}", m.name(), k.name(), dev.mx);
    assert!(lost.mx <= lmax, "{} {}: height lost {} > {lmax}", m.name(), k.name(), lost.mx);
    // BASIC exits a loop up to 5.3 deg off heading (its compass fusion is slow)
    assert!(a.hdg.as_ref().unwrap().mx < if m == SMode::Basic { 8.0 } else { 4.0 }, "{} {}: heading error at exit", m.name(), k.name());
    if m != SMode::Ideal {
        // BASIC's accelerometer leveling is pulled off by the manoeuvre (8..15 deg in a loop)
        assert!(a.att.as_ref().unwrap().mx < if m == SMode::Ekf { 3.0 } else { 20.0 }, "{} {}: attitude estimate", m.name(), k.name());
        assert!(a.pos.as_ref().unwrap().mx < if m == SMode::Ekf { 2.5 } else { 2.0 }, "{} {}: position estimate", m.name(), k.name());
        if a.gps_lost > 0 {
            // estimate back under 1 m within 1 s of the fix returning, every run
            let r = a.reconv.as_ref().expect("re-converged");
            assert!(r.n as u32 == a.gps_lost && r.mx < 1.0, "{} {}: re-convergence {:?}", m.name(), k.name(), r.mx);
        }
    }
}

/// Full matrix: every stunt, 5 seeds IDEAL and 5 seeds REAL (EKF).
#[test]
fn t8_stunt_matrix() {
    for m in [SMode::Ideal, SMode::Ekf] {
        for k in KINDS {
            let a = stunt_batch(m, k, MATRIX_SEEDS);
            check_matrix(m, k, &a);
        }
    }
}

/// Full matrix, REAL BASIC estimator: loop, roll, flip, 5 seeds each.
#[test]
fn t8_stunt_matrix_basic() {
    for k in [Kind::Loop, Kind::Roll, Kind::Flip] {
        let a = stunt_batch(SMode::Basic, k, MATRIX_SEEDS);
        check_matrix(SMode::Basic, k, &a);
    }
}

/// REAL (EKF) inverted hover over 20 seeds: the dead-reckoning case (6 s without GPS).
/// Before drag fusion: mean deviation 2.5 m, mean score 78; now about 1.4 m / 89.
#[test]
fn t8_inverted_dead_reckoning() {
    let a = stunt_batch(SMode::Ekf, Kind::Inverted, 820..840);
    assert_eq!((a.done, a.crashes, a.rescues), (20, 0, 0));
    let (sc, dev) = (a.score.as_ref().unwrap(), a.dev.as_ref().unwrap());
    println!("RUST STUNT REAL EKF Inverted hover x20: dev mean {:.2} max {:.2} m | score mean {:.1} min {:.0}", dev.mean(), dev.mx, sc.mean(), sc.mn);
    assert!(dev.mean() < 1.8 && dev.mx < 3.0, "dead-reckoning drift: mean {} max {}", dev.mean(), dev.mx);
    assert!(sc.mean() >= 85.0 && sc.mn >= 70.0, "score: mean {} min {}", sc.mean(), sc.mn);
}

/// One IDEAL and one REAL (EKF) run per stunt: completes, no crash, no false rescue.
#[test]
fn t8_stunts() {
    for m in [SMode::Ideal, SMode::Ekf] {
        for k in KINDS {
            let a = stunt_batch(m, k, 800 + k as u64..801 + k as u64);
            assert_eq!((a.done, a.crashes, a.rescues), (1, 0, 0), "{} {}", m.name(), k.name());
            assert!(a.lost.unwrap().mx < 3.0, "{} height lost", k.name());
            if m == SMode::Ideal {
                assert!(a.score.unwrap().mn >= 70.0, "{} score", k.name());
            }
        }
    }
}

/// Stunt-aware crash protection: deliberately broken stunts trigger a rescue and recover; with
/// PROTECT off they crash.
#[test]
fn t8_stunt_protect() {
    let cases: [(&str, Kind, StuntHook); 2] = [
        ("loop at 7 m, collective cut over the top", Kind::Loop, StuntHook { alt: Some(7.0), cut_coll: Some((1.2, 0.0)), ..StuntHook::default() }),
        ("collective cut mid-roll", Kind::Roll, StuntHook { cut_coll: Some((0.6, 0.0)), ..StuntHook::default() }),
    ];
    for (name, k, hook) in cases {
        for real in [false, true] {
            let m = if real { SMode::Ekf } else { SMode::Ideal };
            let seed = 840 + k as u64 + if real { 10 } else { 0 };
            let mut t = T::fresh(seed, true, real);
            t.takeoff_hold();
            t.wait(2.0);
            t.s.stunt_hook = hook;
            t.s.start_stunt(k);
            let before = t.s.rescues;
            let (mut trig, mut why, mut min_alt, mut rec) = (None, String::new(), 1e9f64, None);
            let mut in_win_t = None;
            t.run(60.0, |t| {
                if in_win_t.is_none() && t.s.stunt_window().is_some() {
                    in_win_t = Some(t.t);
                }
                if in_win_t.is_some() {
                    min_alt = min_alt.min(t.alt());
                }
                if trig.is_none() && t.s.rescues > before {
                    trig = Some(t.t - in_win_t.unwrap_or(t.t));
                    why = t.last_toast();
                }
                if trig.is_some() && t.mode() == Some(Mode::Hold) {
                    rec = Some(t.t - in_win_t.unwrap_or(t.t));
                    return true;
                }
                t.s.h.crashed
            });
            println!(
                "RUST STUNT PROTECT {} {name}: {} | min alt {:.2} m | {}",
                m.name(),
                match &trig {
                    Some(x) => format!("TRIGGERED +{x:.2} s into the manoeuvre ({why})"),
                    None => "no trigger".into(),
                },
                min_alt,
                if t.s.h.crashed {
                    "CRASHED".to_string()
                } else {
                    format!("recovered to hover hold at +{:.2} s, pitch {}", rec.unwrap_or(-1.0), if t.s.set.pitch3d { "3D" } else { "normal" })
                }
            );
            assert!(trig.is_some() && !t.s.h.crashed && rec.is_some(), "{name}");
            assert!(!t.s.set.pitch3d, "{name}: the user's normal pitch curve is restored after the rescue");
            // (c) the same with PROTECT off crashes
            let mut t = T::fresh(seed, false, real);
            t.takeoff_hold();
            t.wait(2.0);
            t.s.stunt_hook = hook;
            t.s.start_stunt(k);
            let ok = t.run(60.0, |t| t.s.h.crashed || (t.s.stunt_count > 0 && t.mode() == Some(Mode::Hold)));
            println!("RUST STUNT PROTECT {} {name}, PROTECT OFF: {}", m.name(), if t.s.h.crashed { "CRASHED" } else { "no crash" });
            assert!(ok && t.s.h.crashed, "{name} with PROTECT off should crash");
        }
    }
}

/// Stunt guards and interplay: battery, failsafe deferral, stick takeover (inverted keeps 3D),
/// a manual 3D inverted pilot still gets the existing rules, shortest-way rescue.
#[test]
fn t8_stunt_rules() {
    // battery below 25% blocks a stunt
    let mut t = T::fresh(860, true, false);
    t.takeoff_hold();
    t.s.h.bat = 0.24;
    t.wait(0.5);
    t.s.command(Command::StuntFlip);
    println!("RUST STUNT RULES battery 24%: {} | {}", if t.mode() == Some(Mode::Stunt) { "STARTED" } else { "blocked" }, t.last_toast());
    assert_eq!(t.mode(), Some(Mode::Hold));
    assert!(t.last_toast().contains("25%"));

    // the 15% failsafe waits until the manoeuvre has recovered
    let mut t = T::fresh(861, true, false);
    t.takeoff_hold();
    t.s.command(Command::StuntInverted);
    t.run(40.0, |t| t.s.stunt_window().is_some());
    t.wait(1.0);
    t.s.h.bat = 0.14;
    let (mut fs_in_win, mut fs_at, mut done_at) = (false, None, None);
    t.run(40.0, |t| {
        let fs = t.s.auto.as_ref().is_some_and(|a| a.fs);
        if fs && t.s.stunt_window().is_some() {
            fs_in_win = true;
        }
        if done_at.is_none() && t.s.stunt_count > 0 {
            done_at = Some(t.t);
        }
        if fs && fs_at.is_none() {
            fs_at = Some(t.t);
            return true;
        }
        false
    });
    println!(
        "RUST STUNT RULES 15% failsafe during inverted hover: deferred, stunt recovered at {:.2} s, failsafe landing at {:.2} s | {}",
        done_at.unwrap_or(-1.0),
        fs_at.unwrap_or(-1.0),
        t.cue()
    );
    assert!(!fs_in_win && fs_at.is_some() && done_at.is_some_and(|d| fs_at.unwrap() >= d));
    assert!(!t.s.set.pitch3d && !t.s.h.crashed);

    // stick touch while inverted: control handed back, 3D pitch kept; PROTECT's existing
    // 3D rules (bank > 55 deg) then recover it
    for protect in [false, true] {
        let mut t = T::fresh(862, protect, false);
        t.takeoff_hold();
        t.s.command(Command::StuntInverted);
        t.run(40.0, |t| t.s.stunt_window().is_some() && up_of(t.s.h.q).y < -0.95);
        let r0 = t.s.rescues;
        let coll = t.s.h.coll_cmd;
        t.stick_touch();
        let p3d = t.s.set.pitch3d;
        let msg = t.last_toast();
        t.wait(0.2);
        println!(
            "RUST STUNT RULES takeover inverted, PROTECT {}: autopilot {}, pitch {}, throttle {:.2} (collective {:.2} -> {:.2}) | {} | rescue {}",
            if protect { "on" } else { "off" },
            if t.s.auto.as_ref().is_some_and(|a| a.mode != Mode::Rescue) { "on" } else { "off" },
            if p3d { "3D" } else { "normal" },
            t.s.sticks.thr,
            coll,
            t.s.h.coll_cmd,
            msg,
            t.s.rescues > r0
        );
        assert!(p3d && msg.contains("3D"));
        if protect {
            assert!(t.s.rescues > r0, "existing rules: bank > 55 deg in manual flight");
            t.run(10.0, |t| t.mode() == Some(Mode::Hold));
            assert!(!t.s.h.crashed && t.mode() == Some(Mode::Hold) && up_of(t.s.h.q).y > 0.9);
            assert!(t.s.set.pitch3d, "the pilot keeps 3D after taking over inverted");
        } else {
            assert!((t.s.h.coll_cmd - coll).abs() < 0.05, "collective kept on takeover");
            assert!(t.s.auto.is_none() && !t.s.h.crashed);
        }
    }

    // stick touch while climbing: normal pitch curve unchanged
    let mut t = T::fresh(863, true, false);
    t.takeoff_hold();
    t.s.command(Command::StuntLoop);
    t.wait(2.0);
    t.stick_touch();
    assert!(t.s.auto.is_none() && !t.s.set.pitch3d);

    // rescue from a stunt rolls upright the shortest way: pick the axis closer to upright
    for (name, axis, ang) in [("rolled 150 deg", v3(0.0, 0.0, 1.0), 150.0f64), ("pitched 150 deg", v3(1.0, 0.0, 0.0), 150.0), ("pitched -160 deg", v3(1.0, 0.0, 0.0), -160.0)] {
        let mut t = T::fresh(864, true, false);
        hover_at(&mut t, 20.0);
        t.s.set.pitch3d = true;
        t.s.h.q = t.s.h.q * Quat::axis_angle(axis, ang.to_radians());
        t.s.h.w = V3::ZERO;
        let (mut tot, mut prev) = (v3(0.0, 0.0, 0.0), t.s.h.q);
        let mut min_alt = 1e9f64;
        let r0 = t.s.rescues;
        t.run(12.0, |t| {
            let d = rotvec(prev.inv() * t.s.h.q);
            prev = t.s.h.q;
            tot += d;
            min_alt = min_alt.min(t.alt());
            t.s.rescues > r0 && t.mode() == Some(Mode::Hold)
        });
        println!(
            "RUST STUNT RULES rescue {name}: rotated about pitch {:.0} deg / roll {:.0} deg, min alt {:.1} m, {}",
            tot.x.to_degrees(),
            tot.z.to_degrees(),
            min_alt,
            if t.mode() == Some(Mode::Hold) { "hover hold" } else { "NOT recovered" }
        );
        assert!(t.mode() == Some(Mode::Hold) && !t.s.h.crashed);
        let (on, off) = if axis.x != 0.0 { (tot.x, tot.z) } else { (tot.z, tot.x) };
        assert!(on.abs() > 120.0f64.to_radians() && on.abs() < 220.0f64.to_radians() && off.abs() < 60.0f64.to_radians(), "{name}: {tot:?}");
    }
}

/// REAL sensors in a stunt: rangefinder invalid when tilted, GPS lost upside down and
/// reacquired with reduced satellites, vibration up with rotor load, EKF gravity-aiding gate.
#[test]
fn t8_stunt_sensors() {
    let mut t = T::fresh(870, true, true);
    t.takeoff_hold();
    t.s.command(Command::StuntInverted);
    let (mut rf_tilted, mut lost_up, mut lost_panel, mut reacq_panel) = (false, false, false, false);
    let (mut sats_reacq, mut vib_max, mut grav_fused_hi) = (99, 0.0f64, 0u32);
    let mut reacq_dur = 0.0;
    let mut lost_dur = 0.0;
    let mut last = t.s.sens.ekf.h.fused_grav;
    t.run(60.0, |t| {
        let s = &t.s;
        let up = up_of(s.h.q);
        if s.sens.st.rf && up.y < (25.0f64).to_radians().cos() {
            rf_tilted = true;
        }
        if s.sens.st.gps_lost {
            lost_dur += DT;
            if up.y > (110.0f64).to_radians().cos() + 0.05 {
                lost_up = true;
            }
            lost_panel |= s.hud().sens.contains("GPS LOST (inverted)");
        }
        if s.sens.st.gps_reacq {
            reacq_dur += DT;
            sats_reacq = sats_reacq.min(s.sens.st.sats);
            reacq_panel |= s.hud().sens.contains("GPS REACQUIRING");
        }
        vib_max = vib_max.max(s.sens.st.vib);
        let k = &s.sens.ekf;
        if k.h.fused_grav != last {
            last = k.h.fused_grav;
            if s.h.w.length() > 1.0 {
                grav_fused_hi += 1;
            }
        }
        t.s.stunt_count > 0 && t.mode() == Some(Mode::Hold) && !t.s.sens.st.gps_reacq
    });
    println!(
        "RUST STUNT SENSORS inverted hover: GPS lost {:.1} s (panel \"GPS LOST (inverted)\": {}), reacquiring {:.1} s with as few as {} sats (panel: {}), rangefinder used above 25 deg: {}, lost while upright-ish: {}, peak vibration {:.2}x hover, gravity aiding fused at > 1 rad/s: {}",
        lost_dur, lost_panel, reacq_dur, sats_reacq, reacq_panel, rf_tilted, lost_up, vib_max, grav_fused_hi
    );
    assert!(lost_dur > 4.0 && lost_panel && reacq_panel && !rf_tilted && !lost_up);
    assert!((2.0..4.1).contains(&reacq_dur) && sats_reacq <= 8);
    assert!(vib_max > 1.1 && grav_fused_hi == 0);
    assert!(!t.s.h.crashed);
    check_ekf_health(&t.s);
}

// =====================================================================================
// ASSISTS: ground-proximity protection, ALT HOLD, throttle curve, AUTO-YAW, tail RATE
// =====================================================================================
use helisim::assist::{self, hover_stick, thr_curve};
use helisim::fbl::{expo, MAX_TILT};

fn expo_inv(y: f64) -> f64 {
    let y = y.clamp(-1.0, 1.0);
    let mut x = y;
    for _ in 0..30 {
        x -= (expo(x) - y) / (0.65 + 1.05 * x * x);
    }
    x.clamp(-1.0, 1.0)
}

/// A scripted stick pilot: looks at the true heli (as a pilot at the flight line does) and moves
/// the sticks. Cyclic flies a velocity along a closed path (figure of eight or straight line),
/// yaw keeps the nose on the flight path unless AUTO-YAW does it, throttle is fixed or nudged.
struct Pilot {
    pts: Vec<(f64, f64)>,
    i: usize,
    v: f64,
    yaw: bool,
}

impl Pilot {
    /// Figure of eight: two 7 m circles touching at (2, -13), flown at `v` m/s.
    fn eight(v: f64) -> Pilot {
        let (cx, cz, r) = (2.0, -13.0, 7.0);
        let mut pts = vec![];
        let n = 88;
        for k in 0..n {
            let a = k as f64 / n as f64 * 2.0 * std::f64::consts::PI;
            pts.push((cx - r + r * a.cos(), cz - r * a.sin())); // left circle, counter-clockwise from the crossing
        }
        for k in 0..n {
            let a = k as f64 / n as f64 * 2.0 * std::f64::consts::PI;
            pts.push((cx + r - r * a.cos(), cz - r * a.sin())); // right circle, clockwise
        }
        Pilot { pts, i: 0, v, yaw: true }
    }
    /// Hold a spot.
    fn point(x: f64, z: f64) -> Pilot {
        Pilot { pts: vec![(x, z); 4], i: 0, v: 0.0, yaw: false }
    }
    /// Straight line from (x0, z0) along heading direction (dx, dz), 200 m long.
    fn line(x0: f64, z0: f64, dx: f64, dz: f64, v: f64) -> Pilot {
        let pts = (0..200).map(|k| (x0 + dx * k as f64, z0 + dz * k as f64)).collect();
        Pilot { pts, i: 0, v, yaw: false }
    }
    fn sticks(&mut self, s: &Sim) -> (f64, f64, f64) {
        let h = &s.h;
        let n = self.pts.len();
        let d2 = |i: usize| (self.pts[i].0 - h.p.x).powi(2) + (self.pts[i].1 - h.p.z).powi(2);
        let mut bi = self.i;
        for k in 1..10 {
            let j = (self.i + k) % n;
            if d2(j) < d2(bi) {
                bi = j;
            }
        }
        self.i = bi;
        let (a, b) = (self.pts[bi], self.pts[(bi + 3) % n]);
        let (tx, tz) = ((b.0 - a.0), (b.1 - a.1));
        let tl = tx.hypot(tz).max(1e-9);
        let (tx, tz) = (tx / tl, tz / tl);
        let (ex, ez) = (h.p.x - a.0, h.p.z - a.1);
        let (vdx, vdz) = (tx * self.v - 0.6 * ex, tz * self.v - 0.6 * ez);
        let w = s.wind;
        let (rx, rz) = (h.v.x - w.x, h.v.z - w.z);
        let dk = 0.22 + 0.035 * rx.hypot(rz);
        let (ax, az) = (1.4 * (vdx - h.v.x) + dk * rx, 1.4 * (vdz - h.v.z) + dk * rz);
        let (fwd, right) = fwd_right(h.q);
        let lim = 22f64.to_radians();
        let pa = ((ax * fwd.x + az * fwd.z) / 9.81).atan().clamp(-lim, lim);
        let ra = ((ax * right.x + az * right.z) / 9.81).atan().clamp(-lim, lim);
        let yaw = if self.yaw && h.v.hyp_xz() > 1.5 {
            let e = wrap((-h.v.x).atan2(-h.v.z) - heading_of(h.q));
            (-(2.0 * e) / 4.5).clamp(-0.8, 0.8)
        } else {
            0.0
        };
        (expo_inv(pa / MAX_TILT), expo_inv(ra / MAX_TILT), expo_inv(yaw))
    }
}

/// Step with this stick input for `sec` seconds; `f` sees the sim each step, true stops.
fn fly(s: &mut Sim, sec: f64, mut st: impl FnMut(&Sim) -> (f64, f64, f64, Option<f64>), mut f: impl FnMut(&Sim) -> bool) -> bool {
    let n = (sec / DT).round() as u64;
    for i in 0..n {
        if i % 4 == 0 {
            let (pit, rol, yaw, thr) = st(s);
            s.input(&Input { pit, rol, yaw, thr_set: thr, ..Input::default() });
        }
        s.step(DT);
        if f(s) {
            return true;
        }
    }
    false
}

fn mode_of(s: &Sim) -> Option<Mode> {
    s.auto.as_ref().map(|a| a.mode)
}

/// Autopilot to a hover at `alt`, then hand over to the sticks bumplessly (stick = current collective).
fn manual_at(seed: u64, alt: f64, wind: usize, protect: bool, real: bool) -> Sim {
    let mut s = Sim::new(seed);
    s.set.wind = wind;
    s.set.protect = protect;
    if real {
        s.command(Command::Sensors);
    }
    s.command(Command::Takeoff);
    fly(&mut s, 25.0, |_| (0.0, 0.0, 0.0, None), |s| mode_of(s) == Some(Mode::Hold));
    s.auto.as_mut().unwrap().alt = alt;
    fly(&mut s, 20.0, |_| (0.0, 0.0, 0.0, None), |s| (s.h.alt() - alt).abs() < 0.15 && s.h.v.y.abs() < 0.15);
    fly(&mut s, 3.0, |_| (0.0, 0.0, 0.0, None), |_| false);
    s.command(Command::Takeover);
    s
}

#[derive(Debug, Default)]
struct EightRes {
    min: f64,
    max: f64,
    crashed: Option<String>,
    trig: Option<(f64, f64, String)>,
    vmin: f64,
    vmax: f64,
    loss_10s: f64,
}

fn eight_run(seed: u64, alt_hold: bool, protect: bool, thr_off: f64, dur: f64, real: bool) -> EightRes {
    let mut s = manual_at(seed, 5.0, 1, protect, real);
    s.set.alt_hold = alt_hold;
    let thr = s.sticks.thr + thr_off;
    let mut p = Pilot::eight(5.5);
    let mut r = EightRes { min: 1e9, max: -1e9, vmin: 1e9, vmax: 0.0, ..Default::default() };
    let r0 = s.rescues;
    let t0 = s.sim_t;
    let mut started = false;
    let mut a10 = None;
    fly(
        &mut s,
        dur,
        |s| {
            if mode_of(s).is_some() {
                return (0.0, 0.0, 0.0, None);
            }
            let (a, b, c) = p.sticks(s);
            (a, b, c, Some(thr))
        },
        |s| {
            let tt = s.sim_t - t0;
            let hs = s.h.v.hyp_xz();
            if !started && hs > 5.0 {
                started = true;
            }
            if started && s.rescues == r0 {
                r.min = r.min.min(s.h.alt());
                r.max = r.max.max(s.h.alt());
                r.vmin = r.vmin.min(hs);
                r.vmax = r.vmax.max(hs);
            }
            if std::env::var("TRACE").is_ok() && ((tt * 240.0).round() as u64) % 120 == 0 {
                println!("  t {tt:.1} alt {:.2} vy {:.2} hs {:.2} tilt {:.0} ay {:.2} coll {:.3}", s.h.alt(), s.h.v.y, hs, s.h.tilt_deg(), s.ay_f, s.h.coll_cmd);
            }
            if a10.is_none() && tt >= 10.0 {
                a10 = Some(s.h.alt());
            }
            if r.trig.is_none() && s.rescues > r0 {
                r.trig = Some((tt, s.h.alt(), s.last_toast.as_ref().map(|t| t.sub.clone()).unwrap_or_default()));
            }
            if s.h.crashed {
                r.crashed = s.last_toast.as_ref().map(|t| t.sub.clone());
                return true;
            }
            r.trig.is_some() && mode_of(s) == Some(Mode::Hold)
        },
    );
    r.loss_10s = 5.0 - a10.unwrap_or(r.min);
    r
}


/// The user's case: manual figure of eight at 5 m / 5.5 m/s on stick, self-level, no autopilot.
#[test]
fn t9_ground_prox_eight() {
    for (seed, real) in [(900u64, false), (901, false), (902, true), (903, true)] {
        let r = eight_run(seed, true, true, 0.0, 60.0, real);
        println!(
            "RUST ASSIST eight ALT HOLD on, {} seed {seed}: alt {:.2}-{:.2} m (band {:+.2}/{:+.2} m), speed {:.1}-{:.1} m/s, rescues {}, crashed {}",
            if real { "REAL" } else { "IDEAL" }, r.min, r.max, r.min - 5.0, r.max - 5.0, r.vmin, r.vmax, r.trig.is_some(), r.crashed.is_some()
        );
        assert!(r.crashed.is_none() && r.trig.is_none());
        assert!(r.min > 4.3 && r.max < 5.7, "{r:?}");
    }
    for real in [false, true] {
        let tag = if real { "REAL" } else { "IDEAL" };
        let r = eight_run(910, false, true, 0.0, 60.0, real);
        let (tt, ta, why) = r.trig.clone().expect("ground proximity should trigger");
        println!(
            "RUST ASSIST eight {tag} ALT HOLD off, fixed hover throttle, PROTECT on: lost {:.2} m by the trigger, rescue at +{tt:.1} s at {ta:.2} m ({why}), crashed {}",
            5.0 - ta, r.crashed.is_some()
        );
        assert!(why.contains("Too low and fast") && r.crashed.is_none() && ta > 0.5);
        let r = eight_run(910, false, false, 0.0, 60.0, real);
        println!(
            "RUST ASSIST eight {tag} ALT HOLD off, fixed hover throttle, PROTECT off: min alt {:.2} m, crash: {}",
            r.min, r.crashed.clone().unwrap_or("none".into())
        );
        assert!(r.crashed.is_some() && r.trig.is_none());
    }
}

/// Gentle sink into the ground in forward flight: 5 m/s, -0.8 m/s from 4 m (a pilot easing down).
fn sink_run(seed: u64, protect: bool, real: bool) -> (Option<(f64, f64, String)>, Option<String>, f64, f64) {
    let mut s = manual_at(seed, 4.0, 0, protect, real);
    let thr0 = s.sticks.thr;
    let mut p = Pilot::line(0.0, 0.0, 0.0, -1.0, 5.0);
    let (r0, mut ti) = (s.rescues, 0.0);
    let (mut trig, mut crash, mut vs_at, mut min_a) = (None, None, 0.0, 9.0f64);
    fly(
        &mut s,
        25.0,
        |s| {
            if mode_of(s).is_some() {
                return (0.0, 0.0, 0.0, None);
            }
            let (a, b, c) = p.sticks(s);
            // descend at 0.8 m/s once moving (PI on the throttle stick, like a pilot easing it down)
            let vd = if s.h.v.hyp_xz() > 4.5 { -0.8 } else { 0.0 };
            let e = vd - s.h.v.y;
            ti += e * DT * 4.0;
            (a, b, c, Some((thr0 + 0.05 * e + 0.04 * ti).clamp(0.0, 1.0)))
        },
        |s| {
            min_a = min_a.min(s.h.alt());
            if trig.is_none() && s.rescues > r0 {
                trig = Some((s.h.alt(), s.h.v.hyp_xz(), s.last_toast.as_ref().unwrap().sub.clone()));
                vs_at = s.h.v.y;
            }
            if s.h.crashed {
                crash = s.last_toast.as_ref().map(|t| t.sub.clone());
                return true;
            }
            trig.is_some() && mode_of(s) == Some(Mode::Hold)
        },
    );
    (trig, crash, vs_at, min_a)
}

#[test]
fn t9_gentle_sink_forward() {
    for real in [false, true] {
        let tag = if real { "REAL" } else { "IDEAL" };
        for seed in [920u64, 921] {
            let (trig, crash, vs, min_a) = sink_run(seed, true, real);
            let (a, hs, why) = trig.clone().expect("no trigger");
            println!("RUST ASSIST sink {tag} seed {seed} PROTECT on: trigger at {a:.2} m, {hs:.1} m/s, v/s {vs:.2} ({why}); min alt {min_a:.2} m, crashed {}", crash.is_some());
            assert!(crash.is_none() && min_a > 0.2);
        }
        let (trig, crash, _, _) = sink_run(922, false, real);
        println!("RUST ASSIST sink {tag} PROTECT off: {}", crash.clone().unwrap_or("no crash".into()));
        assert!(trig.is_none());
        let c = crash.expect("should crash");
        assert!(c.contains("rollover") || c.contains("Blade strike"), "{c}");
    }
}

/// No false triggers of the new rule (or any rule) in normal flying.
#[test]
fn t9_no_false_triggers() {
    for real in [false, true] {
        let tag = if real { "REAL" } else { "IDEAL" };
        // autopilot LAND x3
        let mut out = vec![];
        for seed in 930u64..933 {
            let mut t = T::fresh(seed, true, real);
            t.s.set.thr_curve = true;
            hover_at(&mut t, 5.0);
            t.s.command(Command::Land);
            let r = watch(&mut t, 40.0);
            assert!(r.trig.is_none() && !t.s.h.crashed, "{tag} land {r:?}");
            out.push(format!("{:.2}", t.s.h.last_impact));
        }
        println!("RUST ASSIST no-false {tag} autopilot LAND x3: 0 rescues, touchdown {} m/s", out.join(" / "));
        // manual gentle landing x3: slow, level, a little drift
        let mut out = vec![];
        for (i, seed) in (940u64..943).enumerate() {
            let mut s = manual_at(seed, 3.0, 1, true, real);
            let thr = s.sticks.thr - 0.03;
            let v = 0.4 * i as f64; // 0, 0.4, 0.8 m/s of forward drift
            let mut p = Pilot::line(s.h.p.x, s.h.p.z, 0.0, -1.0, v);
            let r0 = s.rescues;
            fly(&mut s, 30.0, |s| { let (a, b, c) = p.sticks(s); (a, b, c, Some(thr)) }, |s| s.h.crashed || (s.h.grounded && s.h.airborne && s.h.ground_load > 0.9));
            assert!(s.rescues == r0 && !s.h.crashed, "{tag} manual landing {seed}: {:?}", s.last_toast);
            out.push(format!("{:.2}", s.h.last_impact));
        }
        println!("RUST ASSIST no-false {tag} manual gentle landing x3 (drift 0 / 0.4 / 0.8 m/s): 0 rescues, touchdown {} m/s", out.join(" / "));
        // autorotation demo x2
        let mut out = vec![];
        for seed in 950u64..952 {
            let mut t = T::fresh(seed, true, real);
            t.s.set.thr_curve = true;
            t.s.command(Command::Autorot);
            t.run(150.0, |t| t.s.h.auto_result.is_some() || t.s.h.crashed);
            assert!(t.s.rescues == 0 && !t.s.h.crashed && t.s.h.auto_result.is_some(), "{tag} autorot {:?}", t.s.last_toast);
            out.push(format!("{:.2}", t.s.h.auto_result.as_ref().unwrap().vs));
        }
        println!("RUST ASSIST no-false {tag} AUTOROTATE x2: 0 rescues, touchdown {} m/s", out.join(" / "));
    }
    // stunts x1 each (IDEAL)
    let mut out = vec![];
    for k in KINDS {
        let r = stunt_run(960 + k as u64, SMode::Ideal, k, StuntHook::default(), true);
        assert!(r.done && r.rescues == 0 && !r.crashed, "{k:?}");
        out.push(format!("{} {}", k.name(), r.score));
    }
    println!("RUST ASSIST no-false stunts x1: 0 rescues ({})", out.join(", "));
}

/// ALT HOLD: liftoff into hold, hold 3 m for 20 s, climb / descend commands, touchdown to direct.
#[test]
fn t9_alt_hold_handover() {
    for (real, p3d) in [(false, false), (true, false), (false, true)] {
        let tag = format!("{}{}", if real { "REAL" } else { "IDEAL" }, if p3d { " 3D" } else { "" });
        let mut s = Sim::new(970);
        if real {
            s.command(Command::Sensors);
        }
        if p3d {
            s.command(Command::Pitch);
        }
        s.command(Command::AltHold);
        assert!(s.set.alt_hold);
        s.command(Command::Arm);
        let hs = hover_stick(p3d);
        let mut stick = 0.0;
        let mut jump = 0.0f64;
        let (mut c_prev, mut act_prev) = (0.0, false);
        let mut log: Vec<(f64, f64, bool)> = vec![];
        let mut run = |s: &mut Sim, sec: f64, stick: f64, f: &mut dyn FnMut(&Sim) -> bool| {
            let mut p = Pilot::point(0.0, 0.0);
            fly(s, sec, |s| { let (a, b, c) = p.sticks(s); (a, b, c, Some(stick)) }, |s| {
                let c = s.h.coll_cmd;
                if s.ah.active != act_prev || s.ah.handback {
                    jump = jump.max((c - c_prev).abs());
                }
                act_prev = s.ah.active;
                c_prev = c;
                if std::env::var("TRACE").is_ok() && log.len() % 240 == 0 {
                    println!("  alt {:.2} vy {:.2} ah {} hb {} coll {:.3} stick {:.2} tgt {:.2}", s.h.alt(), s.h.v.y, s.ah.active, s.ah.handback, c, s.sticks.thr, s.ah.target);
                }
                log.push((s.h.alt(), s.h.v.y, s.ah.active));
                f(s)
            })
        };
        run(&mut s, 8.0, stick, &mut |_| false); // spool up
        // lift off: raise the stick slowly until it is airborne and ALT HOLD engages
        let mut t_eng = None;
        for i in 0..400 {
            stick = (0.3 + 0.002 * i as f64).min(hs + 0.12);
            run(&mut s, 0.05, stick, &mut |_| false);
            if s.ah.active {
                t_eng = Some(s.h.alt());
                break;
            }
        }
        let eng_alt = t_eng.expect("ALT HOLD should engage after liftoff");
        // climb to 3 m on the stick, then centre it
        run(&mut s, 20.0, hs + 0.08 + 0.5 * (1.0 - hs - 0.08), &mut |s| s.h.alt() + 0.55 * s.h.v.y > 3.0);
        run(&mut s, 3.0, hs, &mut |_| false);
        let held = s.ah.target;
        let (mut mn, mut mx) = (9.0f64, 0.0f64);
        run(&mut s, 20.0, hs, &mut |s| { mn = mn.min(s.h.alt()); mx = mx.max(s.h.alt()); false });
        // full up 2 s, full down 2 s
        let mut vmax = 0.0f64;
        run(&mut s, 2.0, 1.0, &mut |s| { vmax = vmax.max(s.h.v.y); false });
        let mut vmin = 0.0f64;
        run(&mut s, 2.0, 0.0, &mut |s| { vmin = vmin.min(s.h.v.y); false });
        run(&mut s, 3.0, hs, &mut |_| false);
        // descend to the ground on half-down stick
        let mut td = None;
        run(&mut s, 30.0, hs - 0.3, &mut |s| {
            if s.h.grounded && s.h.airborne && td.is_none() {
                td = Some(s.h.last_impact);
            }
            s.h.grounded && !s.ah.active && !s.ah.handback
        });
        let still = s.ah.active;
        // pull the stick down: direct collective
        run(&mut s, 2.0, 0.0, &mut |_| false);
        let low = s.h.coll_cmd;
        println!(
            "RUST ASSIST althold {tag}: engaged at {eng_alt:.2} m; hold {held:.2} m for 20 s: {mn:.2}-{mx:.2} m (band {:+.2}/{:+.2}); full up {vmax:.2} m/s, full down {vmin:.2} m/s; touchdown {:.2} m/s, back to direct {}; max collective step at handover {jump:.3}",
            mn - held, mx - held, td.unwrap_or(-1.0), !still
        );
        assert!(eng_alt > 0.45 && eng_alt < 1.0);
        assert!(mx - held < 0.4 && held - mn < 0.4 && (held - 3.0).abs() < 0.3, "{mn} {mx} {held}");
        assert!(vmax > 2.4 && vmax < 3.3 && vmin < -1.6 && vmin > -2.3, "{vmax} {vmin}");
        assert!(td.is_some_and(|v| v < 0.8) && !still && !s.h.crashed);
        assert!(jump < 0.05, "collective jump {jump}");
        assert!(low <= if p3d { -0.9 } else { 0.05 });
    }
}

#[test]
fn t9_throttle_curve() {
    for p3d in [false, true] {
        let h = hover_stick(p3d);
        let (mut prev, mut mono) = (-1.0, true);
        for i in 0..=1000 {
            let x = i as f64 / 1000.0;
            let y = thr_curve(x, h);
            mono &= y > prev;
            prev = y;
            assert!((assist::thr_curve_inv(y, h) - x).abs() < 1e-9);
        }
        let k = (thr_curve(h + 0.05, h) - thr_curve(h - 0.05, h)) / 0.1;
        let kl = (thr_curve(0.1, h) - thr_curve(0.0, h)) / 0.1;
        let kh = (thr_curve(1.0, h) - thr_curve(0.9, h)) / 0.1;
        println!(
            "RUST ASSIST curve {}: hover stick {h:.3}, f(0) {:.3} f(1) {:.3} f(h) {:.3}, slope near hover {k:.2}, below {kl:.2}, above {kh:.2}, monotonic {mono}",
            if p3d { "3D" } else { "normal" },
            thr_curve(0.0, h), thr_curve(1.0, h), thr_curve(h, h)
        );
        assert!(mono && thr_curve(0.0, h) == 0.0 && (thr_curve(1.0, h) - 1.0).abs() < 1e-12 && (thr_curve(h, h) - h).abs() < 1e-12);
        assert!((k - 0.5).abs() < 1e-9 && kl > 1.0 && kh > 1.0);
    }
}

fn hdg_err_air(s: &Sim) -> f64 {
    let (vx, vz) = (s.h.v.x - s.wind.x, s.h.v.z - s.wind.z);
    deg(wrap((-vx).atan2(-vz) - heading_of(s.h.q)))
}

#[test]
fn t9_auto_yaw() {
    // straight flight across a 4.5 m/s crosswind (GUSTY, from the right): fly away from the pilot
    for on in [true, false] {
        let mut s = manual_at(980, 6.0, 2, true, false);
        s.set.alt_hold = true;
        s.set.auto_yaw = on;
        let mut p = Pilot::line(s.h.p.x, s.h.p.z, 0.0, -1.0, 5.0);
        p.yaw = false;
        let (mut st, mut mx) = (Stat::new(), 0.0f64);
        let t0 = s.sim_t;
        fly(&mut s, 14.0, |s| { let (a, b, c) = p.sticks(s); (a, b, c, Some(hover_stick(false))) }, |s| {
            if s.sim_t - t0 > 6.0 {
                let e = hdg_err_air(s);
                st.add(e.abs());
                mx = mx.max(e.abs());
            }
            s.h.crashed
        });
        println!("RUST ASSIST autoyaw {} crosswind 4.5 m/s, 5 m/s straight: heading vs air-relative velocity mean {:.1}° max {:.1}°", if on { "ON " } else { "OFF" }, st.s / st.n, mx);
        if on {
            assert!(st.s / st.n < 5.0, "{}", st.s / st.n);
        } else {
            assert!(st.s / st.n > 20.0);
        }
    }
    // coordinated turn with roll only (calm): constant roll stick, pitch holds 5 m/s
    for on in [true, false] {
        let mut s = manual_at(981, 6.0, 0, true, false);
        s.set.alt_hold = true;
        s.set.auto_yaw = on;
        let t0 = s.sim_t;
        let mut st = Stat::new();
        fly(
            &mut s,
            12.0,
            |s| {
                let (fwd, _) = fwd_right(s.h.q);
                let u = s.h.v.x * fwd.x + s.h.v.z * fwd.z;
                let pit = (0.15 * (5.0 - u) + 0.25).clamp(-0.6, 0.6);
                let rol = if s.sim_t - t0 > 3.0 { 0.35 } else { 0.0 };
                (expo_inv(pit), expo_inv(rol), 0.0, Some(hover_stick(false)))
            },
            |s| {
                if s.sim_t - t0 > 5.0 {
                    st.add(hdg_err_air(s).abs());
                }
                s.h.crashed
            },
        );
        println!("RUST ASSIST autoyaw {} coordinated turn, roll stick 0.35 only: sideslip mean {:.1}° max {:.1}°", if on { "ON " } else { "OFF" }, st.s / st.n, st.mx);
        if on {
            assert!(st.s / st.n < 6.0);
        } else {
            assert!(st.s / st.n > 15.0);
        }
    }
    // hover (ground speed < 2 m/s) in a 4.5 m/s wind: heading held
    let mut s = manual_at(982, 4.0, 2, true, false);
    s.set.alt_hold = true;
    s.set.auto_yaw = true;
    let h0 = heading_of(s.h.q);
    let mut p = Pilot::point(s.h.p.x, s.h.p.z);
    let mut mx = 0.0f64;
    fly(&mut s, 15.0, |s| { let (a, b, c) = p.sticks(s); (a, b, c, Some(hover_stick(false))) }, |s| { mx = mx.max(deg(wrap(heading_of(s.h.q) - h0)).abs()); false });
    println!("RUST ASSIST autoyaw ON hover in 4.5 m/s wind, 15 s: heading change max {mx:.1}°");
    assert!(mx < 3.0);
}

#[test]
fn t9_tail_rate() {
    for rate in [false, true] {
        let mut s = manual_at(990, 4.0, 2, true, false);
        s.set.alt_hold = true;
        s.set.tail_rate = rate;
        let h0 = heading_of(s.h.q);
        let mut p = Pilot::point(s.h.p.x, s.h.p.z);
        let mut mx = 0.0f64;
        fly(&mut s, 15.0, |s| { let (a, b, c) = p.sticks(s); (a, b, c, Some(hover_stick(false))) }, |s| { mx = mx.max(deg(wrap(heading_of(s.h.q) - h0)).abs()); false });
        let end = deg(wrap(heading_of(s.h.q) - h0));
        // the wind comes from the right (+x): nose into wind is heading -90
        println!("RUST ASSIST tail {} hover, 4.5 m/s crosswind 15 s: heading max change {mx:.1}°, end {end:+.1}° (into wind -90°)", if rate { "RATE" } else { "HOLD" });
        if rate {
            assert!(mx > 30.0);
        } else {
            assert!(mx < 2.0);
        }
    }
    // autopilot modes force HOLD
    let mut t = T::fresh(991, true, false);
    t.s.set.wind = 2;
    t.s.set.tail_rate = true;
    t.takeoff_hold();
    let h0 = heading_of(t.s.h.q);
    let mut mx = 0.0f64;
    t.run(15.0, |t| { mx = mx.max(deg(wrap(heading_of(t.s.h.q) - h0)).abs()); false });
    println!("RUST ASSIST tail RATE setting, autopilot hover hold: heading max change {mx:.1}°");
    assert!(mx < 3.0);
}

// =====================================================================================
// LAND transit (fast and high when far) and the downward camera precision landing
// =====================================================================================
#[derive(Debug, Default)]
struct LandRes {
    t: f64,
    hmin: f64,
    hmax: f64,
    vmax: f64,
    vs: f64,
    d: f64,
    rescues: u32,
    crashed: bool,
    cam_at_td: bool,
    fused: u32,
    fake_fused: u32,
    max_tilt_low: f64,
}

/// Hover at (px, pz, 5 m), then LAND; `hook` runs every step (camera failures etc.).
fn land_from(seed: u64, px: f64, pz: f64, real: bool, cam: bool, wind: usize, mut hook: impl FnMut(&mut Sim, f64)) -> LandRes {
    let mut s = Sim::new(seed);
    s.set.wind = wind;
    if real {
        s.command(Command::Sensors);
    }
    s.sens.cam.enabled = cam;
    s.command(Command::Takeoff);
    fly(&mut s, 25.0, |_| (0.0, 0.0, 0.0, None), |s| mode_of(s) == Some(Mode::Hold));
    {
        let a = s.auto.as_mut().unwrap();
        a.anchor = v3(px, 0.0, pz);
        a.alt = 5.0;
    }
    fly(&mut s, 60.0, |_| (0.0, 0.0, 0.0, None), |s| (s.h.p.x - px).hypot(s.h.p.z - pz) < 0.5 && s.h.v.hyp_xz() < 0.3 && (s.h.alt() - 5.0).abs() < 0.3);
    fly(&mut s, 3.0, |_| (0.0, 0.0, 0.0, None), |_| false);
    let (r0, t0) = (s.rescues, s.sim_t);
    s.command(Command::Land);
    let mut r = LandRes { hmin: 1e9, ..Default::default() };
    for _ in 0..(120.0 / DT) as u64 {
        let t = s.sim_t - t0;
        hook(&mut s, t);
        s.step(DT);
        let s = &s;
        let d = s.h.p.x.hypot(s.h.p.z);
        if d > 5.0 {
            r.hmin = r.hmin.min(s.h.alt());
            r.hmax = r.hmax.max(s.h.alt());
        }
        if s.h.alt() < 2.0 {
            r.max_tilt_low = r.max_tilt_low.max(s.h.tilt_deg());
        }
        r.vmax = r.vmax.max(s.h.v.hyp_xz());
        r.cam_at_td = s.auto.as_ref().is_some_and(|a| a.cam_tgt);
        if (s.h.grounded && s.h.airborne) || s.h.crashed {
            break;
        }
    }
    r.t = s.sim_t - t0;
    r.vs = s.h.last_impact;
    r.d = s.h.p.x.hypot(s.h.p.z);
    r.rescues = s.rescues - r0;
    r.crashed = s.h.crashed;
    r.fused = s.sens.pl.n_fused;
    r.fake_fused = s.sens.pl.n_fake_fused;
    r
}

const LAND_SEEDS: std::ops::Range<u64> = 1000..1005;

fn land_line(tag: &str, rs: &[LandRes]) {
    let st = |f: &dyn Fn(&LandRes) -> f64| {
        let mut s = Stat::new();
        for r in rs {
            s.add(f(r));
        }
        s
    };
    let (t, hmin, hmax, vmax, vs, d) = (st(&|r| r.t), st(&|r| r.hmin), st(&|r| r.hmax), st(&|r| r.vmax), st(&|r| r.vs), st(&|r| r.d));
    println!(
        "RUST LANDX {tag}: time {:.1}-{:.1} s (mean {:.1}) | transit alt {:.1}-{:.1} m | max speed {:.1}-{:.1} m/s | touchdown {:.2}-{:.2} m/s | {:.2}-{:.2} m from centre (mean {:.2}) | rescues {} | crashes {}",
        t.mn, t.mx, t.mean(), if hmin.mn > 1e8 { 0.0 } else { hmin.mn }, hmax.mx, vmax.mn, vmax.mx, vs.mn, vs.mx, d.mn, d.mx, d.mean(),
        rs.iter().map(|r| r.rescues).sum::<u32>(), rs.iter().filter(|r| r.crashed).count()
    );
}

/// LAND from 30 m and 45 m, IDEAL and REAL EKF, 5 seeds each: faster, higher in transit, soft, no rescue.
#[test]
fn t10_land_transit() {
    for real in [false, true] {
        for (px, pz, dist) in [(-26.0, -15.0, 30.0), (-38.0, -24.0, 45.0)] {
            let rs: Vec<LandRes> = LAND_SEEDS.map(|sd| land_from(sd, px, pz, real, true, 1, |_, _| {})).collect();
            land_line(&format!("{} from {dist} m", if real { "REAL EKF" } else { "IDEAL" }), &rs);
            // before: 20.4 s (30 m) and 26.3 s (45 m) IDEAL at 5 m and 2.5 m/s
            let t_max = if dist > 40.0 { 23.0 + if real { 4.0 } else { 0.0 } } else { 18.0 + if real { 3.0 } else { 0.0 } };
            for r in &rs {
                assert!(!r.crashed && r.rescues == 0, "{r:?}");
                assert!(r.vs < 0.6 && r.d < if real { 0.5 } else { 0.15 }, "{r:?}");
                assert!(r.t < t_max, "{dist} m: {} s", r.t);
                assert!(r.hmin > 4.3 && r.vmax > 5.0 && r.vmax < 8.6, "{r:?}");
                // clear of the 12 m trees at r >= 45 m, high enough to pass over 8 m bushes nearer in
                assert!(r.hmax > if dist > 40.0 { 11.5 } else { 10.0 } && r.hmax < 13.0, "{r:?}");
            }
        }
    }
    // tailwind and headwind (gusty, 4.5 m/s from +x): stops on the pad, no overshoot rescue
    for (px, pz, what) in [(38.0, -24.0, "tailwind"), (-38.0, -24.0, "headwind")] {
        let rs: Vec<LandRes> = LAND_SEEDS.take(3).map(|sd| land_from(sd, px, pz, false, true, 2, |_, _| {})).collect();
        land_line(&format!("IDEAL gusty {what} from 45 m"), &rs);
        for r in &rs {
            assert!(!r.crashed && r.rescues == 0 && r.vs < 0.6 && r.d < 0.3, "{r:?}");
        }
    }
    // short hop from 3.6 m: gentle approach as before, no climb
    let rs: Vec<LandRes> = LAND_SEEDS.take(3).map(|sd| land_from(sd, 3.0, -2.0, true, true, 1, |_, _| {})).collect();
    land_line("REAL short hop 3.6 m", &rs);
    for r in &rs {
        assert!(!r.crashed && r.rescues == 0 && r.vs < 0.6, "{r:?}");
    }
}

/// Camera precision landing vs GPS only (camera disabled), REAL EKF, 8 seeds, 30 m and the pad hover.
#[test]
fn t10_precland_accuracy() {
    let seeds = 1010u64..1018;
    for (px, pz, dist) in [(-26.0, -15.0, "30 m"), (2.0, -3.0, "3.6 m")] {
        let cam: Vec<LandRes> = seeds.clone().map(|sd| land_from(sd, px, pz, true, true, 1, |_, _| {})).collect();
        let gps: Vec<LandRes> = seeds.clone().map(|sd| land_from(sd, px, pz, true, false, 1, |_, _| {})).collect();
        land_line(&format!("REAL camera from {dist}"), &cam);
        land_line(&format!("REAL GPS only from {dist}"), &gps);
        let mean = |v: &[LandRes]| v.iter().map(|r| r.d).sum::<f64>() / v.len() as f64;
        let (mc, mg) = (mean(&cam), mean(&gps));
        for r in cam.iter().chain(gps.iter()) {
            assert!(!r.crashed && r.rescues == 0 && r.vs < 0.6, "{r:?}");
        }
        for r in &cam {
            assert!(r.d < 0.3 && r.fused > 20, "camera landing {r:?}");
        }
        assert!(mc < 0.2 && mc < 0.6 * mg, "camera {mc:.2} m vs GPS {mg:.2} m");
    }
}

/// Camera blocked mid-approach (falls back to GPS and still lands softly), camera noise-only
/// fakes and a persistent offset fake (gated, doesn't drag the landing), gusty camera landings.
#[test]
fn t10_precland_failures() {
    // blocked at 3 m on the way down (and at 20 m out in the transit for the second set)
    for (when, what) in [(0, "blocked in the descent at 3 m"), (1, "blocked from 8 s (approach)")] {
        let rs: Vec<LandRes> = (1020u64..1025)
            .map(|sd| {
                land_from(sd, -26.0, -15.0, true, true, 1, move |s, t| {
                    if (when == 0 && s.auto.as_ref().is_some_and(|a| a.descending) && s.h.alt() < 3.0) || (when == 1 && t > 8.0) {
                        s.sens.cam.enabled = false;
                    }
                })
            })
            .collect();
        land_line(&format!("REAL camera {what}"), &rs);
        for r in &rs {
            // GPS accuracy (as t7_ekf_land_autorotate), flying the GPS target at touchdown
            assert!(!r.crashed && r.rescues == 0 && r.vs < 0.6 && r.d < 1.2, "{r:?}");
            assert!(!r.cam_at_td || when == 0, "{r:?}");
        }
    }
    // persistent false detections 1.2 m off the tag in 25% of the frames: rejected by the gate
    let rs: Vec<LandRes> = (1030u64..1035)
        .map(|sd| land_from(sd, -26.0, -15.0, true, true, 1, |s, _| s.sens.cam.fake_hook = Some((1.2, 0.0, 0.25))))
        .collect();
    land_line("REAL camera with 25% false detections 1.2 m off", &rs);
    for r in &rs {
        assert!(!r.crashed && r.rescues == 0 && r.vs < 0.6 && r.d < 0.35, "{r:?}");
        assert!(r.fake_fused * 10 < r.fused, "fakes fused {} of {}", r.fake_fused, r.fused);
    }
    // gusty, camera vs GPS
    let cam: Vec<LandRes> = (1040u64..1045).map(|sd| land_from(sd, -26.0, -15.0, true, true, 2, |_, _| {})).collect();
    let gps: Vec<LandRes> = (1040u64..1045).map(|sd| land_from(sd, -26.0, -15.0, true, false, 2, |_, _| {})).collect();
    land_line("REAL gusty camera from 30 m", &cam);
    land_line("REAL gusty GPS only from 30 m", &gps);
    for r in cam.iter().chain(gps.iter()) {
        assert!(!r.crashed && r.rescues == 0 && r.vs < 0.9, "{r:?}");
    }
}

/// Camera model geometry: detection range and limits.
#[test]
fn t10_camera_model() {
    use helisim::heli::Heli;
    use helisim::precland::Camera;
    let mut c = Camera::new(1);
    let mut h = Heli::new();
    let seen = |c: &mut Camera, h: &Heli| {
        let mut n = 0;
        for k in 0..40 {
            n += c.step(k as f64 * 0.05 + 0.001, h).iter().filter(|d| !d.fake).count();
        }
        c.reset(0.0);
        n
    };
    let mut lo = vec![];
    for (alt, x, tilt, rate, want) in [
        (5.0, 0.0, 0.0, 0.0, true),
        (25.0, 0.0, 0.0, 0.0, true),   // tag 16 px
        (40.0, 0.0, 0.0, 0.0, false),  // tag 10 px: too small
        (0.4, 0.0, 0.0, 0.0, false),   // tag fills the image
        (5.0, 3.6, 0.0, 0.0, false),   // centre outside the 70 deg field of view
        (5.0, 0.0, 40.0, 0.0, false),  // tilt over 35 deg
        (5.0, 0.0, 0.0, 3.0, false),   // motion blur
    ] {
        h.p = v3(x, alt + 0.29, 0.0);
        h.q = Quat::axis_angle(v3(1.0, 0.0, 0.0), tilt * helisim::units::DEG);
        h.w = v3(rate, 0.0, 0.0);
        let n = seen(&mut c, &h);
        lo.push(format!("alt {alt} x {x} tilt {tilt} rate {rate}: {n} of 39"));
        assert_eq!(n > 20, want, "{alt} {x} {tilt} {rate}: {n}");
    }
    println!("RUST CAMERA geometry: {}", lo.join(" | "));
}

// =====================================================================================
// SHOW: F3C-style schedule, autopilot demo, fly it yourself, simulated judges
// =====================================================================================
use helisim::judge;
use helisim::show::{self, Man, Part, Ph, ShowHook, ShowRes};

/// Run a show (demo or manual with a stick script) until it closes. Returns the result, crashes
/// and rescues. `script` gets the sim each 4 steps and returns stick input (manual mode).
fn show_run(seed: u64, real: bool, part: Part, demo: bool, hook: ShowHook, mut script: impl FnMut(&Sim) -> Option<Input>) -> (ShowRes, bool, u32) {
    let mut s = Sim::new(seed);
    if real {
        s.command(Command::Sensors);
    }
    s.show_hook = hook;
    s.show_ui.demo = demo;
    s.show_ui.part = part;
    s.command(Command::ShowStart);
    assert!(s.show.is_some(), "show did not start");
    for i in 0..240 * 900 {
        if i % 4 == 0 {
            if let Some(inp) = script(&s) {
                s.input(&inp);
            }
        }
        s.step(DT);
        if s.show.is_none() {
            break;
        }
    }
    assert!(s.show.is_none(), "show did not finish");
    (s.show_last.clone().unwrap(), s.h.crashed, s.rescues)
}

fn show_line(tag: &str, r: &ShowRes) {
    let per: Vec<String> = r.res.iter().map(|x| format!("{} {}", short(x.m), helisim::jsonw::to_fixed(x.score, 2))).collect();
    println!("RUST SHOW {tag}: {} | total {:.1} / {:.0} = {:.1}%", per.join(", "), r.total, r.max, r.pct);
}
fn short(m: Man) -> &'static str {
    ["SQ", "TRI", "PIR", "EIGHT", "LAND", "LOOP", "ROLL", "STALL", "RCIRC", "AUTO"][m as usize]
}

#[test]
fn t11_show_schedule_and_references() {
    // schedule, K-factors, parts, references inside the box and clear of the trees
    let ks: Vec<f64> = show::SCHED.iter().map(|m| m.k()).collect();
    assert_eq!(ks, vec![1.0, 1.5, 1.5, 2.0, 1.0, 1.5, 1.5, 2.0, 2.5, 2.0]);
    assert_eq!(Part::Hover.list().len(), 5);
    assert_eq!(Part::Aero.list().len(), 5);
    assert_eq!(Part::Full.list().len(), 10);
    for m in show::SCHED {
        let t = show::build(m, &ShowHook::default());
        let (mut ymin, mut ymax, mut rmax, mut xbox) = (1e9f64, -1e9f64, 0.0f64, 0.0f64);
        for r in &t.pts {
            ymin = ymin.min(r.p.y);
            ymax = ymax.max(r.p.y);
            rmax = rmax.max(r.p.x.hypot(r.p.z + 20.0));
            if r.leg >= 2 && r.leg != 4 {
                xbox = xbox.max((r.p.x - show::geo::AX).abs());
            }
            assert!(r.up.length() > 0.99 && r.p.y.is_finite());
        }
        println!("RUST SHOW ref {:<22} K {}  {:5.1} s  height {:5.1}..{:5.1} m  max r(0,-20) {:4.1} m  figure |x| {:4.1} m", m.name(), m.k(), t.dur, ymin, ymax, rmax, xbox);
        assert!(rmax < 44.0, "{m:?} reaches the trees ({rmax:.1} m)");
        if !m.hover() && m != Man::Autorot {
            assert!(xbox <= show::geo::EDGE + 0.1, "{m:?} figure leaves the box ({xbox:.1})");
            assert!(ymin > 10.0, "{m:?} too low ({ymin:.1})");
        }
    }
}

#[test]
fn t11_show_demo_full() {
    // autopilot demo, full schedule, IDEAL and REAL EKF, 3 seeds each
    let mut pcts = [vec![], vec![]];
    for (ri, real) in [false, true].into_iter().enumerate() {
        for seed in [1100u64, 1101, 1102] {
            let (r, crashed, resc) = show_run(seed, real, Part::Full, true, ShowHook::default(), |_| None);
            show_line(&format!("demo {} seed {seed}", if real { "REAL" } else { "IDEAL" }), &r);
            for x in &r.res {
                println!("    {:<22} base {:5.2} marks {:?} drop {}/{} -> {:.2} x {} = {:5.2} | {}", x.m.name(), x.base, x.marks, x.hi, x.lo, x.score, x.k, x.ks, x.why.join("; "));
            }
            assert!(r.complete && r.res.len() == 10, "incomplete: {}", r.note);
            assert!(!crashed && resc == 0, "crash {crashed} rescues {resc}");
            assert!(r.res.iter().all(|x| x.zero.is_none()), "a zero in the demo");
            pcts[ri].push(r.pct);
        }
    }
    let m = |v: &Vec<f64>| v.iter().sum::<f64>() / v.len() as f64;
    println!("RUST SHOW demo mean: IDEAL {:.1}% ({:.1}..{:.1}), REAL {:.1}% ({:.1}..{:.1})", m(&pcts[0]), pcts[0].iter().cloned().fold(1e9, f64::min), pcts[0].iter().cloned().fold(0.0, f64::max), m(&pcts[1]), pcts[1].iter().cloned().fold(1e9, f64::min), pcts[1].iter().cloned().fold(0.0, f64::max));
    assert!(pcts[0].iter().all(|p| (85.0..=97.0).contains(p)), "IDEAL {:?}", pcts[0]);
    assert!(m(&pcts[1]) < m(&pcts[0]) && pcts[1].iter().all(|p| *p > 60.0), "REAL {:?}", pcts[1]);
}

#[test]
fn t11_show_sloppy() {
    // the demo with injected errors: off-centre box, heading error, position sway, oversized
    // figures, a hard landing. Scores clearly lower with the matching reasons.
    let (good, _, _) = show_run(1110, false, Part::Full, true, ShowHook::default(), |_| None);
    let hook = ShowHook { off: v3(3.5, 0.0, 0.0), hdg: 14f64.to_radians(), scale: 1.35, land_vs: 1.6, sway: 0.6, cut: None };
    let (bad, crashed, resc) = show_run(1110, false, Part::Full, true, hook, |_| None);
    show_line("nominal", &good);
    show_line("sloppy", &bad);
    for x in &bad.res {
        println!("    {:<22} {:.2} | {}", x.m.name(), x.score, x.why.join("; "));
    }
    assert!(!crashed && resc == 0);
    assert!(bad.pct < good.pct - 25.0, "sloppy {:.1}% vs {:.1}%", bad.pct, good.pct);
    let why = |m: Man| bad.res.iter().find(|x| x.m == m).map(|x| x.why.join(" | ")).unwrap_or_default();
    let all: String = bad.res.iter().map(|x| x.why.join(" | ")).collect::<Vec<_>>().join(" | ");
    assert!(all.contains("heading off"), "heading: {all}");
    assert!(why(Man::Square).contains("path off") || why(Man::Square).contains("corner"), "square: {}", why(Man::Square));
    assert!(all.contains("off centre") || all.contains("of centre"), "centring: {all}");
    assert!(why(Man::Loop).contains("too big"), "loop: {}", why(Man::Loop));
    assert!(why(Man::Land).contains("touchdown"), "land: {}", why(Man::Land));
}

#[test]
fn t11_show_judges() {
    // judge statistics: spread of the five marks, drop high / low applied
    let mut rng = helisim::rng::Rng::new(77);
    let mut sd = vec![];
    for show_i in 0..200 {
        let mut bias = [0.0; 5];
        for b in bias.iter_mut() {
            *b = rng.rnd1() * judge::rules::BIAS;
        }
        let base = 5.0 + (show_i % 9) as f64 * 0.5;
        let m = judge::marks(base, false, &bias, &mut rng);
        assert!(m.iter().all(|x| (x * 2.0).fract() == 0.0 && (0.0..=10.0).contains(x)));
        let mean = m.iter().sum::<f64>() / 5.0;
        sd.push((m.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / 4.0).sqrt());
        let (hi, lo) = judge::drop_hi_lo(&m);
        assert!(hi != lo);
        assert!(m.iter().all(|x| *x <= m[hi] && *x >= m[lo]));
        let k = judge::kept_mean(&m, hi, lo);
        let expect = (m.iter().sum::<f64>() - m[hi] - m[lo]) / 3.0;
        assert!((k - expect).abs() < 1e-12);
    }
    let msd = sd.iter().sum::<f64>() / sd.len() as f64;
    // and in a real show
    let (r, _, _) = show_run(1120, false, Part::Hover, true, ShowHook::default(), |_| None);
    let mut sds = vec![];
    for x in &r.res {
        let mean = x.marks.iter().sum::<f64>() / 5.0;
        sds.push((x.marks.iter().map(|m| (m - mean).powi(2)).sum::<f64>() / 4.0).sqrt());
        assert!(x.marks[x.hi] >= x.marks.iter().cloned().fold(0.0, f64::max) && x.marks[x.lo] <= x.marks.iter().cloned().fold(10.0, f64::min));
        assert!((x.ks - x.score * x.k).abs() < 1e-9);
    }
    let zero = judge::marks(7.0, true, &[0.1; 5], &mut rng);
    assert!(zero.iter().all(|x| *x == 0.0));
    println!("RUST SHOW judges: mean std of the 5 marks {:.2} points over 200 synthetic manoeuvres (range {:.2}..{:.2}); in the hover show {:.2}", msd, sd.iter().cloned().fold(9.0, f64::min), sd.iter().cloned().fold(0.0, f64::max), sds.iter().sum::<f64>() / sds.len() as f64);
    assert!((0.3..=0.6).contains(&msd), "judge spread {msd:.2}");
}

/// Stick pilot for the hover square: flies the true heli through the corners (stopping at each)
/// at the hovering height, tail in; ALT HOLD on the left stick (centred = hold).
struct SquarePilot {
    wps: Vec<(f64, f64)>,
    k: usize,
    t_stop: f64,
    started: bool,
    ix: f64,
    iz: f64,
}
impl SquarePilot {
    fn new() -> SquarePilot {
        let (x, z, l) = (show::geo::HX, show::geo::HZ, show::geo::SQ);
        SquarePilot { wps: vec![(x + l / 2.0, z), (x + l / 2.0, z - l), (x - l / 2.0, z - l), (x - l / 2.0, z), (x, z)], k: 0, t_stop: 0.0, started: false, ix: 0.0, iz: 0.0 }
    }
    fn input(&mut self, s: &Sim) -> Option<Input> {
        let sh = s.show.as_ref()?;
        if sh.ph != Ph::Ready && sh.ph != Ph::Fly {
            return None;
        }
        let h = &s.h;
        // throttle: ALT HOLD; hold height (centre = hover stick), nudge toward 1.75 m
        let hs = hover_stick(s.set.pitch3d);
        let eh = show::geo::HA - h.alt();
        let thr = hs + if eh.abs() > 0.1 { (0.1 + 0.15 * eh.abs().min(1.0)) * eh.signum() } else { 0.0 };
        if !self.started {
            self.started = true;
        }
        let (tx, tz) = self.wps[self.k.min(self.wps.len() - 1)];
        let (ex, ez) = (tx - h.p.x, tz - h.p.z);
        let d = ex.hypot(ez);
        if d < 0.25 && h.v.hyp_xz() < 0.2 {
            self.t_stop += DT * 4.0;
            if self.t_stop > 1.2 && self.k < self.wps.len() - 1 {
                self.k += 1;
                self.t_stop = 0.0;
            }
        }
        let vm = (0.8 * d).min(0.9);
        let (vdx, vdz) = if d > 1e-6 { (ex / d * vm, ez / d * vm) } else { (0.0, 0.0) };
        // a pilot trims out what the cyclic needs to hold a spot (wind, flapback): integral
        self.ix = (self.ix + 0.4 * ex * DT * 4.0).clamp(-0.8, 0.8);
        self.iz = (self.iz + 0.4 * ez * DT * 4.0).clamp(-0.8, 0.8);
        let (ax, az) = (1.5 * (vdx - h.v.x) + self.ix, 1.5 * (vdz - h.v.z) + self.iz);
        let (fwd, right) = fwd_right(h.q);
        let lim = 15f64.to_radians();
        let pa = ((ax * fwd.x + az * fwd.z) / 9.81).atan().clamp(-lim, lim);
        let ra = ((ax * right.x + az * right.z) / 9.81).atan().clamp(-lim, lim);
        let yaw = (-(2.0 * wrap(0.0 - heading_of(h.q))) / 4.5).clamp(-0.5, 0.5);
        Some(Input { pit: expo_inv(pa / MAX_TILT), rol: expo_inv(ra / MAX_TILT), yaw: expo_inv(yaw), thr_set: Some(thr), ..Input::default() })
    }
}

#[test]
fn t11_show_manual_square() {
    // fly it yourself: the autopilot positions at the start circle and hands over; a scripted
    // stick pilot (ALT HOLD on) flies the square; start and finish detected, judged
    let mut s = Sim::new(1130);
    s.set.alt_hold = true;
    s.show_ui.demo = false;
    s.show_ui.part = Part::Hover;
    s.command(Command::ShowStart);
    let mut p = SquarePilot::new();
    let (mut t_ready, mut t_fly, mut t_judged) = (None, None, None);
    for i in 0..240 * 200 {
        if i % 4 == 0 {
            if let Some(inp) = p.input(&s) {
                s.input(&inp);
            }
        }
        s.step(DT);
        let sh = s.show.as_ref().unwrap();
        match sh.ph {
            Ph::Ready if t_ready.is_none() => t_ready = Some(s.sim_t),
            Ph::Fly if t_fly.is_none() => t_fly = Some(s.sim_t),
            Ph::Judged if t_judged.is_none() => {
                t_judged = Some(s.sim_t);
                break;
            }
            _ => {}
        }
        if std::env::var("SQDBG").is_ok() && i % 240 == 0 {
            println!("dbg t {:.1} ph {:?} p ({:.2} {:.2} {:.2}) v {:.2} k {} mode {:?} ah {} thr {:.2}", s.sim_t, sh.ph, s.h.p.x, s.h.alt(), s.h.p.z, s.h.v.hyp_xz(), p.k, mode_of(&s), s.ah.active, s.sticks.thr);
        }
        if i % 240 == 0 && sh.ph == Ph::Ready {
            assert!(strip_tags(&s.cue()).contains("GO"), "{}", s.cue());
        }
    }
    let sh = s.show.as_ref().unwrap();
    let r = sh.res.last().expect("square judged");
    println!(
        "RUST SHOW manual square: hand-over at {:.1} s, start detected at {:.1} s, finish at {:.1} s ({:.1} s flown) | base {:.2} marks {:?} -> {:.2} | {}",
        t_ready.unwrap(), t_fly.unwrap(), t_judged.unwrap(), t_judged.unwrap() - t_fly.unwrap(), r.base, r.marks, r.score, r.why.join("; ")
    );
    assert!(r.m == Man::Square && r.zero.is_none() && r.score > 3.0, "{r:?}");
    assert!(t_judged.unwrap() - t_fly.unwrap() > 12.0);
    assert!(s.auto.is_none(), "the pilot flies");
    // the next manoeuvre: the autopilot positions again
    s.steps(240 * 4);
    assert!(s.show.as_ref().is_some_and(|sh| sh.i == 1));
    assert_eq!(mode_of(&s), Some(Mode::Show));
    // DONE before the start skips it (scores 0) and STOP ends the show with the results so far
    for _ in 0..240 * 30 {
        s.step(DT);
        if s.show.as_ref().unwrap().ph == Ph::Ready {
            break;
        }
    }
    let sh = s.show.as_ref().unwrap();
    assert_eq!(sh.ph, Ph::Ready, "triangle ready {:?}", sh.ph);
    s.command(Command::ShowDone);
    s.steps(24);
    assert_eq!(s.show.as_ref().unwrap().res.len(), 2);
    assert!(s.show.as_ref().unwrap().res[1].zero.is_some());
    s.command(Command::ShowStop);
    let r = s.show_last.as_ref().unwrap();
    assert!(!r.complete && r.res.len() == 2);
}

#[test]
fn t11_show_rescue() {
    // a collective cut in the loop: crash protection takes over, the loop scores 0, the show
    // continues with the roll and the rest
    let hook = ShowHook { cut: Some((Man::Loop as u8, 3.5, -0.6)), ..ShowHook::default() };
    let (r, crashed, resc) = show_run(1140, false, Part::Aero, true, hook, |_| None);
    show_line("rescue in the loop", &r);
    for x in &r.res {
        println!("    {:<22} {:.2} | {}", x.m.name(), x.score, x.why.join("; "));
    }
    assert!(!crashed && resc >= 1, "crashed {crashed} rescues {resc}");
    let lp = &r.res[0];
    assert!(lp.m == Man::Loop && lp.score == 0.0 && lp.zero.as_deref().unwrap_or("").starts_with("crash protection"), "{lp:?}");
    assert!(r.complete && r.res.len() == 5 && r.res[1..].iter().all(|x| x.score > 5.0));
}

#[test]
fn t11_show_frame_and_commands() {
    use helisim::wasm::*;
    sim_init(5.0);
    sim_cmd(Command::Show as u32);
    let fr = || {
        let n = sim_frame() as usize;
        unsafe { std::str::from_utf8(std::slice::from_raw_parts(sim_out_ptr(), n)).unwrap().to_string() }
    };
    let s = fr();
    assert!(s.contains("\"show\":{\"sheet\":true"), "{}", &s[s.find("\"show\"").unwrap()..][..200]);
    sim_cmd(Command::ShowHover as u32);
    sim_cmd(Command::ShowStart as u32);
    sim_ff(5.0);
    let s = fr();
    assert!(s.contains("\"on\":true") && s.contains("\"ghost\":[[") && s.contains("\"man\":\"Hover square\""), "{s}");
    // the ghost is sent once per manoeuvre
    let s2 = fr();
    assert!(!s2.contains("\"ghost\""));
    sim_ff(40.0);
    let s = fr();
    assert!(s.contains("\"card\":{") || s.contains("\"ph\":"), "{s}");
    sim_ff(140.0);
    let s = fr();
    assert!(s.contains("\"last\":{") && s.contains("\"rows\":[") && s.contains("Show complete"), "{s}");
    println!("RUST SHOW frame: {} bytes with the scoresheet", s.len());
}
