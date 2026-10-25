//! Simulated F3C-style judging for SHOW mode.
//!
//! Each manoeuvre is compared with its ideal reference (`show::Track`) using objective
//! deviations measured on the TRUE state (what judges see): path shape, height and heading
//! constancy, stops, centring in the box, size, smoothness, entry / exit and touchdowns. Every
//! deviation maps to a downgrade with a "1 point per X beyond a tolerance" rule (in the spirit of
//! the "1 point per 15 degrees" guideline of FAI aerobatics judging; these are not the official
//! F3C rules). Five judges then mark the same base score with a personal bias and noise, rounded
//! to 0.5; the highest and lowest marks are dropped and the mean of the other three times K is
//! the manoeuvre score.

use crate::jsonw::to_fixed;
use crate::math::*;
use crate::rng::Rng;
use crate::show::{geo, Man, Track};
use crate::units::{DEG, PI};

/// One recorded sample of the true state (position y = skid height).
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub t: f64,
    pub p: V3,
    pub v: V3,
    pub up: V3,
    pub nose: V3,
    pub hdg: f64,
    /// body rates, rad/s
    pub w: V3,
    pub gnd: bool,
    pub hold: bool,
}

/// Touchdown: vertical and horizontal speed, where (skid height 0).
#[derive(Clone, Copy, Debug)]
pub struct Touch {
    pub vs: f64,
    pub hs: f64,
    pub p: V3,
}

/// Judged result before the judges' marks: base score and the downgrades that made it.
#[derive(Clone, Debug)]
pub struct Assess {
    pub base: f64,
    /// (points, reason), largest first
    pub downs: Vec<(f64, String)>,
    /// zero score and why (no manoeuvre, crash protection, power on in an autorotation)
    pub zero: Option<String>,
}

/// Judging constants: judges, bias, noise.
pub mod rules {
    pub const JUDGES: usize = 5;
    /// personal bias of each judge, uniform +-BIAS points, drawn once per show
    pub const BIAS: f64 = 0.3;
    /// noise on each mark, 1 sigma, points
    pub const NOISE: f64 = 0.3;
    /// largest downgrade from one rule
    pub const CAP: f64 = 4.0;
}

/// Small insertion sort by a key (the lists are short; keeps the wasm free of the std sort code).
fn isort<T>(v: &mut [T], key: impl Fn(&T) -> f64) {
    for i in 1..v.len() {
        let mut j = i;
        while j > 0 && key(&v[j - 1]) > key(&v[j]) {
            v.swap(j - 1, j);
            j -= 1;
        }
    }
}

struct D(Vec<(f64, String)>);
impl D {
    /// `x` beyond `tol` costs 1 point per `per`, at most `rules::CAP`.
    #[inline(never)]
    fn add(&mut self, x: f64, tol: f64, per: f64, why: &dyn Fn() -> String) {
        if x.is_finite() && x > tol {
            let p = ((x - tol) / per).min(rules::CAP);
            if p >= 0.05 {
                self.0.push((p, why()));
            }
        }
    }
    /// A flat downgrade.
    fn flat(&mut self, p: f64, why: String) {
        self.0.push((p, why));
    }
    fn done(mut self, zero: Option<String>) -> Assess {
        isort(&mut self.0, |d| -d.0);
        let sum: f64 = self.0.iter().map(|d| d.0).sum();
        let base = if zero.is_some() { 0.0 } else { clamp(10.0 - sum, 0.0, 10.0) };
        Assess { base, downs: self.0, zero }
    }
}

pub fn zero(why: impl Into<String>) -> Assess {
    Assess { base: 0.0, downs: Vec::new(), zero: Some(why.into()) }
}

fn pct(v: &[f64], q: f64) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    // sort a decimated copy (at most about 400 values)
    let step = (v.len() / 400).max(1);
    let mut s: Vec<f64> = v.iter().step_by(step).copied().filter(|x| x.is_finite()).collect();
    if s.is_empty() {
        return 0.0;
    }
    isort(&mut s, |x| *x);
    s[((s.len() - 1) as f64 * q).round() as usize]
}
fn median(v: &[f64]) -> f64 {
    pct(v, 0.5)
}
fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        0.0
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}
fn std(v: &[f64]) -> f64 {
    let m = mean(v);
    if v.len() < 2 {
        0.0
    } else {
        (v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / v.len() as f64).sqrt()
    }
}
const ORD: [&str; 6] = ["a stop", "the first leg", "the second leg", "the third leg", "the fourth leg", "the fifth leg"];
fn leg_name(l: u8) -> &'static str {
    ORD.get(l as usize).copied().unwrap_or("a leg")
}
fn side(x: f64) -> &'static str {
    if x < 0.0 {
        "left"
    } else {
        "right"
    }
}

/// Least-squares circle (Kasa) through 2D points: centre and radius.
pub fn fit_circle(pts: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    if pts.len() < 5 {
        return None;
    }
    // [x y 1] [D E F]' = -(x^2 + y^2)
    let mut m = [[0.0f64; 3]; 3];
    let mut b = [0.0f64; 3];
    for &(x, y) in pts {
        let r = [x, y, 1.0];
        let z = -(x * x + y * y);
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += r[i] * r[j];
            }
            b[i] += r[i] * z;
        }
    }
    // Cramer's rule
    let det = |m: &[[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d0 = det(&m);
    if d0.abs() < 1e-9 {
        return None;
    }
    let mut s = [0.0; 3];
    for k in 0..3 {
        let mut mk = m;
        for i in 0..3 {
            mk[i][k] = b[i];
        }
        s[k] = det(&mk) / d0;
    }
    let (cx, cy) = (-s[0] / 2.0, -s[1] / 2.0);
    let r2 = cx * cx + cy * cy - s[2];
    if r2 <= 0.0 {
        return None;
    }
    Some((cx, cy, r2.sqrt()))
}

/// Reference polyline (every 0.1 s) with leg ids.
fn poly(tr: &Track) -> Vec<(V3, u8)> {
    tr.pts.iter().step_by(5).map(|r| (r.p, r.leg)).collect()
}
/// Distance from a point to the polyline (horizontal only, or 3D) and the leg of the nearest segment.
fn near(poly: &[(V3, u8)], p: V3, horiz: bool) -> (f64, u8) {
    let f = |v: V3| if horiz { v3(v.x, 0.0, v.z) } else { v };
    let q = f(p);
    let mut best = (f64::INFINITY, 0u8);
    for w in poly.windows(2) {
        let (a, b) = (f(w[0].0), f(w[1].0));
        let ab = b - a;
        let l2 = ab.length_sq();
        let u = if l2 < 1e-12 { 0.0 } else { clamp((q - a).dot(ab) / l2, 0.0, 1.0) };
        let d = (a + ab * u - q).length();
        if d < best.0 {
            best = (d, if u < 0.5 { w[0].1 } else { w[1].1 });
        }
    }
    best
}

/// Attitude wobble: RMS of the body rate minus its 0.14 s moving average (7 samples), deg/s:
/// oscillation and jerkiness, not the intended rate changes of a figure.
pub fn wobble(rec: &[Sample]) -> f64 {
    let n = rec.len();
    const H: usize = 3;
    if n < 2 * H + 2 {
        return 0.0;
    }
    let mut acc = 0.0;
    let mut k = 0.0;
    for i in H..n - H {
        let mut m = V3::ZERO;
        for s in &rec[i - H..=i + H] {
            m += s.w;
        }
        m *= 1.0 / (2 * H + 1) as f64;
        acc += (rec[i].w - m).length_sq();
        k += 1.0;
    }
    (acc / k).sqrt() / DEG
}

fn hdg_dev(rec: &[Sample], h0: f64) -> Vec<f64> {
    rec.iter().filter(|s| !s.gnd).map(|s| wrap(s.hdg - h0).abs() / DEG).collect()
}

/// Smoothness downgrade: wobble above the tolerance, 1 point per `per` deg/s.
fn smooth(d: &mut D, rec: &[Sample], tol: f64, per: f64) {
    let wb = wobble(rec);
    d.add(wb, tol, per, &|| format!("not smooth: attitude wobble {}°/s", to_fixed(wb, 0)));
}

/// Nearest sample to a point (horizontal or 3D): index and distance.
fn nearest(rec: &[Sample], c: V3, horiz: bool) -> (usize, f64) {
    let mut b = (0, f64::INFINITY);
    for (i, s) in rec.iter().enumerate() {
        let d = if horiz { (s.p.x - c.x).hypot(s.p.z - c.z) } else { (s.p - c).length() };
        if d < b.1 {
            b = (i, d);
        }
    }
    b
}

/// A stop at a corner: some sample within 0.8 m slower than 0.3 m/s.
fn stopped(rec: &[Sample], c: V3, horiz: bool) -> bool {
    rec.iter().any(|s| {
        let d = if horiz { (s.p.x - c.x).hypot(s.p.z - c.z) } else { (s.p - c).length() };
        d < 0.8 && s.v.length() < 0.3
    })
}

/// Assess one manoeuvre. `tr` is the nominal reference, `rec` the recorded flight (start to
/// finish), `td` the touchdown if there was one.
pub fn assess(m: Man, tr: &Track, rec: &[Sample], td: Option<Touch>) -> Assess {
    if rec.len() < 10 {
        return zero("not flown");
    }
    match m {
        Man::Square | Man::Triangle => polygon(m, tr, rec),
        Man::DescPir => desc_pir(tr, rec),
        Man::Eight => eight(tr, rec),
        Man::Land => land(tr, rec, td),
        Man::Loop => looping(tr, rec),
        Man::Roll => roll(tr, rec),
        Man::StallTurn => stall_turn(tr, rec),
        Man::RollCircle => roll_circle(tr, rec),
        Man::Autorot => autorot(rec, td),
    }
}

fn polygon(m: Man, tr: &Track, rec: &[Sample]) -> Assess {
    let mut d = D(Vec::new());
    let horiz = m == Man::Square;
    let pl = poly(tr);
    let air: Vec<&Sample> = rec.iter().filter(|s| !s.gnd).collect();
    // path: 90th percentile of the distance to the reference, and the leg where it was worst
    let mut devs = Vec::new();
    let mut worst = (0.0, 0u8);
    for s in &air {
        let (e, l) = near(&pl, s.p, horiz);
        devs.push(e);
        if e > worst.0 {
            worst = (e, l);
        }
    }
    let dv = pct(&devs, 0.9);
    d.add(dv, 0.25, 0.5, &|| format!("path off by {} m on {}", to_fixed(worst.0, 1), leg_name(worst.1)));
    // height constancy (square: every leg; triangle: z drift toward / away from the judges)
    if horiz {
        let hv: Vec<f64> = air.iter().map(|s| (s.p.y - geo::HA).abs()).collect();
        let h = pct(&hv, 0.9);
        d.add(h, 0.15, 0.3, &|| format!("height varied {} m", to_fixed(h, 1)));
    } else {
        let zv: Vec<f64> = air.iter().map(|s| (s.p.z - geo::HZ).abs()).collect();
        let z = pct(&zv, 0.9);
        d.add(z, 0.25, 0.5, &|| format!("drifted {} m off the plane", to_fixed(z, 1)));
    }
    // heading
    let mut hw = (0.0, 0u8);
    let mut hv = Vec::new();
    for s in &air {
        let e = wrap(s.hdg).abs() / DEG;
        hv.push(e);
        if e > hw.0 {
            hw = (e, near(&pl, s.p, horiz).1);
        }
    }
    let h = pct(&hv, 0.9);
    d.add(h, 3.0, 15.0, &|| format!("heading off {}° on {}", to_fixed(hw.0, 0), leg_name(hw.1)));
    // corners: stops and how close
    let mut corners = Vec::new();
    let nstop = tr.stops.len() - 1; // the last stop is the start circle
    for (i, c) in tr.stops.iter().take(nstop).enumerate() {
        let (j, e) = nearest(rec, *c, horiz);
        corners.push(rec[j].p);
        d.add(e, 0.3, 0.6, &|| format!("corner {} missed by {} m", i + 1, to_fixed(e, 1)));
        if !stopped(rec, *c, horiz) {
            d.flat(0.5, format!("no stop at corner {}", i + 1));
        }
    }
    // size and centring from the corners reached
    let (sz, cen) = if horiz {
        let w = ((corners[0].x - corners[3].x) + (corners[1].x - corners[2].x)) / 2.0;
        let dd = ((corners[0].z - corners[1].z) + (corners[3].z - corners[2].z)) / 2.0;
        let c = v3(mean(&corners.iter().map(|c| c.x).collect::<Vec<_>>()), 0.0, mean(&corners.iter().map(|c| c.z).collect::<Vec<_>>()));
        ((w + dd) / 2.0, (c - tr.centre).hyp_xz())
    } else {
        let w = corners[0].x - corners[1].x;
        let cx = (corners[0].x + corners[1].x) / 2.0;
        (w, (cx - tr.centre.x).abs())
    };
    let what = if horiz { "square" } else { "triangle" };
    let sc = sz / tr.size - 1.0;
    d.add(sc.abs() * 100.0, 8.0, 20.0, &|| format!("{what} {}% too {}", to_fixed(sc.abs() * 100.0, 0), if sc > 0.0 { "big" } else { "small" }));
    d.add(cen, 0.4, 1.0, &|| format!("{what} {} m off centre", to_fixed(cen, 1)));
    // back on the start circle
    let end = rec.last().unwrap().p;
    let st = *tr.stops.last().unwrap();
    let e = (end.x - st.x).hypot(end.z - st.z);
    d.add(e, 0.3, 0.5, &|| format!("finished {} m from the start circle", to_fixed(e, 1)));
    smooth(&mut d, rec, 4.0, 5.0);
    d.done(None)
}

fn desc_pir(tr: &Track, rec: &[Sample]) -> Assess {
    let mut d = D(Vec::new());
    let c = tr.centre;
    let drift = rec.iter().map(|s| (s.p.x - c.x).hypot(s.p.z - c.z)).fold(0.0, f64::max);
    d.add(drift, 0.25, 0.4, &|| format!("drifted {} m off the spot", to_fixed(drift, 1)));
    // rotation: unwrapped heading, yaw rate per sample
    let mut tot = 0.0;
    let mut yr = Vec::new();
    for w in rec.windows(2) {
        let dh = wrap(w[1].hdg - w[0].hdg);
        tot += dh;
        yr.push((w[1].t, dh / (w[1].t - w[0].t).max(1e-3), w[1].v.y));
    }
    let rot = tot.abs() / DEG;
    d.add((rot - 360.0).abs(), 5.0, 15.0, &|| format!("turned {}° instead of 360°", to_fixed(rot, 0)));
    let nom = 2.0 * PI / 8.0;
    let turning: Vec<&(f64, f64, f64)> = yr.iter().filter(|x| x.1.abs() > 0.3 * nom).collect();
    let descending: Vec<&(f64, f64, f64)> = yr.iter().filter(|x| x.2 < -0.1).collect();
    if turning.len() > 10 {
        let n = turning.len();
        let mid: Vec<f64> = turning[n / 10..n - n / 10].iter().map(|x| x.1.abs()).collect();
        let cv = std(&mid) / mean(&mid).max(1e-3) * 100.0;
        d.add(cv, 8.0, 20.0, &|| format!("yaw rate varied {}%", to_fixed(cv, 0)));
        let mv: Vec<f64> = turning[n / 10..n - n / 10].iter().map(|x| -x.2).collect();
        let cvd = std(&mv) / mean(&mv).max(0.05) * 100.0;
        d.add(cvd, 12.0, 25.0, &|| format!("descent rate varied {}%", to_fixed(cvd, 0)));
        if descending.len() > 5 {
            let ds = (descending[0].0 - turning[0].0).abs() + (descending[descending.len() - 1].0 - turning[n - 1].0).abs();
            d.add(ds, 0.6, 1.5, &|| format!("turn and descent {} s out of step", to_fixed(ds, 1)));
        }
    } else {
        return d.done(Some("no pirouette".into()));
    }
    let first: Vec<f64> = rec.iter().filter(|s| s.t < rec[0].t + 1.0).map(|s| s.p.y).collect();
    let last: Vec<f64> = rec.iter().filter(|s| s.t > rec[rec.len() - 1].t - 1.0).map(|s| s.p.y).collect();
    let (h0, h1) = (median(&first), median(&last));
    let (r0, r1) = (tr.pts[0].p.y, tr.pts[tr.pts.len() - 1].p.y);
    let eh = (h0 - r0).abs().max((h1 - r1).abs());
    d.add(eh, 0.25, 0.5, &|| format!("heights {} m to {} m instead of {} to {}", to_fixed(h0, 1), to_fixed(h1, 1), to_fixed(r0, 0), to_fixed(r1, 0)));
    smooth(&mut d, rec, 4.0, 5.0);
    d.done(None)
}

fn eight(tr: &Track, rec: &[Sample]) -> Assess {
    let mut d = D(Vec::new());
    let pl = poly(tr);
    let air: Vec<&Sample> = rec.iter().filter(|s| !s.gnd).collect();
    let mut devs = Vec::new();
    let mut worst = (0.0, 0u8);
    for s in &air {
        let (e, l) = near(&pl, s.p, true);
        devs.push(e);
        if e > worst.0 {
            worst = (e, l);
        }
    }
    let lobe = |l: u8| if l == 2 { "left circle" } else { "right circle" };
    let dv = pct(&devs, 0.9);
    d.add(dv, 0.25, 0.5, &|| format!("path off by {} m on the {}", to_fixed(worst.0, 1), lobe(worst.1)));
    let hv: Vec<f64> = air.iter().map(|s| (s.p.y - geo::HA).abs()).collect();
    let h = pct(&hv, 0.9);
    d.add(h, 0.15, 0.3, &|| format!("height varied {} m", to_fixed(h, 1)));
    let hd: Vec<f64> = hdg_dev(rec, PI);
    let hh = pct(&hd, 0.9);
    let hmax = hd.iter().cloned().fold(0.0, f64::max);
    d.add(hh, 4.0, 15.0, &|| format!("nose-in heading off {}°", to_fixed(hmax, 0)));
    let c = tr.centre;
    let pts = |right: bool| -> Vec<(f64, f64)> {
        rec.iter()
            .filter(|s| s.v.hyp_xz() > 0.4 && if right { s.p.x > c.x + 0.4 } else { s.p.x < c.x - 0.4 })
            .map(|s| (s.p.x, s.p.z))
            .collect()
    };
    match (fit_circle(&pts(true)), fit_circle(&pts(false))) {
        (Some(a), Some(b)) => {
            let mis = (a.2 - b.2).abs();
            d.add(mis, 0.2, 0.5, &|| format!("{} circle {} m smaller", if a.2 < b.2 { "right" } else { "left" }, to_fixed(mis, 1)));
            let sc = (a.2 + b.2) / 2.0 / tr.size - 1.0;
            d.add(sc.abs() * 100.0, 10.0, 20.0, &|| format!("circles {}% too {}", to_fixed(sc.abs() * 100.0, 0), if sc > 0.0 { "big" } else { "small" }));
            let cx = (a.0 + b.0) / 2.0 - c.x;
            let cz = (a.1 + b.1) / 2.0 - c.z;
            let off = cx.hypot(cz);
            d.add(off, 0.4, 1.0, &|| format!("eight {} m off centre", to_fixed(off, 1)));
        }
        _ => return d.done(Some("no figure eight".into())),
    }
    smooth(&mut d, rec, 4.0, 5.0);
    d.done(None)
}

fn land(tr: &Track, rec: &[Sample], td: Option<Touch>) -> Assess {
    let mut d = D(Vec::new());
    let Some(td) = td else { return zero("no touchdown") };
    let pad = tr.centre;
    // touchdown
    d.add(td.vs, 0.45, 0.25, &|| format!("touchdown {} m/s", to_fixed(td.vs, 2)));
    let e = (td.p.x - pad.x).hypot(td.p.z - pad.z);
    d.add(e, 0.15, 0.25, &|| format!("touched down {} m from the pad centre", to_fixed(e, 2)));
    d.add(td.hs, 0.2, 0.3, &|| format!("sliding {} m/s at touchdown", to_fixed(td.hs, 1)));
    // a stop over the pad before the descent
    let stop = rec.iter().any(|s| (s.p.x - pad.x).hypot(s.p.z - pad.z) < 0.6 && s.p.y > geo::HA - 0.5 && s.v.length() < 0.25);
    if !stop {
        d.flat(1.0, "no stop over the pad before the descent".into());
    }
    // the descent: from below HA - 0.3 to touchdown, vertical
    let i0 = rec.iter().position(|s| s.p.y < geo::HA - 0.3 && (s.p.x - pad.x).hypot(s.p.z - pad.z) < 1.5);
    if let Some(i0) = i0 {
        let desc: Vec<&Sample> = rec[i0..].iter().take_while(|s| !s.gnd).collect();
        if let Some(f) = desc.first() {
            let drift = desc.iter().map(|s| (s.p.x - f.p.x).hypot(s.p.z - f.p.z)).fold(0.0, f64::max);
            d.add(drift, 0.15, 0.4, &|| format!("drifted {} m in the descent", to_fixed(drift, 2)));
        }
        let before: Vec<f64> = rec[..i0].iter().filter(|s| !s.gnd).map(|s| (s.p.y - geo::HA).abs()).collect();
        let h = pct(&before, 0.9);
        d.add(h, 0.15, 0.3, &|| format!("height varied {} m before the descent", to_fixed(h, 1)));
    }
    let hd = hdg_dev(rec, 0.0);
    let hh = pct(&hd, 0.9);
    d.add(hh, 4.0, 15.0, &|| format!("heading off {}°", to_fixed(hh, 0)));
    smooth(&mut d, rec, 4.0, 5.0);
    d.done(None)
}

/// Samples inside the aerobatic box (|x| within the box edge), from the first entry.
fn in_box(rec: &[Sample]) -> Vec<Sample> {
    rec.iter().filter(|s| (s.p.x - geo::AX).abs() <= geo::EDGE).cloned().collect()
}
struct Ends {
    h_e: f64,
    z_e: f64,
    v_e: V3,
    h_x: f64,
    z_x: f64,
    v_x: V3,
}
fn ends(w: &[Sample]) -> Ends {
    let (t0, t1) = (w[0].t, w[w.len() - 1].t);
    let a: Vec<&Sample> = w.iter().filter(|s| s.t < t0 + 1.0).collect();
    let b: Vec<&Sample> = w.iter().filter(|s| s.t > t1 - 1.0).collect();
    let m = |v: &Vec<&Sample>, f: &dyn Fn(&Sample) -> f64| median(&v.iter().map(|s| f(s)).collect::<Vec<_>>());
    let mv = |v: &Vec<&Sample>| {
        let mut s = V3::ZERO;
        for x in v {
            s += x.v;
        }
        s * (1.0 / v.len().max(1) as f64)
    };
    Ends { h_e: m(&a, &|s| s.p.y), z_e: m(&a, &|s| s.p.z), v_e: mv(&a), h_x: m(&b, &|s| s.p.y), z_x: m(&b, &|s| s.p.z), v_x: mv(&b) }
}
/// Angle between two horizontal velocity directions, deg (0 if either is slow).
fn track_change(a: V3, b: V3, reverse: bool) -> f64 {
    let (a, b) = (v3(a.x, 0.0, a.z), v3(b.x, 0.0, b.z));
    if a.length() < 1.0 || b.length() < 1.0 {
        return 0.0;
    }
    let b = if reverse { -b } else { b };
    clamp(a.normalize().dot(b.normalize()), -1.0, 1.0).acos() / DEG
}

fn looping(tr: &Track, rec: &[Sample]) -> Assess {
    let mut d = D(Vec::new());
    let w = in_box(rec);
    if w.len() < 30 {
        return zero("not flown in the box");
    }
    let e = ends(&w);
    let fig: Vec<&Sample> = w.iter().filter(|s| s.p.y > e.h_e + 0.8).collect();
    if fig.len() < 10 || !fig.iter().any(|s| s.up.y < -0.5) {
        return zero("no loop");
    }
    let pts: Vec<(f64, f64)> = fig.iter().map(|s| (s.p.x, s.p.y)).collect();
    let Some((cx, cy, r)) = fit_circle(&pts) else { return zero("no loop") };
    let rads: Vec<f64> = pts.iter().map(|p| (p.0 - cx).hypot(p.1 - cy)).collect();
    let rr = rads.iter().cloned().fold(0.0, f64::max) - rads.iter().cloned().fold(1e9, f64::min);
    d.add(rr, 0.6, 1.2, &|| format!("loop radius varied {} m", to_fixed(rr, 1)));
    let sc = r / tr.size - 1.0;
    d.add(sc.abs() * 100.0, 10.0, 25.0, &|| format!("loop {}% too {}", to_fixed(sc.abs() * 100.0, 0), if sc > 0.0 { "big" } else { "small" }));
    d.add((cx - geo::AX).abs(), 1.5, 3.0, &|| format!("loop {} m {} of centre", to_fixed((cx - geo::AX).abs(), 1), side(cx - geo::AX)));
    let lat = w.iter().map(|s| (s.p.z - e.z_e).abs()).fold(0.0, f64::max);
    d.add(lat, 0.4, 1.0, &|| format!("drifted {} m off the line", to_fixed(lat, 1)));
    let dh = e.h_x - e.h_e;
    d.add(dh.abs(), 0.4, 1.0, &|| format!("exited {} m {}", to_fixed(dh.abs(), 1), if dh < 0.0 { "low" } else { "high" }));
    let tc = track_change(e.v_e, e.v_x, false);
    d.add(tc, 4.0, 15.0, &|| format!("exit track {}° off the entry", to_fixed(tc, 0)));
    smooth(&mut d, &w, 8.0, 6.0);
    d.done(None)
}

fn roll(_tr: &Track, rec: &[Sample]) -> Assess {
    let mut d = D(Vec::new());
    let w = in_box(rec);
    if w.len() < 30 {
        return zero("not flown in the box");
    }
    let e = ends(&w);
    let Some(inv) = w.iter().min_by(|a, b| a.up.y.partial_cmp(&b.up.y).unwrap()) else { return zero("no roll") };
    if inv.up.y > -0.7 {
        return zero("no roll");
    }
    let lost = w.iter().map(|s| e.h_e - s.p.y).fold(0.0, f64::max);
    let gain = w.iter().map(|s| s.p.y - e.h_e).fold(0.0, f64::max);
    if lost >= gain {
        d.add(lost, 0.3, 0.6, &|| format!("lost {} m in the roll", to_fixed(lost, 1)));
    } else {
        d.add(gain, 0.3, 0.6, &|| format!("climbed {} m in the roll", to_fixed(gain, 1)));
    }
    let lat = w.iter().map(|s| (s.p.z - e.z_e).abs()).fold(0.0, f64::max);
    d.add(lat, 0.8, 1.0, &|| format!("drifted {} m off the line", to_fixed(lat, 1)));
    let cx = inv.p.x - geo::AX;
    d.add(cx.abs(), 1.5, 3.0, &|| format!("roll {} m {} of centre", to_fixed(cx.abs(), 1), side(cx)));
    let rr: Vec<f64> = w.iter().filter(|s| s.w.z.abs() > 1.0).map(|s| s.w.z.abs()).collect();
    if rr.len() > 8 {
        let n = rr.len();
        let mid = &rr[n / 8..n - n / 8];
        let cv = std(mid) / mean(mid).max(1e-3) * 100.0;
        d.add(cv, 12.0, 20.0, &|| format!("roll rate varied {}%", to_fixed(cv, 0)));
    }
    let tc = track_change(e.v_e, e.v_x, false);
    d.add(tc, 3.0, 15.0, &|| format!("exit track {}° off the entry", to_fixed(tc, 0)));
    let dh = e.h_x - e.h_e;
    d.add(dh.abs(), 0.4, 1.0, &|| format!("exited {} m {}", to_fixed(dh.abs(), 1), if dh < 0.0 { "low" } else { "high" }));
    smooth(&mut d, &w, 8.0, 6.0);
    d.done(None)
}

fn stall_turn(tr: &Track, rec: &[Sample]) -> Assess {
    let mut d = D(Vec::new());
    let w = in_box(rec);
    if w.len() < 30 {
        return zero("not flown in the box");
    }
    let e = ends(&w);
    let top = w.iter().map(|s| s.p.y).fold(-1e9, f64::max);
    let r = tr.size; // pull-up radius
    let vert: Vec<&Sample> = w.iter().filter(|s| s.p.y > e.h_e + r + 0.5 && s.v.y.abs() > 2.0).collect();
    let i_top = w.iter().position(|s| s.p.y >= top - 1e-9).unwrap_or(0);
    let up_nose = w[..i_top.max(1)].iter().any(|s| s.nose.y > 0.7);
    let down_nose = w[i_top..].iter().any(|s| s.nose.y < -0.7);
    if top < e.h_e + r || !up_nose || !down_nose {
        return zero("no stall turn");
    }
    if vert.len() > 4 {
        let xs: Vec<f64> = vert.iter().map(|s| s.p.x).collect();
        let lean = xs.iter().cloned().fold(-1e9, f64::max) - xs.iter().cloned().fold(1e9, f64::min);
        d.add(lean, 0.5, 1.0, &|| format!("vertical line wandered {} m", to_fixed(lean, 1)));
        let cx = mean(&xs) - geo::AX;
        d.add(cx.abs(), 1.5, 3.0, &|| format!("{} m {} of centre", to_fixed(cx.abs(), 1), side(cx)));
    }
    let apex: Vec<&Sample> = w.iter().filter(|s| s.p.y > top - 1.5).collect();
    let rx = apex.iter().map(|s| s.p.x).fold(-1e9, f64::max) - apex.iter().map(|s| s.p.x).fold(1e9, f64::min);
    let rz = apex.iter().map(|s| s.p.z).fold(-1e9, f64::max) - apex.iter().map(|s| s.p.z).fold(1e9, f64::min);
    let piv = rx.hypot(rz);
    d.add(piv, 0.6, 1.0, &|| format!("pivot wandered {} m at the top", to_fixed(piv, 1)));
    let climb = top - e.h_e;
    let nom = tr.pts.iter().map(|p| p.p.y).fold(-1e9, f64::max) - tr.pts[0].p.y;
    let sc = climb / nom - 1.0;
    d.add(sc.abs() * 100.0, 15.0, 25.0, &|| format!("climb {}% too {}", to_fixed(sc.abs() * 100.0, 0), if sc > 0.0 { "high" } else { "short" }));
    let dh = e.h_x - e.h_e;
    d.add(dh.abs(), 0.5, 1.0, &|| format!("pulled out {} m {}", to_fixed(dh.abs(), 1), if dh < 0.0 { "low" } else { "high" }));
    let dz = (e.z_x - e.z_e).abs();
    d.add(dz, 0.3, 0.8, &|| format!("exit line {} m off the entry line", to_fixed(dz, 1)));
    let tc = track_change(e.v_e, e.v_x, true);
    d.add(tc, 4.0, 15.0, &|| format!("exit track {}° off the reciprocal", to_fixed(tc, 0)));
    smooth(&mut d, &w, 8.0, 6.0);
    d.done(None)
}

fn roll_circle(tr: &Track, rec: &[Sample]) -> Assess {
    let mut d = D(Vec::new());
    let w = in_box(rec);
    if w.len() < 30 {
        return zero("not flown in the box");
    }
    let e = ends(&w);
    let circ: Vec<&Sample> = w.iter().filter(|s| s.p.z < e.z_e - 1.5).collect();
    if circ.len() < 20 {
        return zero("no circle");
    }
    let Some(inv) = circ.iter().min_by(|a, b| a.up.y.partial_cmp(&b.up.y).unwrap()) else { return zero("no circle") };
    if inv.up.y > -0.5 {
        return zero("no roll in the circle");
    }
    // shape against the reference (two half circles and the roll chord), horizontal
    let pl = poly(tr);
    let mut worst = 0.0f64;
    let devs: Vec<f64> = circ.iter().map(|s| { let x = near(&pl, s.p, true).0; worst = worst.max(x); x }).collect();
    let dv = pct(&devs, 0.9);
    d.add(dv, 0.8, 1.2, &|| format!("circle off the round by {} m", to_fixed(worst, 1)));
    // size: depth of the circle away from the judges
    let depth = e.z_e - circ.iter().map(|s| s.p.z).fold(1e9, f64::min);
    let sc = depth / (2.0 * tr.size) - 1.0;
    d.add(sc.abs() * 100.0, 10.0, 25.0, &|| format!("circle {}% too {}", to_fixed(sc.abs() * 100.0, 0), if sc > 0.0 { "big" } else { "small" }));
    let hv = circ.iter().map(|s| (s.p.y - e.h_e).abs()).fold(0.0, f64::max);
    d.add(hv, 0.6, 1.0, &|| format!("height varied {} m", to_fixed(hv, 1)));
    // the roll at the far point: inverted closest to the far side, centred on the box
    let far_z = circ.iter().map(|s| s.p.z).fold(1e9, f64::min);
    let rz = inv.p.z - far_z;
    let rx = inv.p.x - geo::AX;
    let roff = rx.hypot(rz);
    d.add(roff, 1.5, 2.5, &|| format!("roll {} m away from the far point", to_fixed(roff, 1)));
    let xs: Vec<f64> = circ.iter().map(|s| s.p.x).collect();
    let cx = (xs.iter().cloned().fold(-1e9, f64::max) + xs.iter().cloned().fold(1e9, f64::min)) / 2.0 - geo::AX;
    d.add(cx.abs(), 1.5, 3.0, &|| format!("circle {} m {} of centre", to_fixed(cx.abs(), 1), side(cx)));
    let tc = track_change(e.v_e, e.v_x, false);
    d.add(tc, 5.0, 15.0, &|| format!("exit track {}° off the entry", to_fixed(tc, 0)));
    let dz = (e.z_x - e.z_e).abs();
    d.add(dz, 0.5, 1.0, &|| format!("exit line {} m off the entry line", to_fixed(dz, 1)));
    smooth(&mut d, &w, 8.0, 6.0);
    d.done(None)
}

fn autorot(rec: &[Sample], td: Option<Touch>) -> Assess {
    let mut d = D(Vec::new());
    let Some(i0) = rec.iter().position(|s| s.hold && !s.gnd) else { return zero("no autorotation: throttle hold never on") };
    let Some(td) = td else { return zero("no touchdown") };
    let w: Vec<&Sample> = rec[i0..].iter().take_while(|s| !s.gnd).collect();
    if w.iter().any(|s| !s.hold) {
        return zero("power on before touchdown");
    }
    d.add(td.vs, 0.6, 0.4, &|| format!("touchdown {} m/s", to_fixed(td.vs, 2)));
    let e = td.p.x.hypot(td.p.z);
    d.add(e, 1.2, 1.5, &|| format!("landed {} m from the pad", to_fixed(e, 1)));
    d.add(td.hs, 1.0, 1.0, &|| format!("sliding {} m/s at touchdown", to_fixed(td.hs, 1)));
    // the steady glide: above the flare height
    let glide: Vec<&&Sample> = w.iter().filter(|s| s.p.y > 7.0 && s.p.y < w[0].p.y - 3.0).collect();
    if glide.len() > 5 {
        let p0 = w[0].p;
        let (ux, uz) = { let (dx, dz) = (-p0.x, -p0.z); let l = dx.hypot(dz).max(1e-6); (dx / l, dz / l) };
        let lat = glide.iter().map(|s| ((s.p.x - p0.x) * uz - (s.p.z - p0.z) * ux).abs()).fold(0.0, f64::max);
        d.add(lat, 0.8, 1.5, &|| format!("glide wandered {} m off the line", to_fixed(lat, 1)));
        let vys: Vec<f64> = glide.iter().map(|s| s.v.y).collect();
        let sv = std(&vys);
        d.add(sv, 0.4, 0.8, &|| format!("descent rate varied {} m/s", to_fixed(sv, 1)));
        let ht = (-ux).atan2(-uz);
        let hd: Vec<f64> = glide.iter().map(|s| wrap(s.hdg - ht).abs() / DEG).collect();
        let hh = pct(&hd, 0.9);
        d.add(hh, 5.0, 15.0, &|| format!("heading off {}° in the glide", to_fixed(hh, 0)));
    }
    let ws: Vec<Sample> = w.iter().map(|s| **s).collect();
    smooth(&mut d, &ws, 6.0, 6.0);
    d.done(None)
}

/// The five judges' marks for a base score: base + bias + noise, rounded to 0.5, clamped 0..10.
/// A zero (no manoeuvre, crash protection) is zero from every judge.
pub fn marks(base: f64, zero: bool, bias: &[f64; rules::JUDGES], rng: &mut Rng) -> [f64; rules::JUDGES] {
    let mut m = [0.0; rules::JUDGES];
    for (j, b) in bias.iter().enumerate() {
        let n = rng.randn();
        m[j] = if zero { 0.0 } else { clamp(((base + b + rules::NOISE * n) * 2.0).round() / 2.0, 0.0, 10.0) };
    }
    m
}

/// Index of the highest and the lowest mark (dropped). Ties: the first highest, the last lowest.
pub fn drop_hi_lo(m: &[f64; rules::JUDGES]) -> (usize, usize) {
    let mut hi = 0;
    let mut lo = rules::JUDGES - 1;
    for i in 0..rules::JUDGES {
        if m[i] > m[hi] {
            hi = i;
        }
    }
    for i in (0..rules::JUDGES).rev() {
        if m[i] < m[lo] && i != hi {
            lo = i;
        }
    }
    if lo == hi {
        lo = if hi == 0 { 1 } else { 0 };
    }
    (hi, lo)
}

/// Mean of the three remaining marks.
pub fn kept_mean(m: &[f64; rules::JUDGES], hi: usize, lo: usize) -> f64 {
    let mut s = 0.0;
    for (i, x) in m.iter().enumerate() {
        if i != hi && i != lo {
            s += x;
        }
    }
    s / (rules::JUDGES - 2) as f64
}
