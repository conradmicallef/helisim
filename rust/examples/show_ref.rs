//! Print a SHOW reference: `cargo run --release --example show_ref -- <man idx 0..9>`
use helisim::show::{build, ShowHook, SCHED};
fn main() {
    let i: usize = std::env::args().nth(1).and_then(|x| x.parse().ok()).unwrap_or(6);
    let m = SCHED[i];
    let t = build(m, &ShowHook::default());
    let (mut zmin, mut zmax, mut ymin, mut ymax) = (1e9f64, -1e9f64, 1e9f64, -1e9f64);
    for (k, r) in t.pts.iter().enumerate() {
        if (r.p.x).abs() < 25.0 && r.leg >= 2 {
            zmin = zmin.min(r.p.z); zmax = zmax.max(r.p.z); ymin = ymin.min(r.p.y); ymax = ymax.max(r.p.y);
        }
        if k % 5 == 0 && std::env::args().any(|a| a == "v") {
            println!("{:6.2} leg {} p ({:6.2} {:6.2} {:6.2}) v ({:5.1} {:5.1} {:5.1}) a ({:5.1} {:5.1} {:5.1}) up ({:5.2} {:5.2} {:5.2}) w ({:5.2} {:5.2} {:5.2}) att {:.1}", k as f64 * 0.02, r.leg, r.p.x, r.p.y, r.p.z, r.v.x, r.v.y, r.v.z, r.a.x, r.a.y, r.a.z, r.up.x, r.up.y, r.up.z, r.w.x, r.w.y, r.w.z, r.att);
        }
    }
    println!("{:?} dur {:.1} s, in box: z {:.2}..{:.2} y {:.2}..{:.2}", m, t.dur, zmin, zmax, ymin, ymax);
}
