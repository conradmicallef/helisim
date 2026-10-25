//! Wind: three presets, blowing from the pilot's right toward the left (-x), with a
//! deterministic two-sine gust pattern and a small crosswind component.

use crate::math::{v3, V3};

pub struct WindPreset {
    pub name: &'static str,
    pub v: f64,
}
pub const WINDS: [WindPreset; 3] =
    [WindPreset { name: "CALM", v: 0.0 }, WindPreset { name: "BREEZE", v: 2.2 }, WindPreset { name: "GUSTY", v: 4.5 }];

/// World wind vector at time t for preset index `level`.
pub fn wind_at(t: f64, level: usize) -> V3 {
    let base = WINDS[level].v;
    let g = base * (0.35 * (t * 0.7).sin() * (t * 0.23 + 2.0).sin() + 0.25 * (t * 1.9 + 1.0).sin());
    v3(-(base + g), 0.0, 0.3 * g)
}
