//! Physical constants and unit conversions. Internal units are SI (m, s, rad); headspeed is a
//! fraction of the governed 2350 rpm, battery charge a fraction of a full 12S pack.

pub const G: f64 = 9.81;
pub const DEG: f64 = core::f64::consts::PI / 180.0;
pub const PI: f64 = core::f64::consts::PI;

/// Governed headspeed in rpm (rpm fraction 1.0).
pub const RPM_FULL: f64 = 2350.0;

pub fn deg(rad: f64) -> f64 {
    rad / DEG
}
pub fn rpm(frac: f64) -> f64 {
    frac * RPM_FULL
}
pub fn pct(frac: f64) -> f64 {
    frac * 100.0
}
