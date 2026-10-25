//! helisim: a 600-class flybarless RC helicopter on a flying field.
//!
//! Rust owns all simulation, control, autopilot, sensor and rule logic; the page (TypeScript +
//! three.js) only renders, gathers input and calls the C-ABI exports in `wasm`.
//!
//! - `math`: Vec3 / quaternion with three.js semantics
//! - `units`, `rng`, `wind`
//! - `heli`: rigid body, rotor aerodynamics, powertrain, skid contact, crash detection
//! - `fbl`: flybarless gyro, self-level, heading-hold tail
//! - `sensors` + `estimator` + `ekf`: simulated sensors on their own clocks, complementary filter
//!   (BASIC) and error-state extended Kalman filter (EKF)
//! - `autopilot`: takeoff, hold, circuit, land, rescue, autorotation, battery failsafe
//! - `stunt`: STUNTS (pirouette, loop, roll, flip, inverted hover) and their scoring
//! - `sim`: world step, commands, landing grading, toasts, HUD and cue text
//! - `jsonw`, `wasm`: frame serialisation and the page interface

pub mod autopilot;
pub mod ekf;
pub mod estimator;
pub mod fbl;
pub mod heli;
pub mod jsonw;
pub mod math;
pub mod rng;
pub mod sensors;
pub mod sim;
pub mod stunt;
pub mod units;
pub mod wasm;
pub mod wind;

pub use sim::Sim;
