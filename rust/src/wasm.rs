//! C-ABI interface for the page. One sim instance lives in a thread_local RefCell; only plain
//! numbers cross the boundary. Per animation frame the page calls `sim_input`, any `sim_cmd`s,
//! `sim_step(n)` for the elapsed fixed 240 Hz steps, then `sim_frame()` and reads the JSON frame
//! (length returned) from `sim_out_ptr()`.

use crate::autopilot::Mode;
use crate::jsonw::Json;
use crate::sim::{Command, Input, Sim, CAMS};
use crate::wind::{wind_at, WINDS};
use std::cell::RefCell;

thread_local! {
    static SIM: RefCell<Option<Sim>> = const { RefCell::new(None) };
    static OUT: RefCell<Json> = RefCell::new(Json::new());
    static INPUT: RefCell<Input> = RefCell::new(Input::default());
}

fn with<R>(f: impl FnOnce(&mut Sim) -> R) -> R {
    SIM.with(|s| {
        let mut s = s.borrow_mut();
        if s.is_none() {
            *s = Some(Sim::new(1));
        }
        f(s.as_mut().unwrap())
    })
}

/// (Re)create the sim with a seed for the random sources.
#[no_mangle]
pub extern "C" fn sim_init(seed: f64) {
    SIM.with(|s| *s.borrow_mut() = Some(Sim::new(seed as u64)));
}

/// Pilot input for this frame. `thr_set` < 0 means "touch stick did not move".
/// `gp` = 1 when a gamepad with 4+ axes is present, then `g0..g3` are its axes.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn sim_input(yaw: f64, pit: f64, rol: f64, thr_delta: f64, thr_set: f64, gp: u32, g0: f64, g1: f64, g2: f64, g3: f64) {
    let inp = Input {
        yaw,
        pit,
        rol,
        thr_delta,
        thr_set: if thr_set >= 0.0 { Some(thr_set) } else { None },
        gp: if gp != 0 { Some([g0, g1, g2, g3]) } else { None },
    };
    INPUT.with(|i| *i.borrow_mut() = inp);
    with(|s| s.input(&inp));
}

/// Button / key command, see `sim::Command` for codes.
#[no_mangle]
pub extern "C" fn sim_cmd(code: u32) {
    if let Some(c) = Command::from_code(code) {
        with(|s| s.command(c));
    }
}

/// Run `n` fixed steps of 1/240 s.
#[no_mangle]
pub extern "C" fn sim_step(n: u32) {
    with(|s| s.steps(n));
}

/// Build the JSON frame. Returns its byte length; read it at `sim_out_ptr()`.
#[no_mangle]
pub extern "C" fn sim_frame() -> u32 {
    OUT.with(|o| {
        let mut j = o.borrow_mut();
        j.clear();
        with(|s| write_frame(s, &mut j));
        j.buf.len() as u32
    })
}

#[no_mangle]
pub extern "C" fn sim_out_ptr() -> *const u8 {
    OUT.with(|o| o.borrow().buf.as_ptr())
}

/// Scratch allocation for the host (unused by the page today, kept for parity with circuitsim).
#[no_mangle]
pub extern "C" fn sim_alloc(n: u32) -> *mut u8 {
    let mut v = Vec::<u8>::with_capacity(n as usize);
    let p = v.as_mut_ptr();
    core::mem::forget(v);
    p
}

fn write_frame(s: &mut Sim, j: &mut Json) {
    j.obj("");
    let h = &s.h;
    j.v3("p", h.p);
    j.quat("q", h.q);
    j.v3("ep", s.est.p);
    j.quat("eq", s.est.q);
    j.num("rpm", h.rpm);
    j.bool("armed", h.armed);
    j.bool("crashed", h.crashed);
    j.bool("hold", h.hold);
    j.num("simT", s.sim_t);
    let w = wind_at(s.sim_t, s.set.wind);
    j.v3("wind", w);
    // settings
    j.bool("level", s.set.level);
    j.bool("p3d", s.set.pitch3d);
    j.bool("protect", s.set.protect);
    j.bool("real", s.set.real);
    j.bool("ekf", s.set.ekf);
    // 2-sigma horizontal uncertainty radius of the estimate (0 when not on the EKF)
    j.num("sig2", if s.set.real && s.set.ekf && s.sens.ekf.init { 2.0 * s.sens.ekf.sig_pos() } else { 0.0 });
    j.str("windName", WINDS[s.set.wind].name);
    j.str("cam", CAMS[s.set.cam]);
    // autopilot state codes for the buttons
    let m = s.auto.as_ref().map(|a| a.mode as i64).unwrap_or(-1);
    j.int("mode", m);
    j.bool("rescue", m == Mode::Rescue as i64);
    // stunts: picker list (names, descriptions, keys, best scores), live phase, last result
    j.bool("picker", s.picker);
    j.arr("stunts");
    for (i, k) in crate::stunt::KINDS.iter().enumerate() {
        j.obj("");
        j.int("id", i as i64);
        j.str("n", k.name());
        j.str("d", k.desc());
        j.str("key", ["1", "2", "3", "4", "5"][i]);
        j.int("best", s.stunt_best[i].map(|b| b as i64).unwrap_or(-1));
        j.end_obj();
    }
    j.end_arr();
    let live = s.auto.as_ref().and_then(|a| a.st.as_ref());
    j.int("stuntK", live.map(|x| x.kind as i64).unwrap_or(-1));
    j.int("stuntPh", live.map(|x| x.phase as i64).unwrap_or(-1));
    j.bool("stuntBlocked", s.est.bat < 0.25 || s.h.crashed || s.auto.as_ref().is_some_and(|a| a.forced));
    j.int("stuntN", s.stunt_count as i64);
    if let Some(r) = &s.stunt_last {
        j.obj("sres");
        j.str("n", r.kind.name());
        j.int("score", r.score as i64);
        j.str("lost", &crate::jsonw::to_fixed(r.height_lost, 1));
        j.str("dev", &crate::jsonw::to_fixed(r.max_dev, 1));
        j.str("hdg", &crate::jsonw::to_fixed(r.hdg_err.abs(), 0));
        j.str("rec", &crate::jsonw::to_fixed(r.rec_t, 1));
        j.str("minAlt", &crate::jsonw::to_fixed(r.min_alt, 1));
        j.end_obj();
    }
    // sticks shown on the gimbals
    let (lx, ly, rx, ry) = s.stick_display();
    j.arr("stk");
    for v in [lx, ly, rx, ry] {
        j.raw_num_item(v);
    }
    j.end_arr();
    // HUD text
    let hud = s.hud();
    j.obj("hud");
    j.str("thr", &hud.thr);
    j.str("rpm", &hud.rpm);
    j.str("rpmC", hud.rpm_cls);
    j.str("alt", &hud.alt);
    j.str("vs", &hud.vs);
    j.str("vsC", hud.vs_cls);
    j.str("spd", &hud.spd);
    j.str("bat", &hud.bat);
    j.str("batC", hud.bat_cls);
    j.str("land", &hud.land);
    j.str("time", &hud.time);
    j.str("resc", &hud.resc);
    j.str("cue", &hud.cue);
    j.str("sens", &hud.sens);
    j.end_obj();
    // toasts raised since the last frame
    j.bool("toastClear", s.toast_clear);
    j.arr("toasts");
    for t in s.toasts.iter() {
        j.obj("");
        j.str("t", &t.title);
        j.str("s", &t.sub);
        j.str("k", t.kind);
        j.end_obj();
    }
    j.end_arr();
    s.toasts.clear();
    s.toast_clear = false;
    j.end_obj();
}

