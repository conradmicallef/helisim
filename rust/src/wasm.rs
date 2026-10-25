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
    /// last ghost path id sent to the page (0 = none)
    static GID: RefCell<u32> = const { RefCell::new(0) };
}

/// SHOW test hook: fast-forward. Runs `secs` of sim time (whole 1/240 s steps) without frames,
/// for page tests of the show (the page itself never calls it).
#[no_mangle]
pub extern "C" fn sim_ff(secs: f64) {
    let n = (secs.max(0.0) * 240.0).round() as u32;
    with(|s| s.steps(n));
}

/// Force the ghost path into the next frame (the page lost it, e.g. after a reload of the view).
#[no_mangle]
pub extern "C" fn sim_ghost_reset() {
    GID.with(|g| *g.borrow_mut() = 0);
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
    // assists: settings, ALT HOLD engaged, climb bar (v/s, colour class, ALT HOLD climb-rate target)
    j.bool("altHold", s.set.alt_hold);
    j.bool("autoYaw", s.set.auto_yaw);
    j.bool("tailRate", s.set.tail_rate);
    j.bool("ahOn", s.ah.active);
    let (vsv, vsc) = s.vsi();
    j.num("vsi", vsv);
    j.str("vsiC", vsc);
    j.num("vsiT", if s.ah.active { s.ah.vz } else { -99.0 });
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
    j.bool("rpmShow", hud.rpm_show);
    j.str("health", hud.health.0);
    j.str("healthC", hud.health.1);
    j.end_obj();
    // downward camera picture-in-picture: shown, lock state, last detection box (image px of a
    // 640 x 480 frame, recent only), camera mount for the TS camera
    j.bool("pip", s.pip_show());
    j.bool("pipSet", s.set.pip);
    {
        let pl = &s.sens.pl;
        j.str("camSt", pl.status(s.sens.cam.enabled));
        j.int("camN", s.sens.cam.n_det as i64);
        j.bool("camLock", s.sens.cam.enabled && pl.tracking());
        j.bool("camTgt", s.auto.as_ref().is_some_and(|a| a.cam_tgt));
        match pl.last_det {
            Some((t, d)) if s.sim_t - t < 0.25 => {
                j.arr("camDet");
                for v in [d.u, d.v, d.span, d.yaw] {
                    j.raw_num_item(v);
                }
                j.end_arr();
            }
            _ => {
                j.arr("camDet");
                j.end_arr();
            }
        }
    }
    // SHOW
    let gid = GID.with(|g| *g.borrow());
    write_show(s, j, gid);
    if let Some(sh) = &s.show {
        GID.with(|g| *g.borrow_mut() = sh.gid);
    }
    // toasts raised since the last frame
    j.bool("toastClear", s.toast_clear);
    j.arr("toasts");
    for t in s.toasts.iter() {
        j.obj("");
        j.str("t", &t.title);
        j.str("s", &t.sub);
        j.str("k", t.kind);
        j.bool("ev", t.ev);
        j.end_obj();
    }
    j.end_arr();
    s.toasts.clear();
    s.toast_clear = false;
    j.end_obj();
}


/// SHOW: the setup sheet, the live manoeuvre (name, K, phase, ghost path, result cards) and the
/// last show's scoresheet. The ghost path is sent only when its id changes (`gid`).
fn write_show(s: &Sim, j: &mut Json, gid_sent: u32) {
    use crate::jsonw::to_fixed;
    use crate::show::{Man, Part, SCHED};
    let u = &s.show_ui;
    j.obj("show");
    j.bool("sheet", u.sheet);
    j.bool("demo", u.demo);
    j.int("part", u.part as i64);
    j.arr("parts");
    for p in [Part::Full, Part::Hover, Part::Aero] {
        j.obj("");
        j.str("n", p.name());
        let l = p.list();
        j.int("cnt", l.len() as i64);
        j.str("k", &to_fixed(l.iter().map(|m| m.k()).sum::<f64>(), 1));
        j.end_obj();
    }
    j.end_arr();
    j.arr("sched");
    for m in SCHED {
        j.obj("");
        j.str("n", m.name());
        j.str("k", &to_fixed(m.k(), 1));
        j.bool("hov", m.hover());
        j.end_obj();
    }
    j.end_arr();
    j.bool("on", s.show.is_some());
    j.bool("resOpen", u.res_open);
    j.bool("hasRes", s.show_last.is_some());
    let card = |j: &mut Json, k: &str, r: &crate::show::ManRes| {
        j.obj(k);
        j.str("n", r.m.name());
        j.str("k", &to_fixed(r.k, 1));
        j.arr("marks");
        for m in r.marks {
            j.str("", &to_fixed(m, 1));
        }
        j.end_arr();
        j.int("hi", r.hi as i64);
        j.int("lo", r.lo as i64);
        j.str("score", &to_fixed(r.score, 2));
        j.str("ks", &to_fixed(r.ks, 2));
        j.bool("zero", r.zero.is_some());
        j.arr("why");
        for w in r.why.iter() {
            j.str("", w);
        }
        j.end_arr();
        j.end_obj();
    };
    if let Some(sh) = &s.show {
        let m = sh.man();
        j.int("i", sh.i as i64);
        j.int("n", sh.list.len() as i64);
        j.str("man", m.name());
        j.str("mk", &to_fixed(m.k(), 1));
        j.int("ph", sh.ph as i64);
        j.bool("live", sh.demo);
        j.bool("hov", m.hover());
        // DONE: fly it yourself, ready or flying
        j.bool("done", !sh.demo && matches!(sh.ph, crate::show::Ph::Ready | crate::show::Ph::Fly));
        j.str("total", &to_fixed(sh.res.iter().map(|r| r.ks).sum::<f64>(), 1));
        j.int("gid", sh.gid as i64);
        if sh.gid != gid_sent {
            j.arr("ghost");
            for p in sh.nom.ghost() {
                j.sep_item();
                j.buf.push('[');
                j.buf_num(p.x);
                j.buf.push(',');
                j.buf_num(p.y);
                j.buf.push(',');
                j.buf_num(p.z);
                j.buf.push(']');
            }
            j.end_arr();
        }
        // trail: record from the start of the manoeuvre (fly-it-yourself: from Ready)
        j.bool("trail", matches!(sh.ph, crate::show::Ph::Fly | crate::show::Ph::Ready));
        if let Some(r) = sh.res.last() {
            if s.sim_t - r.t < crate::show::CARD_T && sh.res.len() == sh.i + 1 {
                card(j, "card", r);
            }
        }
        let _ = Man::Square;
    }
    if let Some(r) = &s.show_last {
        // the full scoresheet (also the result card of the final manoeuvre)
        j.obj("last");
        j.str("title", &format!("{} · {}", r.part.name(), if r.demo { "autopilot demo" } else { "flown by you" }));
        j.str("sens", if r.real { if r.ekf { "REAL sensors, EKF" } else { "REAL sensors, BASIC" } } else { "IDEAL sensors" });
        j.str("total", &to_fixed(r.total, 1));
        j.str("max", &to_fixed(r.max, 0));
        j.str("pct", &to_fixed(r.pct, 1));
        j.bool("complete", r.complete);
        j.str("note", &r.note);
        j.arr("rows");
        for x in r.res.iter() {
            card(j, "", x);
        }
        j.end_arr();
        j.end_obj();
    }
    j.end_obj();
}
