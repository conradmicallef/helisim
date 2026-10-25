// Page entry: loads the wasm core, wires buttons and keys to sim commands, and runs the frame
// loop. Per animation frame: send input, run the elapsed fixed 240 Hz steps (at most 24, like
// the original page), read one JSON frame, render the scene and HUD.

import { Cmd, Engine, type Frame } from './engine';
import { Input } from './input';
import { View3D } from './view3d';
import { Hud } from './hud';

declare global { interface Window { HELI_WASM: string; heli?: { engine: Engine; frame: () => Frame | null; errors: string[] } } }

const DT = 1 / 240, MAX_STEPS = 24;
const $ = (id: string) => document.getElementById(id)!;

async function start() {
  const engine = await Engine.load(window.HELI_WASM, (Math.random() * 2 ** 32) >>> 0);
  const send = (c: number) => engine.cmd(c);
  const hud = new Hud();
  const keyCmd: Record<string, number> = {
    t: Cmd.Takeoff, k: Cmd.Circuit, g: Cmd.Land, u: Cmd.Autorot, ' ': Cmd.Arm, r: Cmd.Reset, c: Cmd.Cam, h: Cmd.Hold,
    l: Cmd.Level, p: Cmd.Pitch, n: Cmd.Wind, x: Cmd.Protect, m: Cmd.Sensors, e: Cmd.Estimator, j: Cmd.Stunts,
    q: Cmd.AltHold, y: Cmd.AutoYaw, v: Cmd.TailRate,
  };
  let frame: Frame | null = null;
  // stunt picker keys while it is open: 1-5, arrows + Enter, Esc
  const pickerKey = (k: string): boolean => {
    if (!frame || !frame.picker) return false;
    const n = frame.stunts.length;
    if (k >= '1' && k <= String(n)) { send(Cmd.StuntFirst + Number(k) - 1); return true; }
    if (k === 'arrowdown' || k === 'arrowup') { hud.sel = (hud.sel + (k === 'arrowdown' ? 1 : n - 1)) % n; return true; }
    if (k === 'enter') { send(Cmd.StuntFirst + hud.sel); return true; }
    if (k === 'escape') { send(Cmd.PickerClose); return true; }
    return false;
  };
  // settings sheet: gear opens it; outside tap, the close button or Esc close it
  const sheet = $('sheet') as HTMLElement, scrim = $('scrim') as HTMLElement;
  const setOpen = (on: boolean) => {
    sheet.hidden = !on; scrim.hidden = !on;
    $('bSet').classList.toggle('on', on); $('bSet').setAttribute('aria-expanded', String(on));
    if (on) ($('bSetX') as HTMLElement).focus({ preventScroll: true }); else if (document.activeElement && sheet.contains(document.activeElement)) ($('bSet') as HTMLElement).focus({ preventScroll: true });
  };
  const sheetKey = (k: string): boolean => { if (k === 'escape' && !sheet.hidden) { setOpen(false); return true; } return false; };
  const input = new Input(send, k => { if (sheetKey(k) || pickerKey(k)) return; if (k in keyCmd) send(keyCmd[k]); }, () => !!(frame && frame.picker));
  const buttons: Record<string, number> = {
    aTake: Cmd.Takeoff, aCirc: Cmd.Circuit, aLand: Cmd.Land, aAuto: Cmd.Autorot, aStunt: Cmd.Stunts, bArm: Cmd.Arm, bHold: Cmd.Hold,
    bProt: Cmd.Protect, bSens: Cmd.Sensors, bEst: Cmd.Estimator, bLvl: Cmd.Level, bPitch: Cmd.Pitch, bWind: Cmd.Wind, bCam: Cmd.Cam, bReset: Cmd.Reset,
    bAlt: Cmd.AltHold, bYaw: Cmd.AutoYaw, bTail: Cmd.TailRate, bPip: Cmd.CamView,
  };
  for (const [id, c] of Object.entries(buttons)) $(id).onclick = () => send(c);
  $('bSet').onclick = () => setOpen(sheet.hidden);
  $('bSetX').onclick = () => setOpen(false);
  scrim.addEventListener('pointerdown', e => { e.preventDefault(); setOpen(false); });
  $('hp').onclick = () => { hud.sensOpen = !hud.sensOpen; };
  // tooltips on desktop only (hover-capable fine pointers); none on touch
  if (!matchMedia('(hover:none) and (pointer:coarse)').matches) {
    for (const b of document.querySelectorAll<HTMLElement>('[data-tip]')) if (b.dataset.tip) b.title = b.dataset.tip;
  }
  $('picker').addEventListener('click', e => {
    const b = (e.target as HTMLElement).closest('button[data-id]') as HTMLButtonElement | null;
    if (b && !b.disabled) send(Cmd.StuntFirst + Number(b.dataset.id));
  });
  $('bHelp').onclick = () => { setOpen(false); $('help').hidden = false; };
  $('bGo').onclick = () => { $('help').hidden = true; };

  const view = new View3D($('gl'));
  let last = performance.now(), acc = 0;
  window.heli = { engine, frame: () => frame, errors: [] };
  window.addEventListener('resize', () => view.resize());
  // #app clips its overflow, but focus or a sliding panel could still scroll it sideways
  const app = $('app'); app.addEventListener('scroll', () => { if (app.scrollLeft || app.scrollTop) app.scrollLeft = app.scrollTop = 0; });

  const loop = (now: number) => {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    const i = input.read(dt);
    engine.input(i.yaw, i.pit, i.rol, i.thrDelta, i.thrSet, i.gp);
    acc += dt; let n = 0;
    while (acc >= DT && n < MAX_STEPS) { acc -= DT; n++; }
    if (n >= MAX_STEPS) acc = 0;
    engine.step(n);
    frame = engine.frame();
    input.drawL(frame.stk[0], frame.stk[1]); input.drawR(frame.stk[2], frame.stk[3]);
    view.render(frame, dt, now, document.getElementById('pip'));
    hud.update(frame, dt);
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

start().catch(e => {
  console.error(e);
  $('cue').textContent = 'Could not start the simulator: ' + (e && e.message ? e.message : String(e));
});
