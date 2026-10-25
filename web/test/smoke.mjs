// Smoke test of the built page without a browser.
// (1) Loads the wasm from the page's embedded base64 through the real engine.ts loader
//     (atob -> Blob stream -> DecompressionStream('gzip') -> WebAssembly.instantiate) and flies
//     takeoff -> circuit (2 laps) -> land through the exports.
// (2) Runs the page's TS bundle in a vm with a DOM / THREE stub (like the old JS harnesses),
//     pumps requestAnimationFrame, clicks buttons and presses keys, and checks for runtime errors.
// Usage: node test/smoke.mjs ../rc-heli-field.html
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import * as esbuild from 'esbuild';

const here = path.dirname(fileURLToPath(import.meta.url));
const file = process.argv[2] || path.join(here, '../../rc-heli-field.html');
const html = fs.readFileSync(file, 'utf8');
const fail = m => { console.error('FAIL', m); process.exit(1); };
const ok = m => console.log('ok  ', m);

const b64 = html.match(/window\.HELI_WASM = "([A-Za-z0-9+/=]+)"/)?.[1];
if (!b64) fail('no embedded wasm');
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
const bundle = scripts[scripts.length - 1];
ok(`page ${(Buffer.byteLength(html) / 1024).toFixed(0)} KB, wasm b64 ${(b64.length / 1024).toFixed(0)} KB, bundle ${(bundle.length / 1024).toFixed(0)} KB`);

// ---------- (1) wasm through engine.ts ----------
const out = path.join(here, '../build/engine.test.mjs');
await esbuild.build({ entryPoints: [path.join(here, '../src/engine.ts')], bundle: true, format: 'esm', outfile: out, logLevel: 'warning' });
const { Engine, Cmd } = await import(out + '?' + Date.now());
const e = await Engine.load(b64, 12345);
ok('wasm instantiated from embedded base64 via DecompressionStream');
const run = (sec, until) => {
  for (let i = 0; i < sec * 60; i++) {
    e.input(0, 0, 0, 0, null, null); e.step(4);
    const f = e.frame();
    if (until && until(f)) return f;
  }
  return e.frame();
};
let toasts = [];
const collect = f => { toasts.push(...f.toasts.map(t => t.t)); return f; };
e.cmd(Cmd.Takeoff);
let f = run(25, f => collect(f).mode === 1);
if (f.mode !== 1) fail('no hover hold');
const alt = f.p[1] - 0.29;
if (Math.abs(alt - 2.7) > 0.4) fail('hold switch altitude ' + alt);
f = run(5, f => { collect(f); });
ok(`takeoff -> hover hold, alt ${(f.p[1] - 0.29).toFixed(2)} m | cue "${f.hud.cue.replace(/<[^>]+>/g, '')}"`);
e.cmd(Cmd.Circuit);
let laps = 0;
f = run(120, f => { collect(f); laps = toasts.filter(t => /^Lap \d+ complete/.test(t)).length; return laps >= 2; });
if (laps < 2 || f.mode !== 2) fail('circuit laps ' + laps);
ok(`circuit: ${laps} laps, alt ${(f.p[1] - 0.29).toFixed(2)} m, HUD speed ${f.hud.spd}, battery ${f.hud.bat}`);
e.cmd(Cmd.Land);
f = run(60, f => { collect(f); return f.mode === -1 && toasts.some(t => /landing on the pad/.test(t)); });
const land = toasts.find(t => /landing on the pad/.test(t));
if (!land || f.crashed) fail('landing ' + land + ' crashed ' + f.crashed);
ok(`land: "${land}" | HUD landings ${f.hud.land} | flight time ${f.hud.time}`);
e.cmd(Cmd.Sensors);
f = run(1, f => { collect(f); });
if (!f.real || !f.hud.sens.includes('POS ERR')) fail('sensor panel');
ok('REAL sensors toggled, sensor panel text present');
if (!f.ekf || !/EKF <b[^>]*>(OK|RANGEFINDER|GPS REJECTED|GPS RESET|BARO ONLY)/.test(f.hud.sens) || !/GPS REJ/.test(f.hud.sens) || !(f.sig2 > 0)) fail('EKF panel ' + f.hud.sens);
e.cmd(Cmd.Estimator);
f = run(1, f => { collect(f); });
if (f.ekf || !/EST <b>BASIC/.test(f.hud.sens) || f.sig2 !== 0) fail('estimator toggle to BASIC');
e.cmd(Cmd.Estimator);
f = run(1, f => { collect(f); });
if (!f.ekf) fail('estimator toggle back to EKF');
ok(`EKF default, panel "${f.hud.sens.replace(/<[^>]+>/g, ' ').replace(/\s+/g, ' ').trim().slice(-70)}", 2-sigma ring ${f.sig2.toFixed(2)} m; Estimator command toggles BASIC / EKF`);

// stunt through the exports: picker list in the frame, pirouette to recovery
e.cmd(Cmd.Sensors); // back to IDEAL
e.cmd(Cmd.Takeoff);
f = run(25, f => collect(f).mode === 1);
e.cmd(Cmd.Stunts);
f = run(0.1, f => { collect(f); });
if (!f.picker || f.stunts.length !== 5 || f.stunts[0].n !== 'Pirouette') fail('stunt picker list ' + JSON.stringify(f.stunts));
e.cmd(Cmd.StuntFirst);
f = run(0.1, f => { collect(f); });
if (f.picker || f.mode !== 6) fail('stunt start: mode ' + f.mode);
f = run(60, f => { collect(f); return f.mode === 1 && f.stuntN > 0; });
if (!f.sres || f.crashed || f.p3d) fail('pirouette result ' + JSON.stringify(f.sres));
ok(`stunt picker (5 stunts), Pirouette via exports: score ${f.sres.score}, height lost ${f.sres.lost} m, "${toasts.filter(t => /Pirouette/.test(t)).join(' / ')}"`);

// ---------- (2) bundle in a DOM / THREE stub ----------
const require = createRequire(path.join(here, '../package.json'));
const THREE = require('three');
const H = {}, els = {}, errors = [];
const proxy = () => new Proxy(function () {}, { get: () => proxy(), apply: () => proxy(), set: () => true });
THREE.WebGLRenderer = function () { return { domElement: {}, setPixelRatio() {}, shadowMap: {}, capabilities: { getMaxAnisotropy: () => 1 }, setSize() {}, render() {}, setScissorTest() {}, setScissor() {}, setViewport() {} }; };
THREE.CanvasTexture = function () { return new THREE.Texture(); };
function el(id) {
  if (els[id]) return els[id];
  const e = {
    id, hidden: ['sheet', 'scrim', 'toast', 'pip', 'hp', 'sens', 'help', 'est', 'rRpm', 'picker', 'sres'].includes(id), style: {}, className: '', textContent: '', innerHTML: '', clientWidth: 150, clientHeight: 150,
    classList: { set: new Set(), add(c) { this.set.add(c); }, remove(c) { this.set.delete(c); }, toggle(c, on) { on ? this.set.add(c) : this.set.delete(c); } },
    setAttribute() {}, setPointerCapture() {}, querySelector: () => ({ style: {} }), appendChild() {}, focus() {}, contains: () => false,
    addEventListener(ev, fn) { H[id + ev] = fn; }, getBoundingClientRect: () => ({ top: 0, left: 0, width: 150, height: 150 }), getContext: () => proxy(),
    set onclick(fn) { H[id + 'click'] = fn; },
  };
  e.firstChild = { textContent: '' };
  return (els[id] = e);
}
const winL = {}, rafs = [];
let now = 0;
const ctx = {
  THREE, console: { ...console, error: (...a) => { errors.push(a.join(' ')); } },
  document: { getElementById: el, createElement: () => el('c' + Math.random()), querySelectorAll: () => [], activeElement: null },
  navigator: {}, matchMedia: () => ({ matches: false }), performance: { now: () => now }, requestAnimationFrame: fn => rafs.push(fn),
  atob, Blob, Response, DecompressionStream, TextDecoder, WebAssembly, Math, JSON, Object, Array, Promise, Uint8Array,
  setTimeout, queueMicrotask,
};
ctx.window = { ...ctx, matchMedia: ctx.matchMedia, addEventListener(ev, fn) { winL[ev] = fn; }, devicePixelRatio: 1, HELI_WASM: b64 };
ctx.globalThis = ctx.window;
vm.createContext(ctx);
try { vm.runInContext(bundle, ctx); } catch (err) { fail('bundle threw: ' + err.stack); }
for (let i = 0; i < 50 && !rafs.length; i++) await new Promise(r => setTimeout(r, 20));
if (!rafs.length) fail('bundle never started its frame loop: ' + errors.join(' | '));
const pump = sec => { for (let i = 0; i < sec * 60; i++) { now += 1000 / 60; const fn = rafs.shift(); if (!fn) fail('raf chain broke'); try { fn(now); } catch (err) { fail('frame threw: ' + err.stack); } } };
const key = k => { winL.keydown({ key: k, preventDefault() {}, repeat: false }); winL.keyup({ key: k }); };
H.bGoclick();
els.help.hidden = true;
pump(1);
if (!/ARM/.test(els.cue.innerHTML)) fail('cue before arming: ' + els.cue.innerHTML);
H.aTakeclick();
pump(12);
if (!els.aTake.classList.set.has('on')) fail('TAKEOFF button not lit');
if (els.bArmL.textContent !== 'ARMED') fail('arm label ' + els.bArmL.textContent);
ok(`DOM stub: takeoff via button, cue "${els.cue.innerHTML.replace(/<[^>]+>/g, '')}", ALT ${els.oAlt.textContent}, RPM ${els.oRpm.textContent}`);
key('k'); pump(8);
if (!els.aCirc.classList.set.has('on')) fail('circuit key K');
key('m'); pump(1);
if (els.hp.hidden || els.est.hidden || !/EKF OK|RANGEFINDER|EKF INIT|GPS/.test(els.hp.textContent) || !els.sens.hidden) fail('sensors key M: pill ' + els.hp.textContent);
H.hpclick(); pump(0.2);
if (els.sens.hidden || !/POS ERR/.test(els.sens.innerHTML) || !/CAMERA/.test(els.sens.innerHTML)) fail('health pill expands the sensor panel');
if (els.bEstV.textContent !== 'EKF') fail('estimator label ' + els.bEstV.textContent);
key('e'); pump(0.2);
if (els.bEstV.textContent !== 'BASIC' || !/EST <b>BASIC/.test(els.sens.innerHTML)) fail('estimator key E');
H.bEstclick(); pump(0.2);
if (els.bEstV.textContent !== 'EKF' || !/EKF <b/.test(els.sens.innerHTML)) fail('estimator button');
key('c'); pump(0.2);
if (els.bCamV.textContent !== 'CHASE') fail('camera key C: ' + els.bCamV.textContent);
H.bPipclick(); pump(0.2);
if (els.bPipV.textContent !== 'ALWAYS' || els.pip.hidden) fail('camera view setting');
H.bPipclick(); pump(0.2);
H.bSetclick(); pump(0.1);
if (els.sheet.hidden) fail('settings sheet opens');
key('Escape'); pump(0.1);
if (!els.sheet.hidden) fail('Esc closes the settings sheet');
H.stRpointerdown({ pointerId: 1, clientX: 75, clientY: 75, preventDefault() {} }); H.stRpointerup({ pointerId: 1 });
pump(0.5);
if (els.aCirc.classList.set.has('on')) fail('stick touch did not cancel the circuit');
key('q'); key('y'); key('v'); pump(0.3);
if (els.bAltV.textContent !== 'ON' || els.bYawV.textContent !== 'ON' || els.bTailV.textContent !== 'RATE' || !/ALT HOLD.*AUTO-YAW.*TAIL RATE/.test(els.pills.innerHTML)) fail('assist keys Q / Y / V ' + els.pills.innerHTML);
if (!/%/.test(els.vsiBar.style.height || '')) fail('climb bar not drawn');
H.bAltclick(); H.bYawclick(); H.bTailclick(); pump(0.2);
if (els.bAltV.textContent !== 'OFF' || els.bYawV.textContent !== 'OFF' || els.bTailV.textContent !== 'HOLD' || /ALT HOLD/.test(els.pills.innerHTML)) fail('assist buttons');
key('j'); pump(0.2);
if (els.picker.hidden || !/Pirouette/.test(els.picker.innerHTML)) fail('J opens the stunt picker');
key('Escape'); pump(0.2);
if (!els.picker.hidden) fail('Esc closes the picker');
key('g'); pump(25);
key('r'); pump(0.5);
if (els.bArmL.textContent !== 'ARM' || !els.toast.hidden) fail('reset');
if (errors.length) fail('console errors: ' + errors.join(' | '));
ok('DOM stub: circuit (K), sensors (M) with the health pill and sensor panel, estimator (E and button), view (C), camera view setting, settings sheet (open / Esc), stick takeover, assists (Q / Y / V and buttons, climb bar), stunt picker (J / Esc), land (G), reset (R): no runtime errors');
console.log('SMOKE PASS');
