// Headless Chromium test of the built page (puppeteer-core + @sparticuz/chromium, both from npm).
// cdnjs is not reachable from the build machine, so the three.js r128 request is answered with
// the identical npm build (three@0.128.0/build/three.min.js); everything else is the page as built.
// Usage: node test/browser.mjs ../rc-heli-field.html [screenshot-dir] [before.html]
//   With before.html, the UI coverage is also measured on that (older) page for comparison.
// Needs: npm i puppeteer-core@23 @sparticuz/chromium@130 (resolved from PW_DIR or here).
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const req = createRequire(path.join(process.env.PW_DIR || path.join(here, '..'), 'package.json'));
const puppeteer = req('puppeteer-core');
const chromium = req('@sparticuz/chromium');
const three = fs.readFileSync(createRequire(path.join(here, '../package.json')).resolve('three/build/three.min.js'));
const file = path.resolve(process.argv[2] || path.join(here, '../../rc-heli-field.html'));
const shots = process.argv[3];
const before = process.argv[4] && path.resolve(process.argv[4]);
const fail = m => { console.error('FAIL', m); process.exit(1); };
const ok = m => console.log('ok  ', m);
const sleep = ms => new Promise(r => setTimeout(r, ms));

const browser = await puppeteer.launch({
  executablePath: await chromium.executablePath(), headless: 'shell',
  args: [...chromium.args, '--enable-unsafe-swiftshader', '--allow-file-access-from-files'],
});
const errors = [];
const VIEWS = { portrait: [390, 844, true], landscape: [844, 390, true], desktop: [960, 600, false] };
async function open(w, h, touch = false, f = file) {
  const page = await browser.newPage();
  await page.setViewport({ width: w, height: h, deviceScaleFactor: 1, hasTouch: touch });
  {
    // The headless shell has no pointer device at all (hover:none, pointer:none): that is the desktop
    // case here; touch emulation makes it a phone (pointer:coarse).
    if (touch) {
      const cdp = await page.createCDPSession();
      await cdp.send('Emulation.setTouchEmulationEnabled', { enabled: true, maxTouchPoints: 5 });
    }
  }
  await page.setRequestInterception(true);
  page.on('request', r => {
    const u = r.url();
    if (u.startsWith('https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js')) r.respond({ status: 200, contentType: 'application/javascript', body: three });
    else if (u.startsWith('file:') || u.startsWith('data:')) r.continue();
    else r.respond({ status: 404, body: '' });           // fonts: fall back to system fonts
  });
  page.on('pageerror', e => errors.push(`pageerror (${w}x${h}): ` + e.message));
  page.on('console', m => { if (m.type() === 'error' && !/Failed to load resource|swiftshader|WebGL/i.test(m.text())) errors.push('console: ' + m.text()); });
  await page.goto('file://' + f, { waitUntil: 'load' });
  await page.waitForFunction(() => window.heli && window.heli.frame(), { timeout: 15000 });
  return page;
}

/** Fraction of the viewport covered by UI chrome: the union (sampled on a 2 px grid) of the
 *  bounding boxes of every painted UI element (own background, border, text or icon) on the
 *  page, excluding the 3D canvas. Invisible (hidden, display:none, opacity 0) and transparent
 *  layout containers don't count; overlapping boxes count once. */
const coverage = page => page.evaluate(() => {
  const W = innerWidth, H = innerHeight, S = 2, gw = Math.ceil(W / S), gh = Math.ceil(H / S);
  const grid = new Uint8Array(gw * gh);
  const vis = e => { for (let x = e; x && x !== document.body; x = x.parentElement) { const c = getComputedStyle(x); if (c.display === 'none' || c.visibility === 'hidden' || +c.opacity === 0 || x.hidden) return false; } return true; };
  const painted = e => {
    const c = getComputedStyle(e);
    const bg = (c.backgroundColor !== 'rgba(0, 0, 0, 0)' && c.backgroundColor !== 'transparent') || c.backgroundImage !== 'none';
    const bd = ['Top', 'Right', 'Bottom', 'Left'].some(s => parseFloat(c['border' + s + 'Width']) > 0 && c['border' + s + 'Style'] !== 'none');
    const txt = [...e.childNodes].some(n => n.nodeType === 3 && n.textContent.trim());
    return bg || bd || txt || e.tagName === 'svg';
  };
  let n = 0;
  for (const e of document.querySelectorAll('#app *')) {
    if (e.closest('#gl') || (e.closest('svg') && e.tagName !== 'svg') || !painted(e) || !vis(e)) continue;
    const r = e.getBoundingClientRect();
    if (r.width < 1 || r.height < 1) continue;
    n++;
    for (let y = Math.max(0, Math.floor(r.top / S)); y < Math.min(gh, Math.ceil(r.bottom / S)); y++)
      for (let x = Math.max(0, Math.floor(r.left / S)); x < Math.min(gw, Math.ceil(r.right / S)); x++) grid[y * gw + x] = 1;
  }
  let c = 0; for (const v of grid) c += v;
  return { frac: c / grid.length, n };
});

/** Coverage of one page build at each viewport in three states: idle on the pad, hover hold
 *  (IDEAL), hover hold with REAL sensors (key M); each measured 3.6 s after the action so the
 *  toasts have faded. */
async function coverageRun(f, name) {
  const out = {};
  for (const [v, [w, h, touch]] of Object.entries(VIEWS)) {
    const p = await open(w, h, touch, f);
    // DOM clicks: this pass measures the layout (the old page's buttons can be under other chrome)
    await p.$eval('#bGo', e => e.click());
    await sleep(3600);
    const idle = await coverage(p);
    await p.$eval('#aTake', e => e.click());
    await p.waitForFunction(() => window.heli.frame().mode === 1, { timeout: 240000 }).catch(() => fail(`${name} ${v}: no hover hold`));
    await sleep(3600);
    const hover = await coverage(p);
    await p.keyboard.press('m');
    await sleep(3600);
    const real = await coverage(p);
    out[v] = { idle: idle.frac, hover: hover.frac, real: real.frac };
    if (shots && name === 'before') await p.screenshot({ path: path.join(shots, `before-${v}.png`) });
    await p.close();
  }
  return out;
}

const pct = x => (100 * x).toFixed(1) + '%';
let covBefore = null;
if (before) {
  covBefore = await coverageRun(before, 'before');
  for (const [v, c] of Object.entries(covBefore)) ok(`coverage BEFORE ${v}: idle ${pct(c.idle)}, hover ${pct(c.hover)}, REAL hover ${pct(c.real)}`);
}
const covAfter = process.env.QUICK ? {} : await coverageRun(file, 'after');   // QUICK=1 skips the coverage pass
for (const [v, c] of Object.entries(covAfter)) {
  ok(`coverage AFTER  ${v}: idle ${pct(c.idle)}, hover ${pct(c.hover)}, REAL hover ${pct(c.real)}` +
    (covBefore ? ` (before ${pct(covBefore[v].idle)} / ${pct(covBefore[v].hover)} / ${pct(covBefore[v].real)})` : ''));
  if (covBefore && !(c.hover < covBefore[v].hover && c.real < covBefore[v].real)) fail(`coverage did not drop at ${v}`);
}

// ---------------------------------------------------------------- desktop 960 x 600
const page = await open(960, 600);
ok('page loaded in Chromium, wasm instantiated from embedded base64 (DecompressionStream), frame loop running');
{ const t0 = (await page.evaluate(() => window.heli.frame().simT)); await sleep(2000); const t1 = await page.evaluate(() => window.heli.frame().simT);
  ok(`sim runs at ${((t1 - t0) / 2).toFixed(2)}x real time under software WebGL`); }
const fr = () => page.evaluate(() => window.heli.frame());
/** wait for a page state (frames come slowly under software WebGL); the asserts after it decide */
const until = (fn, ...a) => page.waitForFunction(fn, { timeout: 15000 }, ...a).catch(() => {});
const text = (id, p = page) => p.$eval('#' + id, e => e.textContent);
const hidden = (id, p = page) => p.$eval('#' + id, e => e.hidden || getComputedStyle(e).display === 'none');
await page.click('#bGo');
// desktop keeps tooltips and key hints
const tips = await page.evaluate(() => ({ title: document.getElementById('aLand').title, k: getComputedStyle(document.querySelector('#aLand i.k')).display }));
if (!tips.title || tips.k === 'none') fail('desktop tooltips / key hints ' + JSON.stringify(tips));
await page.click('#aTake');
await page.waitForFunction(() => window.heli.frame().mode === 1, { timeout: 30000 });
await sleep(1500);
const bar = await page.evaluate(() => [...document.querySelectorAll('#actbar button')].map(b => b.id));
if (bar.join() !== 'bArm,bHold,aTake,aCirc,aLand,aAuto,aStunt,bSet') fail('action bar ' + bar);
if (!(await page.$eval('#aTake', e => e.classList.contains('on')))) fail('TAKEOFF not highlighted');
const fits = await page.evaluate(() => [...document.querySelectorAll('#actbar button, .readouts .r')].every(b => { const r = b.getBoundingClientRect(); return r.width === 0 || (r.right <= innerWidth && b.scrollWidth <= b.clientWidth + 1); }));
if (!fits) fail('desktop: action bar / readouts overflow');
const ro = await page.$$eval('.readouts .r', rs => rs.filter(r => !r.hidden).map(r => r.querySelector('b').textContent));
if (ro.join() !== 'ALT,V/S,SPEED,BATTERY') fail('core readouts ' + ro);
ok(`TAKEOFF: hover hold, action bar ${bar.length} buttons (mode lit), readouts ${ro.join(' / ')} (headspeed hidden at ${(await fr()).rpm.toFixed(2)}), tooltips + key hints on desktop, cue "${await text('cue')}"`);
if (shots) await page.screenshot({ path: path.join(shots, 'desktop.png') });
// toasts: only events; none duplicates the status line
await page.keyboard.press('k');
await page.waitForFunction(() => window.heli.frame().mode === 2, { timeout: 5000 });
await sleep(400);
if (!(await hidden('toast'))) fail('non-event toast shown: ' + (await text('toast')));
ok('K: circuit; the "Autopilot: circuit" toast is suppressed (the status line says it)');
// REAL: EST badge and the health pill, expanding to the sensor panel
await page.keyboard.press('m');
await until(() => !document.getElementById('hp').hidden);
if (await hidden('est') || await hidden('hp') || !(await hidden('sens'))) fail('REAL: EST badge / pill / collapsed panel');
const pill = await text('hp');
if (!/EKF OK|RANGEFINDER|GPS|CAMERA|INIT/.test(pill)) fail('pill ' + pill);
await page.click('#hp'); await until(() => !document.getElementById('sens').hidden && /POS ERR/.test(document.getElementById('sens').textContent));
const sens = await text('sens');
if (await hidden('sens') || !/POS ERR/.test(sens) || !/CAMERA (SEARCHING|TRACKING|LOST)/.test(sens) || (await page.$$('#sens .tr')).length !== 4) fail('sensor panel ' + sens);
await page.click('#hp'); await until(() => document.getElementById('sens').hidden);
if (!(await hidden('sens'))) fail('pill collapses the panel');
ok(`M: REAL, one EST badge, health pill "${pill}" expands to the sensor panel ("${sens.slice(0, 50)}...", camera status) and collapses`);

// settings sheet: opens, every setting toggles, closes on outside tap and Esc
await page.click('#bSet'); await sleep(250);
/** wait for the sheet's slide-in to end (the page renders slowly under software WebGL) */
const settle = (p, id) => p.waitForFunction(i => document.getElementById(i).getAnimations().length === 0, { timeout: 15000, polling: 100 }, id);
await settle(page, 'sheet');
if (await hidden('sheet')) fail('settings sheet did not open');
const side = await page.$eval('#sheet', e => { const r = e.getBoundingClientRect(); return { right: r.right, w: r.width, h: r.height, appScroll: document.getElementById('app').scrollLeft }; });
if (!(side.right >= 959 && side.w <= 330 && side.h > 500)) fail('desktop side panel ' + JSON.stringify(side));
if (shots) await page.screenshot({ path: path.join(shots, 'settings-open.png') });
const rows = ['bLvl', 'bProt', 'bWind', 'bAlt', 'bYaw', 'bTail', 'bSens', 'bEst', 'bCam', 'bPip'];
const changed = [];
for (const id of rows) {
  const v0 = await text(id + 'V');
  await page.click('#' + id);
  await page.waitForFunction((i, old) => document.getElementById(i + 'V').textContent !== old, { timeout: 5000 }, id, v0).catch(() => {});
  const v1 = await text(id + 'V');
  if (v0 === v1) fail(`setting ${id} did not toggle (${v0})`);
  changed.push(`${id.slice(1)} ${v0}->${v1}`);
  // back to where it was (wind and view cycle through 3)
  for (let k = 0; k < (id === 'bWind' || id === 'bCam' ? 2 : 1); k++) {
    const v = await text(id + 'V');
    await page.click('#' + id);
    await page.waitForFunction((i, old) => document.getElementById(i + 'V').textContent !== old, { timeout: 5000 }, id, v).catch(() => {});
  }
  if ((await text(id + 'V')) !== v0) fail(`setting ${id} did not come back to ${v0} (${await text(id + 'V')})`);
}
// pitch curve refuses in the air (toast), help opens the card: both still wired
await page.click('#bPitch'); await until(() => /Land first/.test(document.getElementById('toast').textContent));
if ((await text('bPitchV')) !== 'NORMAL' || !/Land first/.test(await text('toast'))) fail('pitch in the air should be refused with a toast');
changed.push('Pitch refused in the air');
await page.mouse.click(200, 300); await sleep(250);
if (!(await hidden('sheet'))) fail('outside tap should close the sheet');
await page.click('#bSet'); await sleep(200); await page.keyboard.press('Escape'); await sleep(200);
if (!(await hidden('sheet'))) fail('Esc should close the sheet');
await page.click('#bSet'); await sleep(200); await page.click('#bHelp'); await sleep(200);
if (await hidden('help') || !(await hidden('sheet'))) fail('HELP from the sheet');
await page.click('#bGo'); await sleep(150);
ok(`settings sheet: side panel ${side.w.toFixed(0)} px, every setting toggles and back (${changed.join(', ')}), closes on outside tap and Esc, HELP opens the card`);

// REAL LAND: picture-in-picture with the detection box and lock
await page.keyboard.press('g');
await page.waitForFunction(() => window.heli.frame().mode === 3, { timeout: 5000 });
await page.waitForFunction(() => { const f = window.heli.frame(); return f.pip && f.camLock && f.camDet.length === 4; }, { timeout: 120000 });
await sleep(400);
const pip = await page.evaluate(() => {
  const p = document.getElementById('pip').getBoundingClientRect(), b = document.getElementById('pipBox').getBoundingClientRect();
  const sticks = [...document.querySelectorAll('.stick')].map(s => s.getBoundingClientRect()), cue = document.getElementById('cue').getBoundingClientRect();
  const hit = (a, c) => a.left < c.right && c.left < a.right && a.top < c.bottom && c.top < a.bottom;
  return { w: p.width, h: p.height, box: b.width > 1 && b.left >= p.left - 2 && b.right <= p.right + 2, lock: document.getElementById('pip').classList.contains('lock'),
    label: document.getElementById('pipL').textContent, clear: !sticks.some(s => hit(p, s)) && !hit(p, cue) };
});
if (!(pip.w === 120 && pip.h === 90 && pip.box && pip.lock && pip.clear)) fail('PiP ' + JSON.stringify(pip));
if (shots) await page.screenshot({ path: path.join(shots, 'real-landing-pip.png') });
ok(`REAL LAND: PiP ${pip.w}x${pip.h} px with the detection box, "${pip.label}", clear of the sticks and the status line; pill "${await text('hp')}"`);
await page.waitForFunction(() => /landing on the pad/.test(document.getElementById('toast').textContent) && !document.getElementById('toast').hidden, { timeout: 150000 });
const tst = await page.evaluate(() => { const t = document.getElementById('toast').getBoundingClientRect(); return { top: t.top, H: innerHeight }; });
if (!(tst.top > tst.H * 0.6)) fail('landing toast over the centre of the view ' + JSON.stringify(tst));
ok(`G: "${await text('toast')}" shown low (top at ${(100 * tst.top / tst.H).toFixed(0)}% of the height); landings ${await text('oLand')}`);
// the toast timer runs on frame time (3 s; this page renders at a few fps under software WebGL)
await page.waitForFunction(() => document.getElementById('toast').hidden, { timeout: 60000 }).catch(() => fail('toast should fade'));

// stunts: open the picker with the button, check it, fly a Pirouette from it to recovery
await page.keyboard.press('m'); await sleep(200);   // back to IDEAL
await page.click('#aTake');
await page.waitForFunction(() => window.heli.frame().mode === 1, { timeout: 60000 });
await page.click('#aStunt');
await page.waitForFunction(() => !document.getElementById('picker').hidden, { timeout: 5000 });
const items = await page.$$eval('#picker button', bs => bs.map(b => b.textContent));
if (items.length !== 5 || !/Pirouette/.test(items[0]) || !/Inverted hover/.test(items[4])) fail('stunt picker items ' + items);
await page.click('#picker button[data-id="0"]');
await page.waitForFunction(() => window.heli.frame().mode === 6, { timeout: 5000 });
if (!(await page.$eval('#picker', e => e.hidden))) fail('picker should close after picking');
await page.waitForFunction(() => /Stunt: <em>pirouette<\/em>/.test(document.getElementById('cue').innerHTML), { timeout: 60000 });
const cueMan = await text('cue');
await page.waitForFunction(() => { const f = window.heli.frame(); return f.mode === 1 && f.stuntN > 0; }, { timeout: 60000 });
await sleep(200);
const res = await text('toast');
let f = await fr();
if (!f.sres || f.crashed || f.p3d || !/score/.test(res)) fail('pirouette result ' + JSON.stringify(f.sres) + ' toast ' + res);
await page.click('#bSet'); await sleep(200);
const sres = await text('sres'); const nst = await text('oStunts');
if (await hidden('sres') || nst !== '1') fail('stunt result in the settings sheet');
await page.keyboard.press('Escape');
ok(`STUNTS button -> picker (${items.length}) -> Pirouette: cue "${cueMan}", toast "${res}", result in the session stats "${sres}"`);
// manual: reset, arm, throttle up with W
await page.keyboard.press('r');
await page.keyboard.press(' ');
await page.keyboard.down('w'); await sleep(1100); await page.keyboard.up('w');
await sleep(3000);
f = await fr();
if (!f.armed) fail('manual arm');
// assists: ALT HOLD (Q) and AUTO-YAW (settings), pills by the sticks, climb bar beside the left stick
await page.keyboard.press('q');
await page.click('#bSet'); await sleep(150); await page.click('#bYaw'); await page.keyboard.press('Escape');
await sleep(300);
f = await fr();
await until(() => document.querySelectorAll('#pills span').length === 2);
const pills = await page.$$eval('#pills span', s => s.map(x => x.textContent));
if (!f.altHold || !f.autoYaw || pills.join() !== 'ALT HOLD,AUTO-YAW') fail('assist pills ' + pills);
await page.waitForFunction(() => window.heli.frame().rpm > 0.97, { timeout: 60000 });
await page.keyboard.down('w');
await page.waitForFunction(() => window.heli.frame().ahOn, { timeout: 30000 });
await page.waitForFunction(() => window.heli.frame().vsi > 0.3, { timeout: 10000 });
await page.keyboard.up('w');
const vb = await page.evaluate(() => {
  const v = document.getElementById('vsi').getBoundingClientRect(), b = document.getElementById('vsiBar').getBoundingClientRect(), s = document.getElementById('stL').getBoundingClientRect();
  return { vh: v.height, bh: b.height, above: b.bottom <= v.top + v.height / 2 + 1, beside: v.left >= s.right, ah: document.getElementById('vsi').classList.contains('ah') };
});
if (!(vb.vh > 50 && vb.bh > 1 && vb.above && vb.beside && vb.ah)) fail('climb bar ' + JSON.stringify(vb));
ok(`assists: Q and the settings row -> pills "${pills.join(' / ')}" by the sticks, W lifts off into ALT HOLD, climb bar ${vb.bh.toFixed(0)} of ${vb.vh.toFixed(0)} px above zero beside the left stick`);
await page.close();

// ---------------------------------------------------------------- phone portrait 390 x 844 (touch)
const lay = async p => p.evaluate(() => {
  const r = e => e.getBoundingClientRect();
  const btn = [...document.querySelectorAll('#actbar button')].map(r);
  const st = [...document.querySelectorAll('.stick')].map(r);
  return {
    sw: document.documentElement.scrollWidth, w: innerWidth, h: innerHeight,
    btnIn: btn.every(b => b.left >= 0 && b.right <= innerWidth && b.bottom <= innerHeight),
    rows: new Set(btn.map(b => Math.round(b.top))).size, cols: new Set(btn.map(b => Math.round(b.left))).size,
    btnH: Math.min(...btn.map(b => b.height)), btnW: Math.min(...btn.map(b => b.width)),
    sticks: st.every(s => s.right <= innerWidth && s.bottom <= innerHeight) && st[0].left < innerWidth / 3 && st[1].right > innerWidth * 2 / 3 && st.every(s => s.bottom > innerHeight * 0.7),
    vsi: (() => { const v = r(document.getElementById('vsi')), s = r(document.getElementById('stL')); return v.width > 4 && v.left >= s.right; })(),
  };
});
const ph = await open(390, 844, true);
await ph.click('#bGo');
const touchRules = await ph.evaluate(() => ({
  titles: [...document.querySelectorAll('[title]')].filter(e => e.id !== 'est').length,
  k: [...document.querySelectorAll('i.k')].every(e => getComputedStyle(e).display === 'none'),
}));
if (touchRules.titles || !touchRules.k) fail('touch: tooltips / key hints ' + JSON.stringify(touchRules));
let L = await lay(ph);
if (L.sw > L.w || !L.btnIn || L.rows !== 1 || L.btnH < 40 || !L.sticks || !L.vsi) fail('phone portrait layout ' + JSON.stringify(L));
await ph.click('#aTake');
await ph.waitForFunction(() => window.heli.frame().mode === 1, { timeout: 30000 });
await sleep(3500);
if (shots) await ph.screenshot({ path: path.join(shots, 'phone-portrait.png') });
// stunt picker fits
await ph.click('#aStunt'); await sleep(250);
const pk = await ph.evaluate(() => { const r = document.getElementById('picker').getBoundingClientRect(); return r.right <= innerWidth && r.width > 200 && document.querySelectorAll('#picker button').length === 5; });
if (!pk) fail('phone picker');
if (shots) await ph.screenshot({ path: path.join(shots, 'phone-picker.png') });
await ph.click('#aStunt'); await sleep(150);
// settings: bottom sheet
await ph.tap('#bSet'); await sleep(300); await settle(ph, 'sheet');
const sh = await ph.$eval('#sheet', e => { const r = e.getBoundingClientRect(); return { bottom: r.bottom, top: r.top, w: r.width, H: innerHeight, W: innerWidth }; });
if (!(Math.abs(sh.bottom - sh.H) < 2 && sh.w >= sh.W - 2 && sh.top > sh.H * 0.2)) fail('phone bottom sheet ' + JSON.stringify(sh));
if (shots) await ph.screenshot({ path: path.join(shots, 'phone-settings-open.png') });
await ph.touchscreen.tap(195, sh.top / 2); await sleep(400);   // above the sheet
if (!(await hidden('sheet', ph))) fail('phone: outside tap closes the sheet');
ok(`phone portrait 390x844 (touch): no tooltips or key hints, no horizontal scroll, ${L.cols} action buttons in one row (${L.btnW.toFixed(0)}x${L.btnH.toFixed(0)} px), sticks in the bottom corners, climb bar, picker fits, bottom sheet (top at ${(100 * sh.top / sh.H).toFixed(0)}%) closes on outside tap`);
await ph.close();

// ---------------------------------------------------------------- phone landscape 844 x 390 (touch)
const ls = await open(844, 390, true);
await ls.click('#bGo');
L = await lay(ls);
if (L.sw > L.w || !L.btnIn || L.cols !== 1 || L.rows !== 8 || !L.sticks) fail('phone landscape layout ' + JSON.stringify(L));
const colW = await ls.$eval('#actbar', e => e.getBoundingClientRect().width);
await ls.click('#aTake');
await ls.waitForFunction(() => window.heli.frame().mode === 1, { timeout: 30000 });
await sleep(3500);
if (shots) await ls.screenshot({ path: path.join(shots, 'phone-landscape.png') });
ok(`phone landscape 844x390 (touch): action bar is a ${colW.toFixed(0)} px column (${L.rows} buttons, ${L.btnH.toFixed(0)} px tall), sticks in the bottom corners, no horizontal scroll`);
await ls.close();

await browser.close();
if (errors.length) fail(errors.join('\n'));
console.log('BROWSER PASS');
