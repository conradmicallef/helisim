// Headless Chromium test of the built page (puppeteer-core + @sparticuz/chromium, both from npm).
// cdnjs is not reachable from the build machine, so the three.js r128 request is answered with
// the identical npm build (three@0.128.0/build/three.min.js); everything else is the page as built.
// Usage: node test/browser.mjs ../rc-heli-field.html [screenshot-dir]
// Needs: npm i puppeteer-core@23 @sparticuz/chromium@130 (resolved from NODE_PATH or here).
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
const fail = m => { console.error('FAIL', m); process.exit(1); };
const ok = m => console.log('ok  ', m);

const browser = await puppeteer.launch({
  executablePath: await chromium.executablePath(), headless: 'shell',
  args: [...chromium.args, '--enable-unsafe-swiftshader', '--allow-file-access-from-files'],
});
const errors = [];
async function open(w, h) {
  const page = await browser.newPage();
  await page.setViewport({ width: w, height: h, deviceScaleFactor: 1 });
  await page.setRequestInterception(true);
  page.on('request', r => {
    const u = r.url();
    if (u.startsWith('https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js')) r.respond({ status: 200, contentType: 'application/javascript', body: three });
    else if (u.startsWith('file:') || u.startsWith('data:')) r.continue();
    else r.respond({ status: 404, body: '' });           // fonts: fall back to system fonts
  });
  page.on('pageerror', e => errors.push('pageerror: ' + e.message));
  page.on('console', m => { if (m.type() === 'error' && !/Failed to load resource|swiftshader|WebGL/i.test(m.text())) errors.push('console: ' + m.text()); });
  await page.goto('file://' + file, { waitUntil: 'load' });
  await page.waitForFunction(() => window.heli && window.heli.frame(), { timeout: 15000 });
  return page;
}

const page = await open(960, 600);
ok('page loaded in Chromium, wasm instantiated from embedded base64 (DecompressionStream), frame loop running');
{ const t0 = (await page.evaluate(() => window.heli.frame().simT)); await new Promise(r => setTimeout(r, 2000)); const t1 = await page.evaluate(() => window.heli.frame().simT);
  ok(`sim runs at ${((t1 - t0) / 2).toFixed(2)}x real time under software WebGL`); }
const fr = () => page.evaluate(() => window.heli.frame());
const text = id => page.$eval('#' + id, e => e.textContent);
await page.click('#bGo');
await page.click('#aTake');
await page.waitForFunction(() => window.heli.frame().mode === 1, { timeout: 30000 });
await new Promise(r => setTimeout(r, 1500));
let f = await fr();
ok(`TAKEOFF button: hover hold, alt ${text && (await text('oAlt'))}, rpm ${await text('oRpm')}, cue "${(await text('cue'))}"`);
if (shots) await page.screenshot({ path: path.join(shots, 'desktop-hover.png') });
await page.keyboard.press('k');
await page.waitForFunction(() => window.heli.frame().mode === 2, { timeout: 5000 });
await page.waitForFunction(() => /circuit, lap <em>1/.test(document.getElementById('cue').innerHTML), { timeout: 90000 });
await new Promise(r => setTimeout(r, 3000));
f = await fr();
ok(`K: circuit joined (lap 1), speed ${await text('oSpd')}, toast "${await page.$eval('#toast', e => e.textContent)}"`);
await page.keyboard.press('m');
await new Promise(r => setTimeout(r, 500));
const sens = await page.$eval('#sens', e => !e.hidden && e.textContent);
if (!sens || !/POS ERR/.test(sens)) fail('sensor panel');
ok(`M: REAL sensors, panel "${sens.slice(0, 60)}..."`);
if (!/EKF (OK|RANGEFINDER|GPS REJECTED|GPS RESET|BARO ONLY)/.test(sens) || !/GPS REJ/.test(sens) || (await page.$$('#sens .tr')).length !== 4) fail('EKF panel ' + sens);
if ((await text('bEst')) !== 'ESTIMATOR: EKFE') fail('estimator label ' + (await text('bEst')));
await page.keyboard.press('e'); await new Promise(r => setTimeout(r, 300));
if ((await text('bEst')) !== 'ESTIMATOR: BASICE' || !/EST BASIC/.test(await text('sens'))) fail('estimator key E');
await page.click('#bEst'); await new Promise(r => setTimeout(r, 300));
f = await fr();
if (!f.ekf || !(f.sig2 > 0)) fail('estimator button back to EKF');
ok(`E / ESTIMATOR button: EKF -> BASIC -> EKF, panel "${(await text('sens')).replace(/^.*?EKF/, 'EKF')}", 2-sigma ring ${f.sig2.toFixed(2)} m`);
if (shots) await page.screenshot({ path: path.join(shots, 'desktop-circuit-real.png') });
await page.keyboard.press('g');
await page.waitForFunction(() => /landing on Pad A/.test(document.getElementById('toast').textContent), { timeout: 150000 });
ok(`G: "${await page.$eval('#toast', e => e.textContent)}" | landings ${await text('oLand')}`);
// stunts: open the picker with the button, check it, fly a Pirouette from it to recovery
await page.keyboard.press('m'); await new Promise(r => setTimeout(r, 200));   // back to IDEAL
await page.click('#aTake');
await page.waitForFunction(() => window.heli.frame().mode === 1, { timeout: 60000 });
await page.click('#aStunt');
await page.waitForFunction(() => !document.getElementById('picker').hidden, { timeout: 5000 });
const items = await page.$$eval('#picker button', bs => bs.map(b => b.textContent));
if (items.length !== 5 || !/Pirouette/.test(items[0]) || !/Inverted hover/.test(items[4])) fail('stunt picker items ' + items);
if (shots) await page.screenshot({ path: path.join(shots, 'desktop-picker.png') });
await page.click('#picker button[data-id="0"]');
await page.waitForFunction(() => window.heli.frame().mode === 6, { timeout: 5000 });
if (!(await page.$eval('#picker', e => e.hidden))) fail('picker should close after picking');
await page.waitForFunction(() => /Stunt: <em>pirouette<\/em>/.test(document.getElementById('cue').innerHTML), { timeout: 60000 });
const cueMan = await text('cue');
await page.waitForFunction(() => { const f = window.heli.frame(); return f.mode === 1 && f.stuntN > 0; }, { timeout: 60000 });
f = await fr();
if (!f.sres || f.crashed || f.p3d || (await page.$eval('#sres', e => e.hidden))) fail('pirouette result ' + JSON.stringify(f.sres));
ok(`STUNTS button -> picker (${items.length} stunts) -> Pirouette: cue "${cueMan}", result panel "${await text('sres')}", pitch restored`);
if (shots) await page.screenshot({ path: path.join(shots, 'desktop-stunt.png') });
// manual: reset, arm, throttle up with W, hover, touch a stick
await page.keyboard.press('r');
await page.keyboard.press(' ');
await page.keyboard.down('w'); await new Promise(r => setTimeout(r, 1100)); await page.keyboard.up('w');
await new Promise(r => setTimeout(r, 3000));
f = await fr();
ok(`manual: Space arms (${await text('bArm')}), W throttle ${await text('oThr')}, rpm ${await text('oRpm')}`);
if (!f.armed) fail('manual arm');

// phone width
const ph = await open(390, 844);
await ph.click('#bGo');
await ph.keyboard.press('j');
await ph.waitForFunction(() => !document.getElementById('picker').hidden, { timeout: 5000 });
const layout = await ph.evaluate(() => ({
  sw: document.documentElement.scrollWidth, w: innerWidth,
  sticks: [...document.querySelectorAll('.stick')].map(s => s.getBoundingClientRect().right <= innerWidth),
  cols: getComputedStyle(document.querySelector('.readouts')).gridTemplateColumns.split(' ').length,
  picker: document.getElementById('picker').getBoundingClientRect().right <= innerWidth,
}));
if (layout.sw > layout.w || !layout.sticks.every(Boolean) || layout.cols !== 3 || !layout.picker) fail('phone layout ' + JSON.stringify(layout));
if (shots) await ph.screenshot({ path: path.join(shots, 'phone-picker.png') });
await ph.keyboard.press('Escape');
await ph.click('#aTake');
await ph.waitForFunction(() => window.heli.frame().mode === 1, { timeout: 30000 });
ok(`phone 390 px: no horizontal scroll, sticks in view, readouts ${layout.cols} columns, stunt picker fits, takeoff works`);
if (shots) await ph.screenshot({ path: path.join(shots, 'phone-hover.png') });
await browser.close();
if (errors.length) fail(errors.join('\n'));
console.log('BROWSER PASS');
