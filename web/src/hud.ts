// HUD: transmitter-style readouts, cue line, sensor panel, toasts and button states.
// All text and colour classes come from the sim frame; this only writes them to the DOM.

import type { Frame, Toast } from './engine';

const $ = (id: string) => document.getElementById(id)!;
const label = (id: string, text: string) => { const b = $(id); if (b.firstChild && b.firstChild.textContent !== text) b.firstChild.textContent = text; };
const setText = (id: string, text: string) => { const e = $(id); if (e.textContent !== text) e.textContent = text; };
const setHtml = (id: string, html: string) => { const e = $(id); if (e.innerHTML !== html) e.innerHTML = html; };

// autopilot mode codes from the sim
const M = { takeoff: 0, hold: 1, circuit: 2, land: 3, rescue: 4, autorot: 5, stunt: 6 };

export class Hud {
  private toastEl = $('toast');
  private toastT = 0;
  private lastReal: boolean | null = null;

  private pickKey = '';
  /** index of the keyboard-highlighted stunt in the picker */
  sel = 0;
  /** stunt picker: rendered from the list in the frame; rebuilt only when it changes */
  private picker(f: Frame) {
    const el = $('picker') as HTMLElement;
    el.hidden = !f.picker;
    if (!f.picker) return;
    const key = JSON.stringify([f.stunts, f.stuntBlocked, this.sel]);
    if (key === this.pickKey) return;
    this.pickKey = key;
    el.innerHTML = f.stunts.map((s, i) =>
      `<button role="menuitem" data-id="${s.id}" class="${i === this.sel ? 'sel' : ''}"${f.stuntBlocked ? ' disabled' : ''}>` +
      `<kbd>${s.key}</kbd><span>${s.n}<small>${s.d}</small></span><em>${s.best >= 0 ? 'best ' + s.best : ''}</em></button>`).join('') +
      `<p>${f.stuntBlocked ? 'Stunts need at least 25% battery.' : '1-5 or arrows + Enter to fly · Esc closes'}</p>`;
  }

  showToast(t: Toast) {
    this.toastEl.innerHTML = `${t.t}<small>${t.s}</small>`;
    this.toastEl.className = t.k || '';
    this.toastEl.style.opacity = '1';
    this.toastT = 3.2;
  }

  update(f: Frame, dt: number) {
    if (f.toastClear) { this.toastEl.style.opacity = '0'; this.toastT = 0; }
    for (const t of f.toasts) this.showToast(t);
    if (this.toastT > 0) { this.toastT -= dt; if (this.toastT <= 0) this.toastEl.style.opacity = '0'; }

    const h = f.hud;
    setText('oThr', h.thr);
    setText('oRpm', h.rpm); $('oRpm').className = h.rpmC;
    setText('oAlt', h.alt);
    setText('oVs', h.vs); $('oVs').className = h.vsC;
    setText('oSpd', h.spd);
    setText('oBat', h.bat); $('oBat').className = h.batC;
    setText('oLand', h.land);
    setText('oTime', h.time);
    setText('oResc', h.resc);
    setHtml('cue', h.cue);
    if (f.real) setHtml('sens', h.sens);

    // buttons
    const arm = $('bArm'); arm.classList.toggle('armed', f.armed); label('bArm', f.armed ? 'ARMED' : 'DISARMED');
    const hold = $('bHold'); hold.classList.toggle('on', f.hold); label('bHold', f.hold ? 'THR HOLD: ON' : 'THR HOLD');
    const prot = $('bProt'); label('bProt', f.protect ? 'PROTECT: ON' : 'PROTECT: OFF'); prot.classList.toggle('off', !f.protect);
    prot.classList.toggle('active', f.mode === M.rescue);
    label('bSens', f.real ? 'SENSORS: REAL' : 'SENSORS: IDEAL');
    label('bEst', f.ekf ? 'ESTIMATOR: EKF' : 'ESTIMATOR: BASIC'); $('bEst').classList.toggle('off', !f.real);
    label('bLvl', f.level ? 'GYRO: SELF-LEVEL' : 'GYRO: RATE');
    label('bPitch', f.p3d ? 'PITCH: 3D' : 'PITCH: NORMAL');
    label('bWind', 'WIND: ' + f.windName);
    label('bCam', 'VIEW: ' + ({ pilot: 'PILOT', chase: 'CHASE', pad: 'PAD B' } as const)[f.cam]);
    $('aTake').classList.toggle('on', f.mode === M.takeoff || f.mode === M.hold);
    $('aCirc').classList.toggle('on', f.mode === M.circuit);
    $('aLand').classList.toggle('on', f.mode === M.land);
    $('aAuto').classList.toggle('on', f.mode === M.autorot);
    const st = $('aStunt'); st.classList.toggle('on', f.mode === M.stunt || f.picker); st.setAttribute('aria-expanded', String(f.picker));
    this.picker(f);
    const r = f.sres, sr = $('sres') as HTMLElement;
    sr.hidden = !r;
    if (r) setHtml('sres', `<span>LAST STUNT <b>${r.n}</b></span><span>SCORE <b class="sc">${r.score}</b></span><span>HEIGHT LOST <b>${r.lost} m</b></span>` +
      `<span>MIN ALT <b>${r.minAlt} m</b></span><span>MAX DEV <b>${r.dev} m</b></span><span>HDG <b>${r.hdg}°</b></span><span>RECOVERY <b>${r.rec} s</b></span>`);
    if (this.lastReal !== f.real) {
      this.lastReal = f.real;
      ($('sens') as HTMLElement).hidden = !f.real;
      for (const [id, n] of [['lAlt', 'ALT'], ['lVs', 'V/S'], ['lSpd', 'SPEED']]) $(id).textContent = f.real ? n + ' (EST)' : n;
    }
  }
}
