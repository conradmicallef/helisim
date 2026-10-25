// HUD: readouts, action bar and settings states, status line, estimator pill and sensor panel,
// assist pills, event toasts, the camera PiP overlay. All text and colour classes come from the
// sim frame; this only writes them to the DOM.

import type { Frame, ShowCard, Toast } from './engine';

const $ = (id: string) => document.getElementById(id)!;
const setText = (id: string, text: string) => { const e = $(id); if (e.textContent !== text) e.textContent = text; };
const setHtml = (id: string, html: string) => { const e = $(id); if (e.innerHTML !== html) e.innerHTML = html; };
const setCls = (e: HTMLElement, c: string) => { if (e.className !== c) e.className = c; };
/** settings row: value text; `on` highlights it green, `warn` red (only crash protection off) */
const row = (id: string, v: string, on = false, warn = false) => {
  setText(id + 'V', v);
  const e = $(id); e.classList.toggle('on', on); e.classList.toggle('off', warn);
};

// autopilot mode codes from the sim
const M = { takeoff: 0, hold: 1, circuit: 2, land: 3, rescue: 4, autorot: 5, stunt: 6, show: 7 };
const esc = (s: string) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;');
/** the five judges' marks, the dropped highest and lowest struck through */
const marks = (c: ShowCard) => `<span class="marks" aria-label="Judges' marks">${c.marks.map((m, i) => i === c.hi || i === c.lo ? `<s title="dropped">${m}</s>` : `<span>${m}</span>`).join('')}</span>`;

export class Hud {
  private toastEl = $('toast');
  private toastT = 0;
  private lastReal: boolean | null = null;
  /** sensor panel expanded from the health pill */
  sensOpen = false;

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
      `<p>${f.stuntBlocked ? 'Stunts need at least 25% battery.' : 'Tap one to fly it' + (matchMedia('(hover:none) and (pointer:coarse)').matches ? '' : ' · 1-5 or arrows + Enter · Esc closes')}</p>`;
  }

  /** climb bar beside the left stick: centred zero, +-3 m/s, ALT HOLD target mark */
  private vsi(f: Frame) {
    const v = Math.max(-3, Math.min(3, f.vsi)), h = Math.abs(v) / 3 * 50;
    const bar = $('vsiBar') as HTMLElement;
    bar.style.top = (v >= 0 ? 50 - h : 50) + '%'; bar.style.height = h + '%';
    if (bar.className !== f.vsiC) bar.className = f.vsiC;
    const on = f.vsiT > -90;
    $('vsi').classList.toggle('ah', on);
    if (on) ($('vsiT') as HTMLElement).style.top = (50 - Math.max(-3, Math.min(3, f.vsiT)) / 3 * 50) + '%';
  }

  /** event toasts only (landed, crashed, rescued, failsafe, stunt result, GPS reset); the rest
   *  would repeat the status line or a button state */
  showToast(t: Toast) {
    if (!t.ev) return;
    this.toastEl.innerHTML = `${t.t}<small>${t.s}</small>`;
    setCls(this.toastEl, t.k || '');
    this.toastEl.hidden = false;
    this.toastT = 3.0;
  }

  /** downward camera overlay: detection box from the 640 x 480 detection, lock state */
  private pip(f: Frame) {
    const el = $('pip') as HTMLElement;
    el.hidden = !f.pip;
    if (!f.pip) return;
    el.classList.toggle('lock', f.camLock);
    const d = f.camDet, box = $('pipBox') as HTMLElement;
    el.classList.toggle('det', d.length === 4);
    if (d.length === 4) {
      const sx = el.clientWidth / 640, sy = el.clientHeight / 480, w = d[2] * sx, h = d[2] * sy;
      box.style.left = (d[0] * sx - w / 2) + 'px'; box.style.top = (d[1] * sy - h / 2) + 'px';
      box.style.width = w + 'px'; box.style.height = h + 'px'; box.style.transform = `rotate(${-d[3]}rad)`;
    }
    setText('pipL', f.camSt === 'TRACKING' ? (f.camTgt ? 'LOCK · LANDING' : 'LOCK') : f.camSt);
  }

  private showKey = '';
  private scoreKey = '';
  /** SHOW: setup sheet, live bar (manoeuvre, DONE / STOP), the result card, the scoresheet */
  private show(f: Frame) {
    const s = f.show;
    const sheet = $('showSheet') as HTMLElement;
    sheet.hidden = !s.sheet;
    if (s.sheet) {
      const key = JSON.stringify([s.demo, s.part]);
      if (key !== this.showKey) {
        this.showKey = key;
        $('bShowDemo').classList.toggle('on', s.demo); $('bShowMan').classList.toggle('on', !s.demo);
        $('bShowDemo').setAttribute('aria-pressed', String(s.demo)); $('bShowMan').setAttribute('aria-pressed', String(!s.demo));
        setHtml('showPart', s.parts.map((p, i) => `<button data-part="${i}" class="${i === s.part ? 'on' : ''}" aria-pressed="${i === s.part}">${p.n}<small>${p.cnt} · ΣK ${p.k}</small></button>`).join(''));
        setHtml('showList', s.sched.map((m, i) => (s.part === 0 || (s.part === 1) === m.hov) ? `<li><b>${i + 1}. ${m.n}</b><span>K ${m.k}</span></li>` : '').join(''));
        setText('showNote', s.demo
          ? 'The autopilot flies every manoeuvre in front of five judges and positions between them. Touch a stick to stop.'
          : 'The autopilot takes you to each entry and hands over; GO in the status line means it is yours. The yellow line is the ideal path. Crash protection still works: a rescue scores that manoeuvre 0.');
      }
    }
    const bar = $('showBar') as HTMLElement;
    bar.hidden = !s.on;
    if (s.on) {
      setHtml('showBarT', `★ <em>${s.i! + 1}/${s.n}</em> ${esc(s.man || '')} · K ${s.mk} · <em>${s.total}</em> pts`);
      ($('bShowDone') as HTMLElement).hidden = !s.done;
    }
    const card = $('showCard') as HTMLElement, c = s.card;
    card.hidden = !c;
    if (c) setHtml('showCard', `<h4>${esc(c.n)}<small>K ${c.k}</small></h4><div class="sc">${c.score}<small>× K = ${c.ks}</small></div>${marks(c)}` +
      (c.why.length ? `<ul>${c.why.slice(0, 2).map(w => `<li>${esc(w)}</li>`).join('')}</ul>` : ''));
    // scoresheet
    const sc = $('score') as HTMLElement;
    sc.hidden = !(s.resOpen && s.last);
    const L = s.last;
    setText('bScoreRowV', L ? `${L.pct}%` : '—');
    ($('bScoreRow') as HTMLButtonElement).disabled = !L;
    setText('bShowRowV', s.on ? 'RUNNING' : 'START…');
    if (L && s.resOpen) {
      const key = JSON.stringify(L);
      if (key !== this.scoreKey) {
        this.scoreKey = key;
        setText('scoreSub', `${L.title} · ${L.sens}`.toUpperCase());
        setHtml('scoreRows', L.rows.map((r, i) => `<div class="srow${r.zero ? ' z' : ''}"><i>${i + 1}</i><b>${esc(r.n)}<small>K ${r.k}</small></b><em>${r.score}</em><strong>${r.ks}</strong>${marks(r)}` +
          (r.why.length ? `<ul>${r.why.slice(0, 3).map(w => `<li>${esc(w)}</li>`).join('')}</ul>` : '') + '</div>').join(''));
        setText('scoreTot', `${L.total} / ${L.max} · ${L.pct}%`);
        setText('scoreNote', L.complete ? 'Mean of 3 judges × K; highest and lowest struck.' : `Stopped: ${L.note}. Mean of 3 judges × K.`);
      }
    }
  }

  update(f: Frame, dt: number) {
    if (f.toastClear) { this.toastEl.hidden = true; this.toastT = 0; }
    for (const t of f.toasts) this.showToast(t);
    if (this.toastT > 0) { this.toastT -= dt; if (this.toastT <= 0) this.toastEl.hidden = true; }

    const h = f.hud;
    setText('oRpm', h.rpm); setCls($('oRpm'), h.rpmC);
    ($('rRpm') as HTMLElement).hidden = !h.rpmShow;
    setText('oAlt', h.alt);
    setText('oVs', h.vs); setCls($('oVs'), h.vsC);
    setText('oSpd', h.spd);
    setText('oBat', h.bat); setCls($('oBat'), h.batC);
    setText('oLand', h.land);
    setText('oTime', h.time);
    setText('oResc', h.resc);
    setText('oStunts', String(f.stuntN));
    setHtml('cue', h.cue);

    // estimator health pill (REAL) and the sensor panel it expands
    const hp = $('hp') as HTMLElement, sens = $('sens') as HTMLElement;
    hp.hidden = !f.real;
    if (f.real) {
      setText('hp', h.health); setCls(hp, 'hpill ' + h.healthC + (this.sensOpen ? ' open' : ''));
      hp.setAttribute('aria-expanded', String(this.sensOpen));
      if (this.sensOpen) setHtml('sens', h.sens);
    }
    sens.hidden = !(f.real && this.sensOpen);

    // action bar
    const arm = $('bArm'); arm.classList.toggle('armed', f.armed); setText('bArmL', f.armed ? 'ARMED' : 'ARM');
    $('bHold').classList.toggle('on', f.hold);
    $('aTake').classList.toggle('on', f.mode === M.takeoff || f.mode === M.hold);
    $('aCirc').classList.toggle('on', f.mode === M.circuit);
    $('aLand').classList.toggle('on', f.mode === M.land);
    $('aAuto').classList.toggle('on', f.mode === M.autorot);
    const st = $('aStunt'); st.classList.toggle('on', f.mode === M.stunt || f.picker); st.setAttribute('aria-expanded', String(f.picker));
    this.picker(f);
    const sb = $('aShow'); sb.classList.toggle('on', f.show.on || f.show.sheet); sb.setAttribute('aria-expanded', String(f.show.sheet));
    this.show(f);

    // settings sheet rows
    row('bLvl', f.level ? 'SELF-LEVEL' : 'RATE');
    row('bPitch', f.p3d ? '3D' : 'NORMAL');
    row('bProt', f.protect ? (f.mode === M.rescue ? 'ACTIVE' : 'ON') : 'OFF', f.protect, !f.protect);
    row('bWind', f.windName);
    row('bAlt', f.altHold ? 'ON' : 'OFF', f.altHold);
    row('bYaw', f.autoYaw ? 'ON' : 'OFF', f.autoYaw);
    row('bTail', f.tailRate ? 'RATE' : 'HOLD', f.tailRate);
    row('bSens', f.real ? 'REAL' : 'IDEAL', f.real);
    row('bEst', f.ekf ? 'EKF' : 'BASIC');
    row('bCam', ({ pilot: 'PILOT', chase: 'CHASE', pad: 'PAD' } as const)[f.cam]);
    row('bPip', f.pipSet ? 'ALWAYS' : 'AUTO', f.pipSet);
    const r = f.sres, sr = $('sres') as HTMLElement;
    sr.hidden = !r;
    if (r) setHtml('sres', `<span>LAST <b>${r.n}</b></span><span>SCORE <b class="sc">${r.score}</b></span><span>LOST <b>${r.lost} m</b></span>` +
      `<span>DEV <b>${r.dev} m</b></span><span>HDG <b>${r.hdg}°</b></span><span>REC <b>${r.rec} s</b></span>`);

    // assist pills by the sticks: only what is on
    const pills: [boolean, string, string][] = [
      [f.altHold, 'ALT HOLD', ''], [f.autoYaw, 'AUTO-YAW', ''], [f.tailRate, 'TAIL RATE', ''], [f.p3d, '3D PITCH', ''], [f.real, 'REAL SENSORS', 'real'],
    ];
    setHtml('pills', pills.filter(p => p[0]).map(p => `<span class="${p[2]}">${p[1]}</span>`).join(''));

    this.vsi(f);
    this.pip(f);
    if (this.lastReal !== f.real) {
      this.lastReal = f.real;
      ($('est') as HTMLElement).hidden = !f.real;
    }
  }
}
