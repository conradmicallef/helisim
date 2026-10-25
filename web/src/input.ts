// Pilot input: two touch gimbals (Mode 2), keyboard and a USB transmitter / gamepad.
// Gathers raw values only; the sim decides what they mean (takeover, gamepad override, expo).

import { Cmd } from './engine';

export interface InputState {
  yaw: number; pit: number; rol: number;
  thrDelta: number; thrSet: number | null;
  gp: number[] | null;
}

type Send = (code: number) => void;

export class Input {
  private keys: Record<string, boolean> = {};
  private touchYaw = 0; private touchRol = 0; private touchPit = 0;
  private thrSet: number | null = null;
  readonly drawL: (x: number, y: number) => void;
  readonly drawR: (x: number, y: number) => void;

  constructor(private send: Send, onKey: (k: string) => void, private pickerOpen: () => boolean = () => false) {
    this.drawL = this.bindStick(document.getElementById('stL')!, (x, y) => { this.touchYaw = x; this.thrSet = (y + 1) / 2; }, () => { this.touchYaw = 0; });
    this.drawR = this.bindStick(document.getElementById('stR')!, (x, y) => { this.touchRol = x; this.touchPit = y; }, () => { this.touchRol = 0; this.touchPit = 0; });
    window.addEventListener('keydown', e => {
      const k = e.key.toLowerCase();
      if (['arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '].includes(k)) e.preventDefault();
      if (this.pickerOpen() && ['arrowup', 'arrowdown', 'enter', 'escape', '1', '2', '3', '4', '5'].includes(k)) {
        // the open stunt picker owns these keys: no stick input, no takeover
        e.preventDefault();
        if (!e.repeat) onKey(k);
        return;
      }
      this.keys[k] = true;
      if (e.repeat) return;
      if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) this.send(Cmd.Takeover);
      onKey(k);
    });
    window.addEventListener('keyup', e => { this.keys[e.key.toLowerCase()] = false; });
    window.addEventListener('blur', () => { for (const k in this.keys) this.keys[k] = false; });
  }

  private bindStick(el: HTMLElement, onMove: (x: number, y: number) => void, onUp: () => void) {
    const knob = el.querySelector('.knob') as HTMLElement; let id: number | null = null;
    const apply = (e: PointerEvent) => {
      const r = el.getBoundingClientRect(), rad = r.width / 2 - 21;
      const x = Math.max(-1, Math.min(1, (e.clientX - r.left - r.width / 2) / rad));
      const y = Math.max(-1, Math.min(1, -(e.clientY - r.top - r.height / 2) / rad));
      onMove(x, y);
    };
    el.addEventListener('pointerdown', e => { this.send(Cmd.Takeover); id = e.pointerId; el.setPointerCapture(id); el.classList.add('live'); apply(e); e.preventDefault(); });
    el.addEventListener('pointermove', e => { if (e.pointerId === id) apply(e); });
    const end = (e: PointerEvent) => { if (e.pointerId !== id) return; id = null; el.classList.remove('live'); onUp(); };
    el.addEventListener('pointerup', end); el.addEventListener('pointercancel', end);
    return (x: number, y: number) => {
      const rad = el.clientWidth / 2 - 21;
      knob.style.left = `calc(50% + ${x * rad}px)`; knob.style.top = `calc(50% - ${y * rad}px)`;
    };
  }

  /** Read this frame's input. W/S ramp the throttle at 45% per second. */
  read(dt: number): InputState {
    const k = this.keys;
    let thrDelta = 0;
    if (k['w']) thrDelta += 0.45 * dt;
    if (k['s']) thrDelta -= 0.45 * dt;
    let yaw = this.touchYaw, rol = this.touchRol, pit = this.touchPit;
    if (k['a']) yaw = -1; if (k['d']) yaw = 1;
    if (k['arrowleft']) rol = -1; if (k['arrowright']) rol = 1;
    if (k['arrowup']) pit = 1; if (k['arrowdown']) pit = -1;
    let gp: number[] | null = null;
    try {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      for (const p of pads) { if (!p || p.axes.length < 4) continue; gp = [p.axes[0], p.axes[1], p.axes[2], p.axes[3]]; break; }
    } catch (_) { /* no gamepad API */ }
    const thrSet = this.thrSet; this.thrSet = null;
    return { yaw, pit, rol, thrDelta, thrSet, gp };
  }
}
