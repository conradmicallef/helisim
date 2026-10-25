// Loads the helisim wasm core from the base64 + gzip string the build embeds in the page,
// and wraps its C-ABI exports. Only plain numbers cross the boundary; state comes back as JSON.

export interface Toast { t: string; s: string; k: string }
export interface Hud {
  thr: string; rpm: string; rpmC: string; alt: string; vs: string; vsC: string; spd: string;
  bat: string; batC: string; land: string; time: string; resc: string; cue: string; sens: string;
}
export interface StuntItem { id: number; n: string; d: string; key: string; best: number }
export interface StuntRes { n: string; score: number; lost: string; dev: string; hdg: string; rec: string; minAlt: string }
export interface Frame {
  p: [number, number, number]; q: [number, number, number, number];
  ep: [number, number, number]; eq: [number, number, number, number];
  rpm: number; armed: boolean; crashed: boolean; hold: boolean; simT: number;
  wind: [number, number, number];
  level: boolean; p3d: boolean; protect: boolean; real: boolean; ekf: boolean;
  /** 2-sigma horizontal uncertainty radius of the estimate in metres (0 unless REAL + EKF) */
  sig2: number;
  windName: string; cam: 'pilot' | 'chase' | 'pad';
  /** autopilot mode code: -1 off, 0 takeoff, 1 hold, 2 circuit, 3 land, 4 rescue, 5 autorotate, 6 stunt */
  mode: number; rescue: boolean;
  /** gimbal display: left x, left y, right x, right y */
  stk: [number, number, number, number];
  hud: Hud;
  /** stunt picker open, stunt list, live stunt (kind / phase, -1 none), last result */
  picker: boolean; stunts: StuntItem[]; stuntK: number; stuntPh: number; stuntBlocked: boolean; stuntN: number;
  sres?: StuntRes;
  toastClear: boolean; toasts: Toast[];
}

/** Command codes, matching `sim::Command` in Rust. */
export const Cmd = {
  Arm: 0, Hold: 1, Protect: 2, Sensors: 3, Level: 4, Pitch: 5, Wind: 6, Cam: 7, Reset: 8,
  Takeoff: 9, Circuit: 10, Land: 11, Autorot: 12, Takeover: 13, Estimator: 14,
  Stunts: 15, StuntFirst: 16, PickerClose: 21,
} as const;

interface Exports {
  memory: WebAssembly.Memory;
  sim_init(seed: number): void;
  sim_input(yaw: number, pit: number, rol: number, thrDelta: number, thrSet: number, gp: number, g0: number, g1: number, g2: number, g3: number): void;
  sim_cmd(code: number): void;
  sim_step(n: number): void;
  sim_frame(): number;
  sim_out_ptr(): number;
}

export async function wasmBytes(b64: string): Promise<ArrayBuffer> {
  const bin = atob(b64);
  const raw = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) raw[i] = bin.charCodeAt(i);
  const ds = new Blob([raw]).stream().pipeThrough(new DecompressionStream('gzip'));
  return new Response(ds).arrayBuffer();
}

export class Engine {
  private x: Exports;
  private dec = new TextDecoder();
  private constructor(x: Exports) { this.x = x; }

  static async load(b64: string, seed: number): Promise<Engine> {
    const { instance } = await WebAssembly.instantiate(await wasmBytes(b64), {});
    const e = new Engine(instance.exports as unknown as Exports);
    e.x.sim_init(seed);
    return e;
  }
  input(yaw: number, pit: number, rol: number, thrDelta: number, thrSet: number | null, gp: readonly number[] | null) {
    this.x.sim_input(yaw, pit, rol, thrDelta, thrSet === null ? -1 : thrSet, gp ? 1 : 0,
      gp ? gp[0] : 0, gp ? gp[1] : 0, gp ? gp[2] : 0, gp ? gp[3] : 0);
  }
  cmd(code: number) { this.x.sim_cmd(code); }
  step(n: number) { if (n > 0) this.x.sim_step(n); }
  frame(): Frame {
    const n = this.x.sim_frame(), ptr = this.x.sim_out_ptr();
    return JSON.parse(this.dec.decode(new Uint8Array(this.x.memory.buffer, ptr, n))) as Frame;
  }
}
