// three.js scene: sky, field, pads, flight line, trees, windsock, the heli model, the blue
// "ghost" showing the flight controller's belief in REAL sensor mode, and the three cameras.
// Pure rendering: everything it draws comes from the sim frame.

import { THREE } from './three';
import type { Frame } from './engine';

const ROTOR_R = 0.68, MAST_H = 0.29, PAD_R = 1.0;
/** the single pad at the origin; TAG = side of the black tag square (matches heli::TAG_SIZE) */
const PAD = { x: 0, z: 0, color: '#ff7a1a' }, TAG = 0.9;
/** downward camera: 70 deg horizontal FOV, 4:3, mounted under the fuselage (precland::cam::MOUNT) */
const CAM_HFOV = 70, CAM_MOUNT = [0, -0.15, -0.05];
/** 6 x 6 tag payload inside a one-cell black border (8 x 8 cells), AprilTag style */
const TAG_BITS = ['110010', '011101', '100110', '001011', '111000', '010111'];
/** SHOW flight box (show::geo): hovering start circle, aerobatic judging line, centre and edges */
const BOX = { hx: 0, hz: -3.5, ax: 0, az: -14, edge: 25 };
/** smoke trail: samples kept (at about 15 per second) */
const TRAIL_N = 600;

export class View3D {
  private renderer: InstanceType<typeof THREE.WebGLRenderer>;
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(45, 1, 0.05, 600);
  private sun = new THREE.DirectionalLight(0xfff2dc, 1.0);
  private sock = new THREE.Group();
  private pivot = new THREE.Group();
  private heli = new THREE.Group();
  private blades = new THREE.Group();
  private tailRotor = new THREE.Group();
  private ghost = new THREE.Group();
  /** 2-sigma horizontal uncertainty ring around the ghost (unit radius, scaled per frame) */
  private sigRing!: THREE.Mesh;
  private bladeMat!: InstanceType<typeof THREE.MeshPhongMaterial>;
  private discMat!: InstanceType<typeof THREE.MeshBasicMaterial>;
  private PILOT = new THREE.Vector3(0, 1.7, 9);
  private camPos = new THREE.Vector3(0, 1.7, 9);
  private camLook = new THREE.Vector3(0, 0.29, 0);
  private spin = 0; private tspin = 0;
  /** SHOW: ghost path (ideal reference), smoke trail (flown path, fading) */
  private ghostLine!: InstanceType<typeof THREE.Line>;
  private ghostGid = -1;
  private trail!: InstanceType<typeof THREE.Line>;
  private trailPos = new Float32Array(TRAIL_N * 3);
  private trailCol = new Float32Array(TRAIL_N * 3);
  private trailN = 0;
  private trailT = 0;
  private trailOn = false;
  private boxGroup = new THREE.Group();
  /** called when the ghost path must be sent again (unused today: kept for a view rebuild) */
  onLost: () => void = () => {};
  /** SHOW state for page tests: trail points drawn, ghost path shown */
  debug() { return { trail: this.trail.visible ? this.trailN : 0, ghost: this.ghostLine.visible ? (this.ghostLine.geometry.attributes.position?.count ?? 0) : 0 }; }
  /** downward camera for the picture-in-picture */
  private dcam = new THREE.PerspectiveCamera(2 * Math.atan(Math.tan(CAM_HFOV * Math.PI / 360) * 0.75) * 180 / Math.PI, 4 / 3, 0.05, 120);

  constructor(private host: HTMLElement) {
    const r = new THREE.WebGLRenderer({ antialias: true });
    r.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap;
    r.outputEncoding = THREE.sRGBEncoding;
    host.appendChild(r.domElement);
    this.renderer = r;
    this.build();
    window.addEventListener('resize', () => this.resize());
    this.resize();
  }

  private build() {
    const { scene, sun } = this;
    const V3 = THREE.Vector3, Q = THREE.Quaternion;
    // sky
    {
      const c = document.createElement('canvas'); c.width = 4; c.height = 256; const x = c.getContext('2d')!;
      const g = x.createLinearGradient(0, 0, 0, 256); g.addColorStop(0, '#4f86c0'); g.addColorStop(0.55, '#9cc3e0'); g.addColorStop(1, '#e6eee6');
      x.fillStyle = g; x.fillRect(0, 0, 4, 256);
      const tex = new THREE.CanvasTexture(c); tex.encoding = THREE.sRGBEncoding; scene.background = tex;
      scene.fog = new THREE.Fog(0xcfdde4, 60, 260);
    }
    scene.add(new THREE.HemisphereLight(0xdfeeff, 0x4f6b35, 0.75));
    sun.castShadow = true; sun.shadow.mapSize.set(1024, 1024);
    Object.assign(sun.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: 0.5, far: 60 });
    sun.shadow.bias = -0.0005;
    scene.add(sun, sun.target);
    // ground
    {
      const c = document.createElement('canvas'); c.width = c.height = 512; const x = c.getContext('2d')!;
      for (let i = 0; i < 8; i++) { x.fillStyle = i % 2 ? '#5f8f3e' : '#6a9a45'; x.fillRect(i * 64, 0, 64, 512); }
      for (let i = 0; i < 9000; i++) { x.fillStyle = `rgba(${30 + Math.random() * 40},${60 + Math.random() * 50},20,.18)`; x.fillRect(Math.random() * 512, Math.random() * 512, 2, 2); }
      const tex = new THREE.CanvasTexture(c); tex.wrapS = tex.wrapT = THREE.RepeatWrapping; tex.repeat.set(30, 30); tex.encoding = THREE.sRGBEncoding;
      tex.anisotropy = this.renderer.capabilities.getMaxAnisotropy();
      const g = new THREE.Mesh(new THREE.PlaneGeometry(600, 600), new THREE.MeshLambertMaterial({ map: tex }));
      g.rotation.x = -Math.PI / 2; g.receiveShadow = true; scene.add(g);
    }
    // the pad: coloured ring and an AprilTag-style marker (8 x 8 cells: black border, 6 x 6 payload)
    {
      const c = document.createElement('canvas'); c.width = c.height = 512; const x = c.getContext('2d')!;
      x.fillStyle = '#d9d6cc'; x.beginPath(); x.arc(256, 256, 252, 0, 7); x.fill();
      x.strokeStyle = PAD.color; x.lineWidth = 26; x.beginPath(); x.arc(256, 256, 226, 0, 7); x.stroke();
      // tag square of side TAG m on a pad of radius PAD_R: 256 * TAG / PAD_R px, with a white quiet zone
      const side = 256 * TAG / PAD_R, cell = side / 8, o = 256 - side / 2;
      x.fillStyle = '#f4f2ea'; x.fillRect(o - cell * 0.6, o - cell * 0.6, side + cell * 1.2, side + cell * 1.2);
      x.fillStyle = '#111'; x.fillRect(o, o, side, side);
      x.fillStyle = '#f4f2ea';
      TAG_BITS.forEach((r, j) => { for (let i = 0; i < 6; i++) if (r[i] === '1') x.fillRect(o + (i + 1) * cell, o + (j + 1) * cell, cell + 0.5, cell + 0.5); });
      const tex = new THREE.CanvasTexture(c); tex.encoding = THREE.sRGBEncoding; tex.anisotropy = this.renderer.capabilities.getMaxAnisotropy();
      const m = new THREE.Mesh(new THREE.CircleGeometry(PAD_R, 64), new THREE.MeshLambertMaterial({ map: tex }));
      m.rotation.x = -Math.PI / 2; m.position.set(PAD.x, 0.006, PAD.z); m.receiveShadow = true; scene.add(m);
    }
    // flight line rope + posts between pilot and pads, cones
    {
      const postG = new THREE.CylinderGeometry(0.03, 0.03, 0.9, 8), postM = new THREE.MeshLambertMaterial({ color: 0xf2f2f2 });
      for (let x = -12; x <= 12; x += 3) { const m = new THREE.Mesh(postG, postM); m.position.set(x, 0.45, 6.5); m.castShadow = true; scene.add(m); }
      const rope = new THREE.Mesh(new THREE.BoxGeometry(24, 0.02, 0.02), new THREE.MeshBasicMaterial({ color: 0xffd23c }));
      rope.position.set(0, 0.85, 6.5); scene.add(rope);
      const coneG = new THREE.ConeGeometry(0.15, 0.35, 12), coneM = new THREE.MeshLambertMaterial({ color: 0xff6a1f });
      for (const [x, z] of [[-3, -3], [3, -3], [10, -10], [-8, -12], [14, -2], [-10, -20], [14, -20]]) { const m = new THREE.Mesh(coneG, coneM); m.position.set(x, 0.175, z); m.castShadow = true; scene.add(m); }
    }
    // SHOW flight box: judging line on the grass, centre flag, box-edge poles, start circle,
    // judges' table beside the pilot station
    {
      const g = this.boxGroup;
      const lineM = new THREE.MeshBasicMaterial({ color: 0xf4f2ea, transparent: true, opacity: 0.32, depthWrite: false });
      const jl = new THREE.Mesh(new THREE.PlaneGeometry(BOX.edge * 2, 0.12), lineM); jl.rotation.x = -Math.PI / 2; jl.position.set(BOX.ax, 0.012, BOX.az); g.add(jl);
      const hl = new THREE.Mesh(new THREE.PlaneGeometry(12, 0.08), lineM); hl.rotation.x = -Math.PI / 2; hl.position.set(BOX.hx, 0.012, BOX.hz); g.add(hl);
      const sc = new THREE.Mesh(new THREE.RingGeometry(0.42, 0.5, 40), new THREE.MeshBasicMaterial({ color: 0xffd23c, transparent: true, opacity: 0.75, depthWrite: false }));
      sc.rotation.x = -Math.PI / 2; sc.position.set(BOX.hx, 0.013, BOX.hz); g.add(sc);
      const poleG = new THREE.CylinderGeometry(0.05, 0.06, 4, 8);
      const red = new THREE.MeshLambertMaterial({ color: 0xe8432c }), white = new THREE.MeshLambertMaterial({ color: 0xf2f2f2 }), yel = new THREE.MeshLambertMaterial({ color: 0xffd23c, side: THREE.DoubleSide });
      for (const sx of [-1, 1]) {
        const p = new THREE.Mesh(poleG, red); p.position.set(BOX.ax + sx * BOX.edge, 2, BOX.az); p.castShadow = true; g.add(p);
        const t = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.3, 0.02), white); t.position.set(BOX.ax + sx * BOX.edge, 3.8, BOX.az); g.add(t);
      }
      const fp = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 3, 8), white); fp.position.set(BOX.ax, 1.5, BOX.az); fp.castShadow = true; g.add(fp);
      const flag = new THREE.Mesh(new THREE.PlaneGeometry(0.8, 0.5), yel); flag.position.set(BOX.ax + 0.4, 2.7, BOX.az); g.add(flag);
      const cone = new THREE.Mesh(new THREE.ConeGeometry(0.2, 0.45, 12), new THREE.MeshLambertMaterial({ color: 0xffd23c })); cone.position.set(BOX.hx, 0.225, BOX.hz - 0.9); cone.castShadow = true; g.add(cone);
      // judges' table: a top and two trestles, five chairs, to the pilot's left
      const tm = new THREE.MeshLambertMaterial({ color: 0xe9e4d8 }), dm = new THREE.MeshLambertMaterial({ color: 0x3a3f44 });
      const top = new THREE.Mesh(new THREE.BoxGeometry(3.2, 0.05, 0.75), tm); top.position.set(-4.5, 0.74, 9.4); top.castShadow = true; g.add(top);
      for (const dx of [-1.4, 1.4]) { const l = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.72, 0.6), dm); l.position.set(-4.5 + dx, 0.36, 9.4); g.add(l); }
      for (let i = 0; i < 5; i++) { const c = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.45, 0.42), dm); c.position.set(-5.9 + i * 0.7, 0.225, 10.1); g.add(c); }
      this.scene.add(g);
      // ghost path (yellow, dashed look from transparency) and the smoke trail (white, fading)
      this.ghostLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffd23c, transparent: true, opacity: 0.85, depthWrite: false }));
      this.ghostLine.visible = false; this.ghostLine.frustumCulled = false; this.scene.add(this.ghostLine);
      const tg = new THREE.BufferGeometry();
      tg.setAttribute('position', new THREE.BufferAttribute(this.trailPos, 3));
      tg.setAttribute('color', new THREE.BufferAttribute(this.trailCol, 3));
      tg.setDrawRange(0, 0);
      this.trail = new THREE.Line(tg, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.95, depthWrite: false }));
      this.trail.frustumCulled = false; this.scene.add(this.trail);
    }
    // tree ring (instanced)
    {
      const N = 120, rnd = (s => () => (s = (s * 16807) % 2147483647) / 2147483647)(42);
      const crown = new THREE.InstancedMesh(new THREE.ConeGeometry(1, 1, 8), new THREE.MeshLambertMaterial({ color: 0x3c6630 }), N);
      const trunk = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.15, 0.2, 1, 6), new THREE.MeshLambertMaterial({ color: 0x5a4330 }), N);
      const m = new THREE.Matrix4(), s = new V3(), pos = new V3(), q0 = new Q();
      for (let i = 0; i < N; i++) {
        const a = rnd() * Math.PI * 2, r = 45 + rnd() * 70;
        const x = Math.cos(a) * r, z = Math.sin(a) * r - 20;
        if (z > 20 && Math.abs(x) < 30) { i--; continue; }
        const hgt = 5 + rnd() * 7, w = 1.6 + rnd() * 1.6;
        m.compose(pos.set(x, hgt * 0.35 + hgt * 0.65 / 2, z), q0, s.set(w, hgt * 0.75, w)); crown.setMatrixAt(i, m);
        m.compose(pos.set(x, hgt * 0.2, z), q0, s.set(1, hgt * 0.4, 1)); trunk.setMatrixAt(i, m);
      }
      scene.add(crown, trunk);
    }
    // windsock
    {
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 3, 8), new THREE.MeshLambertMaterial({ color: 0xdddddd }));
      pole.position.y = 1.5; pole.castShadow = true;
      const cloth = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.2, 0.9, 12, 1, true), new THREE.MeshLambertMaterial({ color: 0xff6a1f, side: THREE.DoubleSide }));
      cloth.rotation.z = Math.PI / 2; cloth.position.set(0.45, 0, 0);
      this.pivot.position.y = 2.9; this.pivot.add(cloth);
      this.sock.add(pole, this.pivot); this.sock.position.set(-7, 0, -4); scene.add(this.sock);
    }
    // helicopter model (nose toward -z)
    const heli = this.heli, blades = this.blades, tailRotor = this.tailRotor;
    scene.add(heli);
    {
      const L = (c: number, o: object = {}) => new THREE.MeshPhongMaterial(Object.assign({ color: c, shininess: 60 }, o));
      const add = (geo: InstanceType<typeof THREE.BufferGeometry>, mat: InstanceType<typeof THREE.Material>, x: number, y: number, z: number, rx = 0, ry = 0, rz = 0) => {
        const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.rotation.set(rx, ry, rz); m.castShadow = true; heli.add(m); return m;
      };
      const can = add(new THREE.SphereGeometry(1, 24, 16), L(0xff6a14), 0, 0.02, -0.12); can.scale.set(0.15, 0.16, 0.36);
      const ws = add(new THREE.SphereGeometry(1, 20, 12, 0, Math.PI * 2, 0, Math.PI / 2.4), L(0x1c2a33, { shininess: 120 }), 0, 0.06, -0.28, -1.05); ws.scale.set(0.12, 0.14, 0.13);
      add(new THREE.BoxGeometry(0.16, 0.12, 0.34), L(0x2e3338), 0, 0.02, 0.12);          // frame
      add(new THREE.BoxGeometry(0.1, 0.05, 0.22), L(0x56c2ff), 0, 0.1, 0.16);             // battery
      add(new THREE.CylinderGeometry(0.022, 0.016, 0.78, 10), L(0xf2f2f2), 0, 0.04, 0.62, Math.PI / 2); // boom
      add(new THREE.BoxGeometry(0.012, 0.2, 0.11), L(0xe11d2a), 0, 0.1, 0.98);             // fin (red, marks the tail)
      add(new THREE.BoxGeometry(0.22, 0.01, 0.06), L(0x222222), 0, 0.03, 0.7);             // stabiliser
      add(new THREE.CylinderGeometry(0.012, 0.012, 0.2, 8), L(0x888888), 0, 0.19, 0);      // mast
      add(new THREE.CylinderGeometry(0.04, 0.04, 0.04, 12), L(0x3a3a3a), 0, MAST_H - 0.02, 0); // head
      const skM = L(0x1d1d1d);
      for (const sx of [0.14, -0.14]) {
        add(new THREE.CylinderGeometry(0.01, 0.01, 0.58, 8), skM, sx, -0.29, 0.01, Math.PI / 2);
        add(new THREE.CylinderGeometry(0.008, 0.008, 0.24, 6), skM, sx * 0.75, -0.18, -0.12, 0, 0, sx > 0 ? 0.35 : -0.35);
        add(new THREE.CylinderGeometry(0.008, 0.008, 0.24, 6), skM, sx * 0.75, -0.18, 0.14, 0, 0, sx > 0 ? 0.35 : -0.35);
      }
      this.bladeMat = L(0x161616, { transparent: true });
      const bg = new THREE.BoxGeometry(ROTOR_R * 2, 0.004, 0.05);
      const b1 = new THREE.Mesh(bg, this.bladeMat); b1.castShadow = true; blades.add(b1);
      const tipM = L(0xffd23c);
      for (const sx of [ROTOR_R - 0.03, -ROTOR_R + 0.03]) { const t = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.005, 0.051), tipM); t.position.x = sx; blades.add(t); }
      blades.position.y = MAST_H; heli.add(blades);
      this.discMat = new THREE.MeshBasicMaterial({ color: 0x1a1a1a, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide });
      const disc = new THREE.Mesh(new THREE.RingGeometry(0.06, ROTOR_R, 48), this.discMat); disc.rotation.x = -Math.PI / 2; disc.position.y = MAST_H; heli.add(disc);
      const tb = new THREE.Mesh(new THREE.BoxGeometry(0.004, 0.24, 0.02), L(0x161616)); tailRotor.add(tb);
      tailRotor.position.set(0.02, 0.06, 1.01); heli.add(tailRotor);
    }
    // flight controller's belief, shown in REAL sensor mode
    {
      const gm = new THREE.MeshBasicMaterial({ color: 0x4fb6ff, transparent: true, opacity: 0.55, depthWrite: false });
      const ring = new THREE.Mesh(new THREE.RingGeometry(0.55, 0.62, 40), gm); ring.rotation.x = -Math.PI / 2; ring.position.y = MAST_H;
      const cone = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.3, 10), gm); cone.rotation.x = -Math.PI / 2; cone.position.set(0, 0, -0.35);
      const boom = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.02, 0.9), gm); boom.position.z = 0.3;
      this.ghost.add(ring, cone, boom); this.ghost.visible = false; scene.add(this.ghost);
      const sm = new THREE.MeshBasicMaterial({ color: 0x4fb6ff, transparent: true, opacity: 0.16, depthWrite: false, side: THREE.DoubleSide });
      this.sigRing = new THREE.Mesh(new THREE.RingGeometry(0.93, 1, 48), sm); this.sigRing.rotation.x = -Math.PI / 2;
      this.sigRing.visible = false; scene.add(this.sigRing);
    }
  }

  resize() {
    const w = this.host.clientWidth, h = this.host.clientHeight;
    this.renderer.setSize(w, h, false); this.camera.aspect = w / Math.max(1, h); this.camera.updateProjectionMatrix();
  }

  private updateCamera(f: Frame, dt: number) {
    const { camera, camPos, camLook } = this;
    const target = new THREE.Vector3(f.p[0], f.p[1], f.p[2]);
    const clamp = THREE.MathUtils.clamp;
    if (f.cam === 'pilot') {
      camPos.copy(this.PILOT);
      camLook.lerp(target, Math.min(1, dt * 12));
      const d = camPos.distanceTo(target);
      const fov = clamp(2 * Math.atan(3.2 / d) * 180 / Math.PI, 16, 52);
      camera.fov += (fov - camera.fov) * Math.min(1, dt * 3);
    } else if (f.cam === 'chase') {
      const q = new THREE.Quaternion(f.q[0], f.q[1], f.q[2], f.q[3]);
      const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(q); fwd.y = 0; if (fwd.lengthSq() < 1e-4) fwd.set(0, 0, -1); fwd.normalize();
      const want = new THREE.Vector3().copy(target).addScaledVector(fwd, -3.6); want.y = Math.max(0.6, target.y + 1.3);
      camPos.lerp(want, Math.min(1, dt * 3)); camLook.lerp(target, Math.min(1, dt * 10));
      camera.fov += (55 - camera.fov) * Math.min(1, dt * 3);
    } else {
      // pad close-up: low beside the pad, looking at the heli
      camPos.set(PAD.x - 3.2, 1.0, PAD.z + 2.6);
      camLook.lerp(target, Math.min(1, dt * 10));
      const d = camPos.distanceTo(target);
      camera.fov += (clamp(2 * Math.atan(3 / d) * 180 / Math.PI, 18, 60) - camera.fov) * Math.min(1, dt * 3);
    }
    camera.position.copy(camPos); camera.lookAt(camLook); camera.updateProjectionMatrix();
  }

  /** downward camera into a viewport over the PiP box (scissored, same renderer) */
  private renderPip(f: Frame, box: HTMLElement) {
    const r = this.renderer, host = this.host.getBoundingClientRect(), b = box.getBoundingClientRect();
    if (b.width < 4 || b.height < 4) return;
    const q = new THREE.Quaternion(f.q[0], f.q[1], f.q[2], f.q[3]);
    const off = new THREE.Vector3(CAM_MOUNT[0], CAM_MOUNT[1], CAM_MOUNT[2]).applyQuaternion(q);
    const c = this.dcam;
    c.position.set(f.p[0] + off.x, f.p[1] + off.y, f.p[2] + off.z);
    // looking down the body -y axis, image up = nose (-z): a camera rotated -90 deg about body x
    c.quaternion.copy(q).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2));
    const x = b.left - host.left, y = host.bottom - b.bottom;
    r.setScissorTest(true); r.setScissor(x, y, b.width, b.height); r.setViewport(x, y, b.width, b.height);
    const vis = this.heli.visible; this.heli.visible = false; this.ghost.visible = false; this.sigRing.visible = false;
    r.render(this.scene, c);
    this.heli.visible = vis;
    r.setScissorTest(false); r.setViewport(0, 0, host.width, host.height);
  }

  /** SHOW: the ideal path (rebuilt when its id changes) and the flown path as a fading smoke trail */
  private showPaths(f: Frame, dt: number) {
    const s = f.show;
    if (!s.on) { this.ghostLine.visible = false; this.trail.visible = false; this.trailN = 0; this.ghostGid = -1; return; }
    if (s.ghost && s.gid !== undefined && s.gid !== this.ghostGid) {
      this.ghostGid = s.gid;
      const a = new Float32Array(s.ghost.length * 3);
      s.ghost.forEach((p, i) => { a[i * 3] = p[0]; a[i * 3 + 1] = p[1]; a[i * 3 + 2] = p[2]; });
      this.ghostLine.geometry.dispose();
      const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(a, 3));
      this.ghostLine.geometry = g;
      this.trailN = 0;
    }
    this.ghostLine.visible = this.ghostGid >= 0;
    // trail: new samples while the manoeuvre is flown, fading over its length
    const on = !!s.trail;
    if (on && !this.trailOn) this.trailN = 0;
    this.trailOn = on;
    this.trailT += dt;
    if (on && this.trailT > 1 / 15) {
      this.trailT = 0;
      if (this.trailN >= TRAIL_N) { this.trailPos.copyWithin(0, 3); this.trailN = TRAIL_N - 1; }
      const i = this.trailN++ * 3;
      this.trailPos[i] = f.p[0]; this.trailPos[i + 1] = f.p[1] - 0.15; this.trailPos[i + 2] = f.p[2];
    }
    const n = this.trailN;
    for (let k = 0; k < n; k++) { const b = 0.35 + 0.65 * (n > 1 ? k / (n - 1) : 1); this.trailCol[k * 3] = b; this.trailCol[k * 3 + 1] = b; this.trailCol[k * 3 + 2] = b; }
    const g = this.trail.geometry;
    g.setDrawRange(0, n);
    (g.attributes.position as InstanceType<typeof THREE.BufferAttribute>).needsUpdate = true;
    (g.attributes.color as InstanceType<typeof THREE.BufferAttribute>).needsUpdate = true;
    this.trail.visible = n > 1;
  }

  render(f: Frame, dt: number, nowMs: number, pip?: HTMLElement | null) {
    this.showPaths(f, dt);
    const { heli, ghost } = this;
    heli.position.set(f.p[0], f.p[1], f.p[2]); heli.quaternion.set(f.q[0], f.q[1], f.q[2], f.q[3]);
    ghost.visible = f.real;
    if (f.real) { ghost.position.set(f.ep[0], f.ep[1], f.ep[2]); ghost.quaternion.set(f.eq[0], f.eq[1], f.eq[2], f.eq[3]); }
    const r = f.real ? f.sig2 : 0;
    this.sigRing.visible = r > 0;
    if (r > 0) { this.sigRing.position.set(f.ep[0], f.ep[1] + 0.29, f.ep[2]); this.sigRing.scale.set(r, r, 1); }
    this.spin += f.rpm * 24 * dt; this.tspin += f.rpm * 70 * dt;
    this.blades.rotation.y = -this.spin; this.tailRotor.rotation.x = this.tspin;
    this.discMat.opacity = Math.max(0, (f.rpm - 0.3) * 0.32);
    this.bladeMat.opacity = 1 - Math.max(0, (f.rpm - 0.35)) * 0.9;
    const w = f.wind, ws = Math.hypot(w[0], w[2]);
    this.pivot.rotation.y = Math.atan2(-w[2], w[0]);
    this.pivot.rotation.z = -Math.max(0, 1.25 - ws * 0.35) + Math.sin(nowMs / 160) * 0.04 * ws;
    this.sun.position.set(f.p[0] + 8, f.p[1] + 18, f.p[2] + 5); this.sun.target.position.set(f.p[0], f.p[1], f.p[2]);
    this.updateCamera(f, dt);
    this.renderer.render(this.scene, this.camera);
    if (pip && f.pip) {
      const ghostVis = this.ghost.visible, ringVis = this.sigRing.visible;
      this.renderPip(f, pip);
      this.ghost.visible = ghostVis; this.sigRing.visible = ringVis;
    }
  }
}
