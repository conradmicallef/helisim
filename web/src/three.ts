// three.js r128 is loaded by a <script> tag from cdnjs; only its types are imported here,
// so the bundle stays small and the page uses the same library build as before.
import type * as THREE_NS from 'three';
export const THREE = (globalThis as unknown as { THREE: typeof THREE_NS }).THREE;
export type { THREE_NS };
