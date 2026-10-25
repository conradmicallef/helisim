//! Tiny Vec3 / quaternion module with the exact semantics of three.js r128 `Vector3` and
//! `Quaternion` (rotation convention, multiply order, slerp), so the port keeps the JS numbers
//! and the wasm build stays dependency-free.

use core::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}
pub const Y: V3 = v3(0.0, 1.0, 0.0);

impl V3 {
    pub const ZERO: V3 = v3(0.0, 0.0, 0.0);
    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, b: V3) -> V3 {
        v3(self.y * b.z - self.z * b.y, self.z * b.x - self.x * b.z, self.x * b.y - self.y * b.x)
    }
    pub fn length_sq(self) -> f64 {
        self.dot(self)
    }
    pub fn length(self) -> f64 {
        self.length_sq().sqrt()
    }
    /// three.js: divides by length, or by 1 when the length is zero.
    pub fn normalize(self) -> V3 {
        let l = self.length();
        self * (1.0 / if l == 0.0 { 1.0 } else { l })
    }
    pub fn lerp(self, to: V3, t: f64) -> V3 {
        self + (to - self) * t
    }
    pub fn hyp_xz(self) -> f64 {
        self.x.hypot(self.z)
    }
    /// Rotate by a unit quaternion (three.js `applyQuaternion`).
    pub fn rot(self, q: Quat) -> V3 {
        let (x, y, z) = (self.x, self.y, self.z);
        let (qx, qy, qz, qw) = (q.x, q.y, q.z, q.w);
        let ix = qw * x + qy * z - qz * y;
        let iy = qw * y + qz * x - qx * z;
        let iz = qw * z + qx * y - qy * x;
        let iw = -qx * x - qy * y - qz * z;
        v3(
            ix * qw + iw * -qx + iy * -qz - iz * -qy,
            iy * qw + iw * -qy + iz * -qx - ix * -qz,
            iz * qw + iw * -qz + ix * -qy - iy * -qx,
        )
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for V3 {
    type Output = V3;
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Mul<f64> for V3 {
    type Output = V3;
    fn mul(self, s: f64) -> V3 {
        v3(self.x * s, self.y * s, self.z * s)
    }
}
impl Neg for V3 {
    type Output = V3;
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}
impl AddAssign for V3 {
    fn add_assign(&mut self, o: V3) {
        *self = *self + o;
    }
}
impl SubAssign for V3 {
    fn sub_assign(&mut self, o: V3) {
        *self = *self - o;
    }
}
impl MulAssign<f64> for V3 {
    fn mul_assign(&mut self, s: f64) {
        *self = *self * s;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };
    pub fn axis_angle(axis: V3, angle: f64) -> Quat {
        let (s, c) = (angle / 2.0).sin_cos();
        Quat { x: axis.x * s, y: axis.y * s, z: axis.z * s, w: c }
    }
    /// Conjugate (three.js r128 `invert` on a unit quaternion).
    pub fn inv(self) -> Quat {
        Quat { x: -self.x, y: -self.y, z: -self.z, w: self.w }
    }
    pub fn normalize(self) -> Quat {
        let l = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if l == 0.0 {
            Quat::IDENTITY
        } else {
            let k = 1.0 / l;
            Quat { x: self.x * k, y: self.y * k, z: self.z * k, w: self.w * k }
        }
    }
    /// three.js slerp, including its shortest-arc and near-parallel branches.
    pub fn slerp(self, qb: Quat, t: f64) -> Quat {
        if t == 0.0 {
            return self;
        }
        if t == 1.0 {
            return qb;
        }
        let (x, y, z, w) = (self.x, self.y, self.z, self.w);
        let mut cos_half = w * qb.w + x * qb.x + y * qb.y + z * qb.z;
        let mut r = if cos_half < 0.0 {
            cos_half = -cos_half;
            Quat { x: -qb.x, y: -qb.y, z: -qb.z, w: -qb.w }
        } else {
            qb
        };
        if cos_half >= 1.0 {
            return self;
        }
        let sqr_sin = 1.0 - cos_half * cos_half;
        if sqr_sin <= f64::EPSILON {
            let s = 1.0 - t;
            r = Quat { w: s * w + t * r.w, x: s * x + t * r.x, y: s * y + t * r.y, z: s * z + t * r.z };
            return r.normalize();
        }
        let sin_half = sqr_sin.sqrt();
        let half = sin_half.atan2(cos_half);
        let ra = ((1.0 - t) * half).sin() / sin_half;
        let rb = (t * half).sin() / sin_half;
        Quat { w: w * ra + r.w * rb, x: x * ra + r.x * rb, y: y * ra + r.y * rb, z: z * ra + r.z * rb }
    }
}

/// Quaternion product a * b (three.js `multiplyQuaternions(a, b)`).
impl Mul for Quat {
    type Output = Quat;
    fn mul(self, b: Quat) -> Quat {
        let a = self;
        Quat {
            x: a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y,
            y: a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
            z: a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        }
    }
}

pub fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    lo.max(hi.min(v))
}
pub fn smooth01(x: f64) -> f64 {
    let x = clamp(x, 0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}
pub fn wrap(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}
/// Heading of the body nose (-z) projected on the ground, radians, 0 = facing away from the pilot.
pub fn heading_of(q: Quat) -> f64 {
    let f = v3(0.0, 0.0, -1.0).rot(q);
    (-f.x).atan2(-f.z)
}
pub fn up_of(q: Quat) -> V3 {
    Y.rot(q)
}
/// Level horizontal forward and right unit vectors for an attitude (forward defaults to -z when vertical).
pub fn fwd_right(q: Quat) -> (V3, V3) {
    let mut f = v3(0.0, 0.0, -1.0).rot(q);
    f.y = 0.0;
    if f.length_sq() < 1e-4 {
        f = v3(0.0, 0.0, -1.0);
    }
    let f = f.normalize();
    (f, f.cross(Y))
}

/// Rotation vector (axis * angle, shortest arc) of a unit quaternion.
pub fn rotvec(q: Quat) -> V3 {
    let q = if q.w < 0.0 { Quat { x: -q.x, y: -q.y, z: -q.z, w: -q.w } } else { q };
    let s = (q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    if s < 1e-9 {
        return v3(2.0 * q.x, 2.0 * q.y, 2.0 * q.z);
    }
    let ang = 2.0 * s.atan2(q.w);
    v3(q.x, q.y, q.z) * (ang / s)
}

/// Attitude from a body up vector (world) and a heading: the nose is the heading direction
/// projected into the plane normal to `up` (works upright and inverted).
pub fn quat_up_heading(up: V3, heading: f64) -> Quat {
    quat_up_nose(up, v3(-heading.sin(), 0.0, -heading.cos()))
}

/// Attitude from a body up vector and a nose hint (both world): the nose is the hint projected
/// into the plane normal to `up` (any direction, also vertical: loops and stall turns).
pub fn quat_up_nose(up: V3, nose_hint: V3) -> Quat {
    let u = up.normalize();
    let f0 = nose_hint;
    let mut nose = f0 - u * f0.dot(u);
    if nose.length_sq() < 1e-8 {
        nose = v3(0.0, 0.0, -1.0) - u * (-u.z);
    }
    if nose.length_sq() < 1e-8 {
        nose = v3(1.0, 0.0, 0.0) - u * u.x;
    }
    let nose = nose.normalize();
    let right = nose.cross(u);
    let back = -nose;
    // rotation matrix columns: right, up, back (three.js setFromRotationMatrix)
    let (m11, m12, m13) = (right.x, u.x, back.x);
    let (m21, m22, m23) = (right.y, u.y, back.y);
    let (m31, m32, m33) = (right.z, u.z, back.z);
    let tr = m11 + m22 + m33;
    let q = if tr > 0.0 {
        let s = 0.5 / (tr + 1.0).sqrt();
        Quat { w: 0.25 / s, x: (m32 - m23) * s, y: (m13 - m31) * s, z: (m21 - m12) * s }
    } else if m11 > m22 && m11 > m33 {
        let s = 2.0 * (1.0 + m11 - m22 - m33).sqrt();
        Quat { w: (m32 - m23) / s, x: 0.25 * s, y: (m12 + m21) / s, z: (m13 + m31) / s }
    } else if m22 > m33 {
        let s = 2.0 * (1.0 + m22 - m11 - m33).sqrt();
        Quat { w: (m13 - m31) / s, x: (m12 + m21) / s, y: 0.25 * s, z: (m23 + m32) / s }
    } else {
        let s = 2.0 * (1.0 + m33 - m11 - m22).sqrt();
        Quat { w: (m21 - m12) / s, x: (m13 + m31) / s, y: (m23 + m32) / s, z: 0.25 * s }
    };
    q.normalize()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rotation_matches_three() {
        let q = Quat::axis_angle(Y, core::f64::consts::FRAC_PI_2);
        let v = v3(0.0, 0.0, -1.0).rot(q);
        assert!((v.x + 1.0).abs() < 1e-12 && v.z.abs() < 1e-12);
        assert!((heading_of(q) - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }
    #[test]
    fn up_heading_roundtrip() {
        for &(u, h) in &[(v3(0.0, 1.0, 0.0), 0.7), (v3(0.0, -1.0, 0.0), -2.0), (v3(0.3, 0.9, -0.2), 1.2), (v3(0.2, -0.95, 0.1), 3.0)] {
            let q = quat_up_heading(u, h);
            let uu = up_of(q);
            assert!((uu - u.normalize()).length() < 1e-9, "{uu:?}");
            // nose in the plane through `up` and the heading direction, pointing along the heading
            let nose = v3(0.0, 0.0, -1.0).rot(q);
            let f0 = v3(-h.sin(), 0.0, -h.cos());
            assert!(nose.dot(uu).abs() < 1e-9 && nose.dot(f0) > 0.0 && nose.dot(f0.cross(uu)).abs() < 1e-9);
            if u.x == 0.0 && u.z == 0.0 {
                assert!(wrap(heading_of(q) - h).abs() < 1e-9 || u.y < 0.0);
            }
        }
        let q = Quat::axis_angle(v3(1.0, 0.0, 0.0), 2.5);
        assert!((rotvec(q) - v3(2.5, 0.0, 0.0)).length() < 1e-9);
    }
}
