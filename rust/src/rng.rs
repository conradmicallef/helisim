//! Seeded random numbers (replaces Math.random so runs are reproducible).
//! xoshiro256** seeded through splitmix64; `randn` is Box-Muller exactly as the JS page did it.

#[derive(Clone, Debug)]
pub struct Rng {
    s: [u64; 4],
}

fn splitmix(x: &mut u64) -> u64 {
    *x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        let mut x = seed;
        Rng { s: [splitmix(&mut x), splitmix(&mut x), splitmix(&mut x), splitmix(&mut x)] }
    }
    fn next_u64(&mut self) -> u64 {
        let s = &mut self.s;
        let r = s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        r
    }
    /// Uniform in [0, 1).
    pub fn f(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }
    /// Uniform in [-1, 1).
    pub fn rnd1(&mut self) -> f64 {
        self.f() * 2.0 - 1.0
    }
    /// Standard normal (Box-Muller, cosine branch).
    pub fn randn(&mut self) -> f64 {
        let mut u = 0.0;
        while u == 0.0 {
            u = self.f();
        }
        (-2.0 * u.ln()).sqrt() * (2.0 * core::f64::consts::PI * self.f()).cos()
    }
}
