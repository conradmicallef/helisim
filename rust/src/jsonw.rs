//! Tiny JSON writer for the per-frame state, plus `to_fixed` (JS `Number.prototype.toFixed`
//! style) for HUD, cue and toast text. Avoids serde_json and core::fmt float formatting, which
//! together would add most of the wasm binary.

/// Format `x` with `d` decimals, JS `toFixed` style (half rounds up in magnitude, "-0.0" kept for
/// small negatives).
pub fn to_fixed(x: f64, d: u32) -> String {
    let mut s = String::new();
    push_fixed(&mut s, x, d);
    s
}

pub fn push_fixed(s: &mut String, x: f64, d: u32) {
    if !x.is_finite() {
        s.push_str(if x.is_nan() { "NaN" } else if x > 0.0 { "Infinity" } else { "-Infinity" });
        return;
    }
    let neg = x < 0.0;
    let mut scale = 1u64;
    for _ in 0..d {
        scale *= 10;
    }
    let m = x.abs() * scale as f64;
    // tiny bias so values like 1.005 (stored as 1.00499..) still round the way a reader expects
    let r = if m < 9.0e15 { (m + 0.5 + m * 1e-15).floor() as u64 } else { m as u64 };
    if neg {
        s.push('-');
    }
    let ip = r / scale;
    push_u64(s, ip);
    if d > 0 {
        s.push('.');
        let fp = r % scale;
        let mut div = scale / 10;
        while div > 0 {
            s.push((b'0' + ((fp / div) % 10) as u8) as char);
            div /= 10;
        }
    }
}

pub fn push_u64(s: &mut String, mut v: u64) {
    let mut buf = [0u8; 20];
    let mut i = buf.len();
    if v == 0 {
        s.push('0');
        return;
    }
    while v > 0 {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    for &b in &buf[i..] {
        s.push(b as char);
    }
}

/// JS `Math.round`: halves round toward +infinity.
pub fn js_round(x: f64) -> f64 {
    (x + 0.5).floor()
}
/// `Math.round(x)` printed as an integer.
pub fn round_str(x: f64) -> String {
    let r = js_round(x);
    let mut s = String::new();
    if r < 0.0 {
        s.push('-');
    }
    push_u64(&mut s, r.abs() as u64);
    s
}

/// Streaming JSON object/array writer. Commas are inserted automatically.
pub struct Json {
    pub buf: String,
    first: bool,
}

impl Default for Json {
    fn default() -> Self {
        Json::new()
    }
}

impl Json {
    pub fn new() -> Json {
        Json { buf: String::with_capacity(4096), first: true }
    }
    pub fn clear(&mut self) {
        self.buf.clear();
        self.first = true;
    }
    fn sep(&mut self) {
        if !self.first {
            self.buf.push(',');
        }
        self.first = false;
    }
    fn key(&mut self, k: &str) {
        self.sep();
        self.buf.push('"');
        self.buf.push_str(k);
        self.buf.push_str("\":");
    }
    pub fn obj(&mut self, k: &str) {
        if k.is_empty() {
            self.sep();
        } else {
            self.key(k);
        }
        self.buf.push('{');
        self.first = true;
    }
    pub fn end_obj(&mut self) {
        self.buf.push('}');
        self.first = false;
    }
    pub fn arr(&mut self, k: &str) {
        self.key(k);
        self.buf.push('[');
        self.first = true;
    }
    pub fn end_arr(&mut self) {
        self.buf.push(']');
        self.first = false;
    }
    /// Number with up to 5 decimals (enough for rendering, keeps frames short).
    pub fn num(&mut self, k: &str, v: f64) {
        self.key(k);
        self.raw_num(v);
    }
    /// Bare number as the next array item.
    pub fn raw_num_item(&mut self, v: f64) {
        self.sep();
        self.raw_num(v);
    }
    pub fn raw_num(&mut self, v: f64) {
        if !v.is_finite() {
            self.buf.push('0');
            return;
        }
        let mut t = String::new();
        push_fixed(&mut t, v, 5);
        // trim trailing zeros
        let t = if t.contains('.') { t.trim_end_matches('0').trim_end_matches('.').to_string() } else { t };
        self.buf.push_str(if t == "-0" { "0" } else { &t });
    }
    pub fn int(&mut self, k: &str, v: i64) {
        self.key(k);
        if v < 0 {
            self.buf.push('-');
        }
        push_u64(&mut self.buf, v.unsigned_abs());
    }
    pub fn bool(&mut self, k: &str, v: bool) {
        self.key(k);
        self.buf.push_str(if v { "true" } else { "false" });
    }
    pub fn str(&mut self, k: &str, v: &str) {
        if k.is_empty() {
            self.sep();
        } else {
            self.key(k);
        }
        self.buf.push('"');
        for c in v.chars() {
            match c {
                '"' => self.buf.push_str("\\\""),
                '\\' => self.buf.push_str("\\\\"),
                '\n' => self.buf.push_str("\\n"),
                c if (c as u32) < 0x20 => {
                    self.buf.push_str("\\u00");
                    let h = b"0123456789abcdef";
                    self.buf.push(h[(c as usize) >> 4] as char);
                    self.buf.push(h[(c as usize) & 15] as char);
                }
                c => self.buf.push(c),
            }
        }
        self.buf.push('"');
    }
    /// Comma before the next array item (for hand-written items).
    pub fn sep_item(&mut self) {
        self.sep();
    }
    /// A bare number without a separator.
    pub fn buf_num(&mut self, v: f64) {
        let f = self.first;
        self.raw_num(v);
        self.first = f;
    }
    pub fn v3(&mut self, k: &str, v: crate::math::V3) {
        self.arr(k);
        for x in [v.x, v.y, v.z] {
            self.sep();
            self.raw_num(x);
        }
        self.end_arr();
    }
    pub fn quat(&mut self, k: &str, q: crate::math::Quat) {
        self.arr(k);
        for x in [q.x, q.y, q.z, q.w] {
            self.sep();
            self.raw_num(x);
        }
        self.end_arr();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fixed() {
        assert_eq!(to_fixed(-0.04, 1), "-0.0");
        assert_eq!(to_fixed(0.0, 1), "0.0");
        assert_eq!(to_fixed(3.0, 1), "3.0");
        assert_eq!(to_fixed(0.345, 2), "0.35");
        assert_eq!(to_fixed(14.25, 1), "14.3");
        assert_eq!(to_fixed(12.0, 0), "12");
        assert_eq!(round_str(44.5), "45");
        assert_eq!(round_str(-0.4), "0");
    }
}
