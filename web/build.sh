#!/usr/bin/env bash
# Build rc-heli-field.html: Rust core -> wasm32 (std built from source), wasm-opt -Oz,
# TypeScript type check, esbuild bundle (iife), then embed gzip+base64 wasm and the bundle
# into page.html at the /*WASM*/ and /*BUNDLE*/ placeholders.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
OUT="${1:-$ROOT/rc-heli-field.html}"
BUILD="$HERE/build"
mkdir -p "$BUILD"

echo "== rust -> wasm32"
cd "$ROOT/rust"
if rustup target list --installed 2>/dev/null | grep -q wasm32-unknown-unknown; then
  cargo build --profile wasm --lib --target wasm32-unknown-unknown
else
  # the wasm32 std can't be downloaded here: build it from rust-src (see README)
  RUSTC_BOOTSTRAP=1 cargo build --profile wasm --lib --target wasm32-unknown-unknown -Zbuild-std=std,panic_abort
fi
WASM="$ROOT/rust/target/wasm32-unknown-unknown/wasm/helisim.wasm"
cp "$WASM" "$BUILD/helisim.wasm"

cd "$HERE"
[ -d node_modules ] || npm install --no-audit --no-fund
if [ -x node_modules/.bin/wasm-opt ]; then
  echo "== wasm-opt -Oz"
  node_modules/.bin/wasm-opt -Oz --enable-bulk-memory --enable-nontrapping-float-to-int --enable-sign-ext --enable-mutable-globals \
    "$BUILD/helisim.wasm" -o "$BUILD/helisim.opt.wasm"
else
  echo "== wasm-opt not available, skipping"
  cp "$BUILD/helisim.wasm" "$BUILD/helisim.opt.wasm"
fi

echo "== tsc"
node_modules/.bin/tsc -p tsconfig.json
echo "== esbuild"
node_modules/.bin/esbuild src/main.ts --bundle --format=iife --target=es2020 --minify --outfile="$BUILD/bundle.js" --log-level=warning

echo "== embed"
python3 - "$HERE/page.html" "$BUILD/helisim.opt.wasm" "$BUILD/bundle.js" "$OUT" <<'PY'
import base64, gzip, sys
page, wasm, bundle, out = sys.argv[1:5]
html = open(page, encoding='utf-8').read()
w = base64.b64encode(gzip.compress(open(wasm, 'rb').read(), 9, mtime=0)).decode()
js = open(bundle, encoding='utf-8').read().replace('</script', '<\\/script')
assert html.count('/*WASM*/') == 1 and html.count('/*BUNDLE*/') == 1
html = html.replace('/*WASM*/', w).replace('/*BUNDLE*/', js)
open(out, 'w', encoding='utf-8').write(html)
print(f"wasm {len(open(wasm,'rb').read())} B, gz+b64 {len(w)} B, bundle {len(js)} B, page {len(html.encode())} B -> {out}")
PY
