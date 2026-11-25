#!/usr/bin/env python3
"""A.5.2: regenerasi firmware_export.json dan kin_golden.json dari firmware SAAT INI, bandingkan dengan salinan di omni_sim.
Gagal bila beda (golden basi) atau bila golden bukan 'firmware-compiled'. Toleransi float 1e-4 (libm beda antar mesin)."""
import argparse, json, os, subprocess, sys, tempfile


def close(a, b, tol=1e-4):
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(close(a[k], b[k], tol) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        return abs(a - b) <= tol
    return a == b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--firmware-dir", required=True)
    ap.add_argument("--sim-pkg", required=True)
    a = ap.parse_args()
    tool = os.path.join(a.sim_pkg, "tools", "export_firmware_config.py")
    cur = {"export": os.path.join(a.sim_pkg, "config", "firmware_export.json"),
           "golden": os.path.join(a.sim_pkg, "test", "kin_golden.json")}
    ok = True
    with tempfile.TemporaryDirectory() as t:
        fresh = {"export": os.path.join(t, "e.json"), "golden": os.path.join(t, "g.json")}
        r = subprocess.run([sys.executable, tool, "--firmware-dir", a.firmware_dir,
                            "--out-export", fresh["export"], "--out-golden", fresh["golden"]], capture_output=True, text=True)
        print(r.stdout.strip())
        if r.returncode:
            print("GAGAL: export_firmware_config.py:", r.stderr.strip()); sys.exit(1)
        for k in ("export", "golden"):
            if not os.path.exists(cur[k]):
                print(f"GAGAL: {cur[k]} tidak ada"); ok = False; continue
            same = close(json.load(open(cur[k])), json.load(open(fresh[k])))
            print(f"  {k}: {'sama dengan firmware' if same else 'BASI / BEDA dari firmware'}  ({os.path.relpath(cur[k])})")
            ok &= same
        g = json.load(open(fresh["golden"]))
        if g.get("source") != "firmware-compiled":
            print(f"GAGAL: golden dibuat dari '{g.get('source')}', bukan kompilasi firmware"); ok = False
    print("kin_golden: LULUS" if ok else "kin_golden: GAGAL (jalankan tools/export_firmware_config.py --firmware-dir <omni3x>)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
