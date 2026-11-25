#!/usr/bin/env python3
"""Salin fungsi firmware APA ADANYA dari sebuah .cpp ke file baru untuk tes host (A.5.6, tanpa fork).

motors.cpp bergantung perangkat keras (LEDC, ISR encoder, Preferences), jadi tidak bisa dikompilasi di host (HAL-nya Fase 2).
kinInverse/kinForward murni matematika; teksnya disalin tiap build, sehingga tes selalu menguji kode firmware terkini.
Gagal keras (exit 1) bila tanda tangan tidak ditemukan: tidak ada skip diam-diam.
"""
import argparse, hashlib, sys


def extract(src, sig):
    i = src.find(sig)
    if i < 0:
        sys.exit(f"extract_funcs: '{sig}' tidak ditemukan")
    j = src.index("{", i)
    depth = 0
    for k in range(j, len(src)):
        depth += src[k] == "{"
        depth -= src[k] == "}"
        if depth == 0:
            return src[i:k + 1]
    sys.exit(f"extract_funcs: fungsi '{sig}' tidak lengkap")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--sig", action="append", required=True)
    ap.add_argument("--include", action="append", default=[])
    a = ap.parse_args()
    text = open(a.src, encoding="utf-8").read()
    sha = hashlib.sha256(text.encode()).hexdigest()[:12]
    body = "\n\n".join(extract(text, s) for s in a.sig)
    inc = "".join(f'#include "{h}"\n' for h in a.include)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(f"// DIHASILKAN oleh host_test/extract_funcs.py dari {a.src} (sha256 {sha}). Jangan diedit; isi fungsi disalin apa adanya.\n")
        f.write(inc + "\n" + body + "\n")
    print(f"extract_funcs: {len(a.sig)} fungsi dari {a.src} -> {a.out}")


if __name__ == "__main__":
    main()
