#!/usr/bin/env python3
"""Regenerate the README figures from real captures and the firmware constants.

Inputs:
  codes/samsung_tv.txt   frames captured by this firmware from a Samsung TV remote
  src/main.c             SAMSUNG_* timing constants used by samsung_frame()
The decoder below is a line-by-line port of decode_pd()/near() in src/ir_rx.c.

    python3 docs/make_figures.py        # needs numpy + matplotlib
"""
import os
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "img")


def consts():
    src = open(os.path.join(ROOT, "src", "main.c")).read()
    return {k: int(v, 0) for k, v in re.findall(r"#define\s+(SAMSUNG_\w+)\s+(0x[0-9A-Fa-f]+|\d+)", src)}


def captures():
    frames, name = [], None
    for line in open(os.path.join(ROOT, "codes", "samsung_tv.txt")):
        line = line.strip()
        if line.startswith("# ") and ":" in line:
            name = line[2:].split(":")[0]
        elif line.startswith("TX "):
            parts = line.split()
            frames.append((name, int(parts[1]), [int(x) for x in parts[2:]]))
    return frames


def samsung_frame(c, cmd):
    a = c["SAMSUNG_ADDR"]
    v = a | (a << 8) | (cmd << 16) | ((~cmd & 0xFF) << 24)
    d = [c["SAMSUNG_HDR"], c["SAMSUNG_HDR"]]
    for i in range(32):
        d += [c["SAMSUNG_BIT"], c["SAMSUNG_ONE"] if v >> i & 1 else c["SAMSUNG_ZERO"]]
    return d + [c["SAMSUNG_BIT"], c["SAMSUNG_GAP"]]


def near(v, t):
    return t * 3 // 4 < v < t * 5 // 4


def decode_pd(d, start, mark, one, zero):
    val, bits, i = 0, 0, start
    while i + 1 < len(d) and bits < 32:
        if not near(d[i], mark):
            break
        if near(d[i + 1], one):
            val |= 1 << bits
        elif not near(d[i + 1], zero):
            break
        bits += 1
        i += 2
    return bits, val


def main():
    os.makedirs(OUT, exist_ok=True)
    c = consts()
    caps = captures()
    marks, zeros, ones, hdr = [], [], [], []
    for name, khz, d in caps:
        bits, v = decode_pd(d, 2, 560, 1690, 560)
        ok = bits == 32 and near(d[0], 4500) and near(d[1], 4500)
        print("%-9s %d durations, decoded %d bits -> addr=0x%04X cmd=0x%02X %s"
              % (name, len(d), bits, v & 0xFFFF, (v >> 16) & 0xFF, "OK" if ok else "FAIL"))
        hdr += d[0:2]
        for i in range(2, 66, 2):
            marks.append(d[i])
            (ones if near(d[i + 1], 1690) else zeros).append(d[i + 1])
        marks.append(d[66])

    def stat(label, xs, nominal):
        xs = np.array(xs)
        err = (xs - nominal) / nominal * 100
        print("%-12s n=%3d mean=%7.1f us  nominal=%4d  error mean %+5.1f%%  range [%+5.1f%%, %+5.1f%%]"
              % (label, len(xs), xs.mean(), nominal, err.mean(), err.min(), err.max()))

    stat("header", hdr, 4500)
    stat("bit mark", marks, 560)
    stat("space '0'", zeros, 560)
    stat("space '1'", ones, 1690)

    # Figure 1: captured POWER frame vs the frame samsung_frame() synthesises.
    name, _, cap = caps[0]
    nom = samsung_frame(c, 0x02)

    def wave(d):
        t, x, y = 0, [0], [0]
        for i, dur in enumerate(d):
            lvl = 1 if i % 2 == 0 else 0
            x += [t, t + dur]
            y += [lvl, lvl]
            t += dur
        return np.array(x) / 1000.0, np.array(y)

    fig, ax = plt.subplots(figsize=(9, 2.8))
    x, y = wave(nom[:-1])
    ax.plot(x, y + 1.3, color="#264653", lw=1, label="samsung_frame(0x02), synthesised")
    x, y = wave(cap[:-1])
    ax.plot(x, y, color="#e76f51", lw=1, label="captured from the remote (%s)" % name)
    ax.set_yticks([])
    ax.set_xlabel("time (ms), high = carrier on (mark)")
    ax.set_title("Samsung32 POWER: synthesised vs captured envelope")
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "samsung_power_frame.png"), dpi=120)
    plt.close(fig)

    # Figure 2: measured durations against the decoder acceptance windows.
    fig, ax = plt.subplots(figsize=(9, 3))
    bins = np.arange(300, 2200, 10)
    ax.hist(marks, bins=bins, color="#2a9d8f", alpha=0.8, label="marks (n=%d)" % len(marks))
    ax.hist(zeros, bins=bins, color="#e9c46a", alpha=0.8, label="'0' spaces (n=%d)" % len(zeros))
    ax.hist(ones, bins=bins, color="#e76f51", alpha=0.8, label="'1' spaces (n=%d)" % len(ones))
    for t in (560, 1690):
        ax.axvspan(t * 3 // 4, t * 5 // 4, color="#999", alpha=0.15)
        ax.axvline(t, color="#555", lw=0.8, ls="--")
    ax.set_xlabel("duration (us); grey = near() window (0.75 t, 1.25 t)")
    ax.set_ylabel("count")
    ax.set_title("Captured Samsung32 timings (3 frames) vs nominal and decoder tolerance")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "samsung_timing_hist.png"), dpi=120)
    plt.close(fig)
    print("figures written to", OUT)


if __name__ == "__main__":
    main()
