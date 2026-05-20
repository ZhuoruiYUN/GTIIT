#!/usr/bin/env python3
"""
record_zero_drift.py — zero-drift & noise analyser with before/after comparison.

Arduino must output:  W,<filtered_g>,<raw_cal_g>
(raw_cal = after calibration poly, before median+EMA filter)

Plots:
  1. Time-series overlay: raw vs filtered, with trend lines
  2. Histogram comparison: raw vs filtered distributions
  3. Rolling 20-s std-dev: shows whether noise is stationary

Printed stats: mean, std, peak-to-peak, drift slope — for both channels.

Usage:
  python record_zero_drift.py
  python record_zero_drift.py --duration 120 --warmup 5
"""

import argparse
import csv
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from scipy import stats as sp_stats

DEFAULT_PORT     = "/dev/tty.usbmodem31301"
DEFAULT_BAUD     = 115200
DEFAULT_DURATION = 120.0
DEFAULT_WARMUP   = 5.0

GREEN  = "#1D9E75"
RED    = "#A32D2D"
AMBER  = "#BA7517"
GREY   = "#888888"


# ── Serial recording ───────────────────────────────────────────────────────────

def record(port: str, baud: int, duration: float, warmup: float):
    """Returns list of (t, filtered_g, raw_cal_g)."""
    samples = []
    with serial.Serial(port, baud, timeout=2) as ser:
        print(f"  Warming up ({warmup:.0f} s) …", flush=True)
        deadline = time.time() + warmup
        while time.time() < deadline:
            ser.readline()

        print(f"  Recording {duration:.0f} s — keep scale EMPTY …", flush=True)
        t0 = time.time()
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("W,"):
                continue
            parts = line.split(",")
            try:
                filt = float(parts[1])
                raw  = float(parts[2]) if len(parts) > 2 else filt
            except (ValueError, IndexError):
                continue
            t = time.time() - t0
            samples.append((t, filt, raw))
            print(f"\r  {t:6.1f}/{duration:.0f} s  raw={raw:+7.3f} g  filtered={filt:+7.3f} g",
                  end="", flush=True)
            if t >= duration:
                break
    print()
    return samples


# ── Statistics ─────────────────────────────────────────────────────────────────

def stats(t: np.ndarray, g: np.ndarray, label: str) -> dict:
    mean  = float(np.mean(g))
    std   = float(np.std(g, ddof=1))
    p2p   = float(np.ptp(g))
    slope, intercept, r, _, _ = sp_stats.linregress(t, g)
    dt    = float(np.median(np.diff(t))) if len(t) > 1 else 0.1
    win   = max(1, int(20.0 / dt))
    roll  = np.array([float(np.std(g[max(0, i - win):i + 1], ddof=0)) for i in range(len(g))])
    return dict(label=label, mean=mean, std=std, p2p=p2p,
                slope=slope, intercept=intercept, r=r, roll=roll, dt=dt)


def print_stats(s: dict):
    print(f"  [{s['label']}]")
    print(f"    mean          : {s['mean']:+.4f} g")
    print(f"    std (noise)   : {s['std']:.4f} g")
    print(f"    peak-to-peak  : {s['p2p']:.4f} g")
    print(f"    drift slope   : {s['slope']*60:+.4f} g/min   (r={s['r']:.3f})")


# ── Plot ────────────────────────────────────────────────────────────────────────

def plot(t: np.ndarray, raw: np.ndarray, filt: np.ndarray,
         s_raw: dict, s_filt: dict, out_png: Path):

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("Zero-drift & noise: raw vs filtered", fontweight="bold")

    # ── 1. Time-series overlay ─────────────────────────────
    ax = axes[0]
    ax.plot(t, raw,  color=GREY,  linewidth=0.8, alpha=0.7, label=f"raw   σ={s_raw['std']:.3f} g")
    ax.plot(t, filt, color=GREEN, linewidth=1.4, label=f"filtered  σ={s_filt['std']:.3f} g")

    # trend lines
    tr_raw  = s_raw['intercept']  + s_raw['slope']  * t
    tr_filt = s_filt['intercept'] + s_filt['slope'] * t
    ax.plot(t, tr_raw,  color=GREY,  linewidth=1.0, linestyle="--", alpha=0.6)
    ax.plot(t, tr_filt, color=RED,   linewidth=1.0, linestyle="--",
            label=f"trend {s_filt['slope']*60:+.4f} g/min")

    ax.axhline(0, color="#222", linewidth=0.6, linestyle=":")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Weight (g)")
    ax.set_title("Time series")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.2)

    # ── 2. Histogram comparison ────────────────────────────
    ax = axes[1]
    lo = min(raw.min(), filt.min())
    hi = max(raw.max(), filt.max())
    bins = np.linspace(lo, hi, 40)

    ax.hist(raw,  bins=bins, density=True, alpha=0.45, color=GREY,  label="raw",      edgecolor="white")
    ax.hist(filt, bins=bins, density=True, alpha=0.60, color=GREEN, label="filtered", edgecolor="white")

    xg = np.linspace(lo, hi, 300)
    ax.plot(xg, sp_stats.norm.pdf(xg, s_raw['mean'],  s_raw['std']),
            color=GREY, linewidth=1.5, linestyle="--")
    ax.plot(xg, sp_stats.norm.pdf(xg, s_filt['mean'], s_filt['std']),
            color=RED,  linewidth=1.8)

    noise_reduction = (1 - s_filt['std'] / s_raw['std']) * 100 if s_raw['std'] > 0 else 0
    ax.set_xlabel("Weight (g)")
    ax.set_ylabel("Density")
    ax.set_title(f"Histogram   noise ↓{noise_reduction:.1f}%\n"
                 f"raw σ={s_raw['std']:.3f} g  →  filtered σ={s_filt['std']:.3f} g")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.2)

    # ── 3. Rolling std-dev ────────────────────────────────
    ax = axes[2]
    ax.plot(t, s_raw['roll'],  color=GREY,  linewidth=0.9, alpha=0.7, label="raw rolling σ")
    ax.plot(t, s_filt['roll'], color=GREEN, linewidth=1.3, label="filtered rolling σ")
    ax.axhline(s_raw['std'],  color=GREY, linewidth=1.0, linestyle="--", alpha=0.6)
    ax.axhline(s_filt['std'], color=RED,  linewidth=1.0, linestyle="--",
               label=f"global σ filtered = {s_filt['std']:.3f} g")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rolling σ (g)  [20 s window]")
    ax.set_title("Noise stationarity")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.2)

    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    print(f"\n  Plot saved → {out_png}")
    plt.show()


# ── Main ────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Zero-drift & noise comparison: raw vs filtered")
    ap.add_argument("--port",     default=DEFAULT_PORT)
    ap.add_argument("--baud",     type=int,   default=DEFAULT_BAUD)
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    ap.add_argument("--warmup",   type=float, default=DEFAULT_WARMUP)
    ap.add_argument("--out",      default="zero_drift.csv")
    args = ap.parse_args()

    print("=" * 58)
    print("  Zero-drift comparison: raw (pre-filter) vs filtered")
    print("=" * 58)
    print(f"  Port: {args.port}   Baud: {args.baud}")
    print(f"  Duration: {args.duration:.0f} s   Warmup: {args.warmup:.0f} s")
    print("  Scale must be EMPTY. Place on a stable, vibration-free surface.")
    input("  Press Enter to start …\n")

    samples = record(args.port, args.baud, args.duration, args.warmup)

    if len(samples) < 10:
        raise SystemExit("Too few samples received — check serial connection.")

    # save CSV
    out_path = Path(args.out)
    with out_path.open("w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_s", "filtered_g", "raw_cal_g"])
        wr.writerows(samples)
    print(f"  Saved {len(samples)} samples → {out_path}")

    t    = np.array([r[0] for r in samples])
    filt = np.array([r[1] for r in samples])
    raw  = np.array([r[2] for r in samples])

    s_raw  = stats(t, raw,  "raw (pre-filter)")
    s_filt = stats(t, filt, "filtered")

    print()
    print("─" * 58)
    print("  Noise & drift summary")
    print("─" * 58)
    print_stats(s_raw)
    print()
    print_stats(s_filt)

    noise_reduction = (1 - s_filt['std'] / s_raw['std']) * 100 if s_raw['std'] > 0 else 0
    print()
    print(f"  Noise reduction : {noise_reduction:.1f}%")

    if abs(s_filt['slope']) > 0.005:
        rate = s_filt['slope'] * 60
        print(f"  ⚠  Drift detected: {rate:+.4f} g/min after filtering.")
        print("     Consider longer warm-up or mechanical isolation.")
    else:
        print("  ✓  No significant drift after filtering.")
    print("─" * 58)

    out_png = out_path.with_suffix(".png")
    plot(t, raw, filt, s_raw, s_filt, out_png)


if __name__ == "__main__":
    main()

# run: python record_zero_drift.py --duration 120 --warmup 5 --out zero_drift.csv
