#!/usr/bin/env python3
"""
Record and plot a step response: place and remove a known weight.
Shows raw and filtered signals with rise/settle annotations.

Detection logic:
    - Place event  : first raw sample > place_threshold (default 5 g)
    - Remove event : first sustained raw drop below remove_threshold (default 495 g)
                                     after raw has been above that threshold for top_hold_s

Expected serial lines: W,<grams>
"""

import argparse
import csv
import threading
import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import serial

DEFAULT_PORT = "/dev/tty.usbmodem31301"
DEFAULT_BAUD = 115200


# ---------------------------------------------------------------------------
# Serial reader
# ---------------------------------------------------------------------------

def serial_reader(port, baud, t0, data_t, data_g, lock, stop_flag):
    with serial.Serial(port, baud, timeout=1) as ser:
        while not stop_flag[0]:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("W,"):
                continue
            try:
                grams = float(line.split(",")[1])
            except ValueError:
                continue
            ts = time.time() - t0
            with lock:
                data_t.append(ts)
                data_g.append(grams)


# ---------------------------------------------------------------------------
# Signal processing
# ---------------------------------------------------------------------------

def ema_filter(x, alpha):
    y = np.empty_like(x, dtype=float)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1.0 - alpha) * y[i - 1]
    return y


def detect_place_time(t, raw, place_threshold):
    """
    Place event = first raw sample that exceeds place_threshold.
    This is the TRUE moment the weight touches the scale.
    """
    idx = np.where(raw > place_threshold)[0]
    if idx.size == 0:
        return None
    return float(t[idx[0]])


def first_sustained_above(t, y, threshold, hold_s, start_idx=0):
    dt = float(np.median(np.diff(t)))
    hold_n = max(1, int(round(hold_s / dt)))
    for i in range(start_idx, len(y) - hold_n):
        if np.all(y[i:i + hold_n] >= threshold):
            return i
    return None


def first_sustained_below(t, y, threshold, hold_s, start_idx=0):
    dt = float(np.median(np.diff(t)))
    hold_n = max(1, int(round(hold_s / dt)))
    for i in range(start_idx, len(y) - hold_n):
        if np.all(y[i:i + hold_n] <= threshold):
            return i
    return None


def detect_remove_time(t, raw, remove_threshold, t_place, top_hold_s, drop_hold_s):
    """
    Remove event = first RAW sample that drops below remove_threshold,
    BUT only after the FILTERED signal has reached step_weight * top_band.

    Using filt to confirm "at top" avoids false triggers during the noisy
    raw rise. Using raw for the actual drop gives earliest true detection.
    """
    if t_place is None:
        return None

    start_idx = int(np.searchsorted(t, t_place))

    # Ensure the raw signal has truly reached the top for some time
    i_top = first_sustained_above(t, raw, remove_threshold, top_hold_s, start_idx)
    if i_top is None:
        return None

    # Then detect a sustained drop below the threshold
    i_drop = first_sustained_below(t, raw, remove_threshold, drop_hold_s, i_top + 1)
    if i_drop is None:
        return None
    return float(t[i_drop])


# ---------------------------------------------------------------------------
# Analysis helpers
# ---------------------------------------------------------------------------

def baseline_before(t, y, t_event, pre_window_s):
    """Mean of filtered signal in the window just before the event."""
    mask = (t >= t_event - pre_window_s) & (t < t_event)
    if np.any(mask):
        return float(np.mean(y[mask]))
    mask = t < t_event
    if np.any(mask):
        return float(np.mean(y[mask]))
    return float(y[0])


def first_crossing(t, y, t0, level, direction):
    """First time >= t0 that y crosses level in the given direction."""
    mask = t >= t0
    t_seg = t[mask]
    y_seg = y[mask]
    if direction > 0:
        idx = np.where(y_seg >= level)[0]
    else:
        idx = np.where(y_seg <= level)[0]
    if idx.size == 0:
        return None
    return float(t_seg[idx[0]])


def settling_time(t, y, t0, target, tol, hold_s):
    """
    Time (relative to t0) for y to stay within ±tol of target
    for at least hold_s seconds continuously.
    """
    if len(t) < 3:
        return None
    dt = float(np.median(np.diff(t)))
    hold_n = max(1, int(round(hold_s / dt)))
    start = int(np.searchsorted(t, t0))
    for i in range(start, len(t) - hold_n):
        window = y[i: i + hold_n]
        if np.all(np.abs(window - target) <= tol):
            return float(t[i] - t0)
    return None


def analyze_place_step(t, filt, t_place, step_weight, pre_window_s, settle_band, hold_s):
    """
    Analyse the rising edge (place).
    baseline : mean before place
    target   : baseline + step_weight
    rise     : 10%→90% time on the FILTERED signal (after t_place)
    settle   : time to stay within settle_band of target
    """
    base = baseline_before(t, filt, t_place, pre_window_s)
    target = base + step_weight
    amp = target - base

    t10 = first_crossing(t, filt, t_place, base + 0.10 * amp, +1)
    t90 = first_crossing(t, filt, t_place, base + 0.90 * amp, +1)
    rise = float(t90 - t10) if (t10 is not None and t90 is not None and t90 >= t10) else None

    tol = abs(amp) * settle_band
    settle = settling_time(t, filt, t_place, target, tol, hold_s)

    return dict(baseline=base, target=target, t10=t10, t90=t90, rise=rise, settle=settle)


def analyze_remove_step(t, filt, t_remove, step_weight, pre_window_s, settle_band, hold_s):
    """
    Analyse the falling edge (remove).
    baseline : mean of filtered signal just before t_remove  (≈ step_weight)
    target   : 0 g  (scale returns to zero)
    rise     : 10%→90% fall time (measured as 90%→10% descent on filtered)
    settle   : time for filtered signal to stay within settle_band of 0
    """
    base = baseline_before(t, filt, t_remove, pre_window_s)
    target = 0.0          # scale should return to zero
    amp = target - base   # negative

    # 10% and 90% of the falling edge
    t10 = first_crossing(t, filt, t_remove, base + 0.10 * amp, -1)   # 90% of full value
    t90 = first_crossing(t, filt, t_remove, base + 0.90 * amp, -1)   # 10% of full value
    rise = float(t90 - t10) if (t10 is not None and t90 is not None and t90 >= t10) else None

    tol = abs(step_weight) * settle_band
    settle = settling_time(t, filt, t_remove, target, tol, hold_s)

    return dict(baseline=base, target=target, t10=t10, t90=t90, rise=rise, settle=settle)


# ---------------------------------------------------------------------------
# CSV
# ---------------------------------------------------------------------------

def save_csv(path: Path, t, raw, filt):
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_s", "raw_g", "filt_g"])
        writer.writerows(zip(t, raw, filt))


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def add_step_annotations(ax, label, t_event, stats, color, y_text):
    """Draw vertical lines and text for one step event."""
    ax.axvline(t_event, color=color, linestyle="--", linewidth=1.4,
               label=f"{label} (raw detect)")
    ax.text(t_event + 0.1, y_text, label, color=color, ha="left", va="top",
            fontsize=9, fontweight="bold")

    t10, t90 = stats["t10"], stats["t90"]
    if t10 is not None:
        ax.axvline(t10, color=color, linestyle=":", linewidth=1.0)
    if t90 is not None:
        ax.axvline(t90, color=color, linestyle=":", linewidth=1.0)

    if stats["rise"] is not None and t90 is not None:
        ax.text(t90 + 0.1, y_text, f"rise {stats['rise']:.2f}s",
                color=color, ha="left", va="top", fontsize=8)

    if stats["settle"] is not None:
        t_settled = t_event + stats["settle"]
        ax.axvline(t_settled, color=color, linestyle="-.", linewidth=1.0)
        ax.text(t_settled + 0.1, y_text, f"settle {stats['settle']:.2f}s",
                color=color, ha="left", va="top", fontsize=8)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--alpha", type=float, default=0.2, help="EMA alpha")
    ap.add_argument("--step-weight", type=float, default=500.0, help="Known weight in g")
    ap.add_argument("--place-threshold", type=float, default=5.0,
                    help="Raw threshold (g) to detect place event")
    ap.add_argument("--remove-threshold", type=float, default=495.0,
                    help="Raw threshold (g) to detect remove event (weight drops below this)")
    ap.add_argument("--pre-window", type=float, default=1.0,
                    help="Baseline window before event (s)")
    ap.add_argument("--top-hold", type=float, default=0.4,
                    help="Min time raw stays above remove-threshold before removal (s)")
    ap.add_argument("--drop-hold", type=float, default=0.2,
                    help="Min time raw stays below remove-threshold to confirm removal (s)")
    ap.add_argument("--settle-band", type=float, default=0.02,
                    help="Settling band as fraction of step weight")
    ap.add_argument("--hold", type=float, default=0.5,
                    help="Consecutive hold time for settling criterion (s)")
    ap.add_argument("--warmup", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=35.0)
    ap.add_argument("--out", default="step_response.csv")
    ap.add_argument("--plot", default="step_response.png")
    args = ap.parse_args()

    if not args.port:
        raise SystemExit("Set DEFAULT_PORT or pass --port.")

    data_t, data_g = [], []
    lock = threading.Lock()
    stop_flag = [False]

    t0 = time.time()
    reader = threading.Thread(
        target=serial_reader,
        args=(args.port, args.baud, t0, data_t, data_g, lock, stop_flag),
        daemon=True,
    )
    reader.start()

    if args.warmup > 0:
        time.sleep(args.warmup)

    print(f"Recording for {args.duration:.1f}s. Place then remove the {args.step_weight:.0f}g weight.")
    time.sleep(args.duration)

    stop_flag[0] = True
    reader.join(timeout=1)

    with lock:
        t_arr = np.array(data_t, dtype=float)
        raw   = np.array(data_g,  dtype=float)

    if t_arr.size < 3:
        raise SystemExit("Not enough data captured.")

    filt = ema_filter(raw, args.alpha)
    save_csv(Path(args.out), t_arr, raw, filt)

    # ------------------------------------------------------------------
    # Detect events on the RAW signal (no filter lag)
    # ------------------------------------------------------------------
    t_place  = detect_place_time(t_arr, raw, args.place_threshold)
    t_remove = detect_remove_time(
        t_arr,
        raw,
        args.remove_threshold,
        t_place,
        args.top_hold,
        args.drop_hold,
    )

    print(f"Place  detected at t = {t_place}")
    print(f"Remove detected at t = {t_remove}")

    # ------------------------------------------------------------------
    # Analyse rise / fall on the FILTERED signal (less noisy)
    # ------------------------------------------------------------------
    place_stats  = None
    remove_stats = None

    if t_place is not None:
        place_stats = analyze_place_step(
            t_arr, filt, t_place,
            args.step_weight, args.pre_window, args.settle_band, args.hold,
        )
        print("Place step:", place_stats)

    if t_remove is not None:
        remove_stats = analyze_remove_step(
            t_arr, filt, t_remove,
            args.step_weight, args.pre_window, args.settle_band, args.hold,
        )
        print("Remove step:", remove_stats)

    # ------------------------------------------------------------------
    # Plot
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t_arr, raw,  color="#aaaaaa", linewidth=1.0, label="Raw")
    ax.plot(t_arr, filt, color="#1D9E75", linewidth=2.0, label="Filtered (EMA)")

    y_min = float(np.min(filt))
    y_max = float(np.max(filt))
    y_span = y_max - y_min
    y_text_place  = y_max + 0.06 * y_span
    y_text_remove = y_max + 0.13 * y_span   # offset so labels don't overlap

    if place_stats is not None:
        add_step_annotations(ax, "place",  t_place,  place_stats,  "#1D6FD6", y_text_place)
    if remove_stats is not None:
        add_step_annotations(ax, "remove", t_remove, remove_stats, "#D65C1D", y_text_remove)

    ax.set_title(f"Step Response ({args.step_weight:.0f} g place/remove)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Weight (g)")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()

    plot_path = Path(args.plot)
    if not plot_path.is_absolute():
        plot_path = Path(__file__).parent / plot_path
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=200)
    print(f"Saved plot → {plot_path}")
    plt.show()


if __name__ == "__main__":
    main()