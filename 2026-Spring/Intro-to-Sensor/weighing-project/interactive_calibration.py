
#!/usr/bin/env python3
"""
interactive_calibration.py
Stratified-random 10-point calibration for the CZL611N / HX711 scale.

Workflow:
  1. Generates 10 targets spread evenly across 0-1000 g (one per 100 g band).
  2. For each target, reads live serial data. Press Enter to capture.
  3. Optionally enter the actual reference weight (or press Enter to use target).
  4. Fits a correction polynomial: reference = poly(scale_reading).
  5. Prints Arduino-ready correction code and saves a PNG + CSV.

Usage:
  python interactive_calibration.py
  python interactive_calibration.py --port /dev/tty.usbmodem31301 --order 3
"""

import argparse
import csv
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial

DEFAULT_PORT = "/dev/tty.usbmodem31301"
DEFAULT_BAUD = 115200
N_STABLE = 25          # readings averaged on capture
STABLE_SPREAD_G = 1.0  # threshold to show STABLE indicator

_latest: list[float | None] = [None]
_lock = threading.Lock()


# ── Serial thread ──────────────────────────────────────────────────────────────

def _serial_thread(port: str, baud: int, stop: list[bool]) -> None:
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            while not stop[0]:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("W,"):
                    try:
                        with _lock:
                            _latest[0] = float(line.split(",")[1])
                    except ValueError:
                        pass
    except serial.SerialException as exc:
        print(f"\n[serial] {exc}")


def _wait_for_data(timeout: float = 90.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        with _lock:
            if _latest[0] is not None:
                return True
        time.sleep(0.1)
    return False


# ── Capture ────────────────────────────────────────────────────────────────────

def capture_on_enter() -> float | None:
    """Display live reading; press Enter to lock in average of last N_STABLE readings."""
    buf: list[float] = []
    done = threading.Event()

    def _live() -> None:
        while not done.is_set():
            with _lock:
                g = _latest[0]
            if g is not None:
                buf.append(g)
                recent = buf[-N_STABLE:]
                spread = (max(recent) - min(recent)) if len(recent) > 1 else 999.9
                stable_tag = "STABLE" if len(recent) == N_STABLE and spread < STABLE_SPREAD_G else "      "
                print(
                    f"\r  [{stable_tag}]  {g:8.2f} g   spread={spread:5.2f} g   n={len(buf):4d}",
                    end="",
                    flush=True,
                )
            time.sleep(0.08)

    t = threading.Thread(target=_live, daemon=True)
    t.start()
    input()
    done.set()
    t.join()
    print()

    if not buf:
        return None
    return float(np.mean(buf[-N_STABLE:]))


# ── Polynomial helpers ─────────────────────────────────────────────────────────

def _arduino_code(coeffs: np.ndarray, fn: str = "correction_weight") -> str:
    degree = len(coeffs) - 1
    lines: list[str] = []
    for i, c in enumerate(coeffs):
        p = degree - i
        lines.append(f"const float CORR_A{p} = {c:.10g}f;")
    # Horner's method
    expr = f"CORR_A{degree}"
    for i in range(1, degree + 1):
        p = degree - i
        expr = f"({expr} * x + CORR_A{p})"
    lines += [
        f"",
        f"float {fn}(float x) {{",
        f"  return {expr};",
        f"}}",
    ]
    return "\n".join(lines)


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="Interactive 10-point scale calibration")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--order", type=int, default=4, help="Correction polynomial order (default 3)")
    ap.add_argument("--seed", type=int, default=None, help="RNG seed for reproducibility")
    ap.add_argument("--out", default="calibration_interactive.csv")
    args = ap.parse_args()

    # ── Generate stratified targets: one random point per 100 g band ──────────
    rng = np.random.default_rng(args.seed)
    targets = sorted(int(rng.integers(b * 100 + 5, (b + 1) * 100 - 5)) for b in range(10))

    print("=" * 60)
    print("  Interactive Calibration — 10-point stratified sampling")
    print("=" * 60)
    print(f"  Targets (g): {targets}")
    print(f"  Port       : {args.port}  @  {args.baud} baud")
    print(f"  Poly order : {args.order}")
    print("=" * 60)
    print("Connecting …")

    stop = [False]
    th = threading.Thread(target=_serial_thread, args=(args.port, args.baud, stop), daemon=True)
    th.start()

    if not _wait_for_data():
        stop[0] = True
        raise SystemExit("No serial data — check port, baud, and that the Arduino is running.")
    print("Connected.\n")

    scale_readings: list[float] = []
    reference_weights: list[float] = []

    for i, tgt in enumerate(targets):
        sep = "─" * 55
        print(sep)
        print(f"  Point {i+1:2d} / 10   │   Target ≈ {tgt} g")
        print(sep)
        print(f"  1. Place {tgt} g on the scale.")
        print("  2. Wait for STABLE, then press Enter to capture.")

        reading = capture_on_enter()
        if reading is None:
            print("  No data captured — skipping.\n")
            continue

        prompt = (
            f"  Captured: {reading:.2f} g  │  "
            f"Enter actual reference weight (or Enter to use {tgt}): "
        )
        raw = input(prompt).strip()
        try:
            ref = float(raw)
        except ValueError:
            ref = float(tgt)

        scale_readings.append(reading)
        reference_weights.append(ref)
        print(f"  Saved: scale={reading:.2f} g  →  reference={ref:.2f} g\n")

    stop[0] = True

    n = len(scale_readings)
    if n < 2:
        raise SystemExit("Not enough points to fit (need ≥ 2).")

    x = np.array(scale_readings)
    y = np.array(reference_weights)

    # ── Save CSV ───────────────────────────────────────────────────────────────
    out_path = Path(args.out)
    with out_path.open("w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["scale_g", "reference_g", "error_before_g"])
        for sx, sy in zip(x, y):
            wr.writerow([f"{sx:.4f}", f"{sy:.4f}", f"{sx - sy:.4f}"])
    print(f"Data saved → {out_path}")

    # ── Errors before correction ───────────────────────────────────────────────
    err_before = x - y
    rmse_b = float(np.sqrt(np.mean(err_before ** 2)))
    max_b = float(np.max(np.abs(err_before)))

    # ── Fit polynomial ─────────────────────────────────────────────────────────
    order = min(args.order, n - 1)
    coeffs = np.polyfit(x, y, order)
    poly = np.poly1d(coeffs)
    y_pred = poly(x)
    err_after = y_pred - y
    rmse_a = float(np.sqrt(np.mean(err_after ** 2)))
    max_a = float(np.max(np.abs(err_after)))

    # ── Results table ──────────────────────────────────────────────────────────
    print()
    print("┌──────────────────────────────────────────────────┐")
    print("│              Calibration Results                 │")
    print("├──────────────────────────────────────────────────┤")
    print(f"│  {'Point':>5}  {'Scale':>8}  {'Ref':>8}  {'Before':>8}  {'After':>8}  │")
    print("├──────────────────────────────────────────────────┤")
    for i, (sx, sy, eb, ea) in enumerate(zip(x, y, err_before, err_after)):
        flag = " !" if abs(ea) > 1.0 else "  "
        print(f"│  {i+1:>5}  {sx:>8.2f}  {sy:>8.2f}  {eb:>+8.2f}  {ea:>+8.2f}{flag}│")
    print("├──────────────────────────────────────────────────┤")
    print(f"│  RMSE   before: {rmse_b:6.3f} g    after: {rmse_a:6.3f} g        │")
    print(f"│  MaxErr before: {max_b:6.3f} g    after: {max_a:6.3f} g        │")
    print("└──────────────────────────────────────────────────┘")

    # ── Arduino code ───────────────────────────────────────────────────────────
    print()
    print("─" * 60)
    print("  Paste into test_tarezero.ino  (before setup())")
    print("─" * 60)
    print(_arduino_code(coeffs))
    print("─" * 60)
    print()
    print("  Then in loop(), change the last processing line to:")
    print()
    print("    float grams_disp = correction_weight(filter_weight(grams_cal));")
    print()
    print("  And update Serial.println:")
    print()
    print("    Serial.println(grams_disp, 2);")
    print("─" * 60)

    # ── Plot ───────────────────────────────────────────────────────────────────
    x_plot = np.linspace(max(0.0, float(x.min()) - 30), min(1050.0, float(x.max()) + 30), 400)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.scatter(x, y, color="#222", s=50, zorder=5, label="Calibration points")
    ax1.plot(x_plot, poly(x_plot), color="#1D9E75", linewidth=2, label=f"Poly order {order}")
    ax1.plot([0, 1000], [0, 1000], "--", color="#aaa", linewidth=1, label="Ideal (y = x)")
    for sx, sy in zip(x, y):
        ax1.annotate(f"{sy:.0f}", (sx, sy), textcoords="offset points", xytext=(4, 4), fontsize=7)
    ax1.set_xlabel("Scale reading (g)")
    ax1.set_ylabel("Reference weight (g)")
    ax1.set_title("Correction Curve")
    ax1.grid(alpha=0.3)
    ax1.legend()

    colors_after = ["#A32D2D" if abs(e) > 1.0 else "#1D9E75" for e in err_after]
    ax2.bar(range(n), err_after, color=colors_after, label="Residual after fit", zorder=3)
    ax2.scatter(range(n), err_before, color="#888", s=30, zorder=5, label="Error before fit")
    ax2.axhline(0, color="#222", linewidth=0.8)
    ax2.axhline(1, color="#BA7517", linewidth=0.8, linestyle="--")
    ax2.axhline(-1, color="#BA7517", linewidth=0.8, linestyle="--", label="±1 g")
    ax2.set_xticks(range(n))
    ax2.set_xticklabels([f"{int(r)}" for r in y], rotation=45, ha="right", fontsize=8)
    ax2.set_xlabel("Reference weight (g)")
    ax2.set_ylabel("Error (g)")
    ax2.set_title("Residuals")
    ax2.legend()

    fig.tight_layout()
    img_path = out_path.with_suffix(".png")
    fig.savefig(img_path, dpi=150)
    print(f"Plot saved → {img_path}")
    plt.show()


if __name__ == "__main__":
    main()
