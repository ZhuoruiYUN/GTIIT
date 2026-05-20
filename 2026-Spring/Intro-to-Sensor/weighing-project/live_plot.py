#!/usr/bin/env python3
"""
Live plot of weight data from Arduino serial.
Expected lines: W,<grams>
"""

import argparse
import threading
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

DEFAULT_PORT = "/dev/tty.usbmodem31301"  # Update to your serial port
DEFAULT_BAUD = 115200
DEFAULT_INTERVAL_MS = 50
DEFAULT_SAVE_EVERY = 100


def serial_reader(port, baud, data_t, data_g, lock, stop_flag):
    t0 = time.time()
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--save-every", type=int, default=DEFAULT_SAVE_EVERY)
    args = ap.parse_args()

    if not args.port:
        raise SystemExit("Set DEFAULT_PORT in the script or pass --port.")

    data_t = deque()
    data_g = deque()
    lock = threading.Lock()
    stop_flag = [False]

    t = threading.Thread(
        target=serial_reader,
        args=(args.port, args.baud, data_t, data_g, lock, stop_flag),
        daemon=True,
    )
    t.start()

    fig, ax = plt.subplots(figsize=(9, 4.5))
    line, = ax.plot([], [], color="#1D9E75", linewidth=1.6)
    readout = ax.text(0.02, 0.95, "", transform=ax.transAxes, ha="left", va="top")
    ax.set_title("Live Weight")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Weight (g)")
    ax.grid(True, alpha=0.2)

    frame_count = [0]
    out_dir = Path(__file__).with_name("frames")
    out_dir.mkdir(exist_ok=True)

    def update(_):
        with lock:
            if not data_t:
                return line,
            x = list(data_t)
            y = list(data_g)
        line.set_data(x, y)
        ax.set_xlim(0.0, max(1.0, x[-1]))
        ymin = min(y)
        ymax = max(y)
        pad = max(0.5, (ymax - ymin) * 0.1)
        ax.set_ylim(ymin - pad, ymax + pad)
        readout.set_text(f"Current: {y[-1]:.2f} g")

        frame_count[0] += 1
        if args.save_every > 0 and frame_count[0] % args.save_every == 0:
            out_path = out_dir / f"frame_{frame_count[0]:06d}.png"
            fig.savefig(out_path, dpi=120)
        return line, readout

    ani = FuncAnimation(fig, update, interval=DEFAULT_INTERVAL_MS, blit=False)

    try:
        plt.show()
    finally:
        stop_flag[0] = True


if __name__ == "__main__":
    main()
