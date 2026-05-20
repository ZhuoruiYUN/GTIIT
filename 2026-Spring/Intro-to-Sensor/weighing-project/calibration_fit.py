#!/usr/bin/env python3
"""
Fit correction models (order 1-5) from measured output to target weight.
Column 1 = nominal weight, column 2 = reference scale weight,
column 3 = current scale output (already 4th-order calibrated).
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

DATA_PATH = Path(__file__).with_name("M2weigh-calibration.csv")
ORDERS = range(1, 6)
SELECTED_ORDER = 4
DEFAULT_TARGET = "nominal"  # "nominal" or "reference"


def parse_value(text):
    if text is None:
        return None
    s = text.strip()
    if not s:
        return None
    if "+" in s:
        parts = s.split("+")
        try:
            return sum(float(p) for p in parts)
        except ValueError:
            return None
    try:
        return float(s)
    except ValueError:
        return None


def load_data(path: Path):
    nominal = []
    reference = []
    measured = []
    with path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) < 3:
                continue
            n = parse_value(row[0])
            r = parse_value(row[1])
            m = parse_value(row[2])
            if n is None or r is None or m is None:
                continue
            nominal.append(n)
            reference.append(r)
            measured.append(m)
    return (
        np.array(nominal, dtype=float),
        np.array(reference, dtype=float),
        np.array(measured, dtype=float),
        header,
    )


def fit_models(x, y):
    models = {}
    for n in ORDERS:
        coeffs = np.polyfit(x, y, n)
        models[n] = coeffs
    return models


def evaluate_model(coeffs, x, y):
    p = np.poly1d(coeffs)
    y_pred = p(x)
    rmse, mae, max_abs = error_stats(y_pred, y)
    return y_pred, rmse, mae, max_abs


def error_stats(y_pred, y_true):
    err = y_pred - y_true
    rmse = math.sqrt(np.mean(err ** 2))
    mae = np.mean(np.abs(err))
    max_abs = np.max(np.abs(err))
    return rmse, mae, max_abs


def format_poly(coeffs):
    terms = []
    degree = len(coeffs) - 1
    for i, c in enumerate(coeffs):
        p = degree - i
        if p == 0:
            terms.append(f"{c:.8g}")
        elif p == 1:
            terms.append(f"{c:.8g}*x")
        else:
            terms.append(f"{c:.8g}*x^{p}")
    return " + ".join(terms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--target",
        choices=["nominal", "reference"],
        default=DEFAULT_TARGET,
        help="Use nominal or reference scale weights as target",
    )
    args = ap.parse_args()

    nominal, reference, measured, header = load_data(DATA_PATH)
    if measured.size == 0:
        raise SystemExit("No data loaded. Check CSV format.")

    if args.target == "reference":
        y = reference
        target_label = "reference scale"
    else:
        y = nominal
        target_label = "nominal"

    x = measured

    models = fit_models(x, y)
    x_plot = np.linspace(x.min(), x.max(), 300)

    print("Fitting target weight = f(measured output)")
    if header:
        print("CSV columns:", header)
    print("Data points:", len(x))
    raw_rmse, raw_mae, raw_max = error_stats(x, y)
    print(
        f"Raw measured vs {target_label}: RMSE={raw_rmse:.4f} g, "
        f"MAE={raw_mae:.4f} g, MaxAbs={raw_max:.4f} g"
    )

    orders_list = []
    rmse_list = []
    mae_list = []
    max_abs_list = []

    plt.figure(figsize=(9, 6))
    plt.scatter(x, y, color="#222", s=28, label="Data")
    plt.plot(x_plot, x_plot, color="#888", linewidth=1.0, linestyle="--", label="y=x")

    for n in ORDERS:
        coeffs = models[n]
        p = np.poly1d(coeffs)
        _, rmse, mae, max_abs = evaluate_model(coeffs, x, y)
        print(
            f"Order {n}: RMSE={rmse:.4f} g, MAE={mae:.4f} g, MaxAbs={max_abs:.4f} g"
        )
        orders_list.append(n)
        rmse_list.append(rmse)
        mae_list.append(mae)
        max_abs_list.append(max_abs)
        is_selected = n == SELECTED_ORDER
        plt.plot(
            x_plot,
            p(x_plot),
            linewidth=2.2 if is_selected else 1,
            label=f"Order {n}" + (" (selected)" if is_selected else ""),
        )

    if SELECTED_ORDER in models:
        coeffs = models[SELECTED_ORDER]
        print("\nSelected order:", SELECTED_ORDER)
        print("Polynomial (weight = f(sensor)):")
        print("  w =", format_poly(coeffs))

    plt.title("Calibration Fit: Target Weight vs Measured Output")
    plt.grid(alpha=0.3)
    plt.xlabel("Measured output (4th-order result)")
    plt.ylabel(f"Target weight ({target_label}, g)")
    plt.legend()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(9, 6))
    plt.plot(orders_list, rmse_list, marker="o", label="RMSE (g)")
    plt.plot(orders_list, mae_list, marker="o", label="MAE (g)")
    plt.plot(orders_list, max_abs_list, marker="o", label="MaxAbs (g)")
    plt.title("Calibration Fit Error vs Polynomial Order")
    plt.grid(alpha=0.3)
    plt.xlabel("Polynomial order")
    plt.ylabel("Error (g)")
    plt.xticks(list(ORDERS))
    plt.legend()
    plt.tight_layout()
    plt.show()




if __name__ == "__main__":
    main()
