# Weighing Scale Project — Improvements Summary

## 1. Removed Snap/Quantisation Function

**Problem:** The original `snap_weight()` function rounded readings to a fixed set of preset levels (10, 20, 50, 100, 200, 500, 700, 800, 900 g) within a ±0.5 g tolerance window. This masked the true sensor error and made precision testing with arbitrary weights (e.g. 65, 289, 370, 693 g) impossible — the displayed value would simply be the nearest preset level rather than the actual measurement.

**Fix:** Removed `snap_weight()` entirely and all associated constants (`SNAP_TOL_G`, `SNAP_LEVELS[]`, `SNAP_LEVEL_COUNT`). The scale now outputs the true measured value at all times.

---

## 2. Filter Redesign — Median + EMA, No Deadband

**Problem:** The original `filter_weight()` used three mechanisms:
- **EMA** (α = 0.2) — exponential moving average
- **Zero-tracking** — slowly pulled the reading toward zero regardless of load
- **Deadband** (±0.2 g) — any reading below 0.2 g was forced to exactly 0.0 g

The deadband created a visible discontinuity: the display would jump between `0.00 g` and a real value without any intermediate transition, making the noise look worse than it actually was. The zero-tracking also interfered with real small loads.

**New filter (two-stage):**

| Stage | Method | Role |
|---|---|---|
| 1 | Median filter, window = 5 | Rejects spike outliers from the ADC |
| 2 | EMA, α = 0.23 | Smooths residual random noise |

Hardware averaging was also increased from `AVG_SAMPLES = 3` to `AVG_SAMPLES = 8`, reducing raw noise at the HX711 level before any software processing.

**Measured noise (empty scale, 120 s recording):**

| Signal | std (g) | Peak-to-peak (g) |
|---|---|---|
| Raw (pre-filter) | 0.092 | 0.36 |
| Filtered output | 0.075 | 0.28 |

No significant drift was detected (slope < 0.001 g/s, r < 0.41).

---

## 3. Auto-Zero at Startup

**Problem:** The correction polynomial (fitted in calibration) has a non-zero constant term `CORR_A0`. Because `scale.tare()` zeros only the HX711 ADC and not the polynomial output, the displayed reading at zero load was approximately **−0.50 g** even with the scale empty.

**Fix:** In `setup()`, after `scale.tare()`, the Arduino runs a 60-iteration warm-up loop:
- First 30 iterations: fills the median buffer and lets the EMA converge.
- Last 30 iterations: averages the output and stores it as `zero_correction_g`.

In `loop()`, every reading is offset by this value:
```cpp
float grams_disp = correction_weight(filter_weight(grams_cal)) - zero_correction_g;
```

**Result:** Empty-scale mean dropped from −0.50 g to +0.06 g.

**Requirement:** The scale must be empty during the ~6-second startup sequence. The LCD (or serial) displays `Zeroing...` until the procedure is complete.

---

## 4. Calibration Pipeline

The full signal chain is:

```
HX711 ADC
  → ÷ calibration_factor (646.5)          [linear scale to grams]
  → calibrated_weight()                   [4th-order nonlinearity correction]
  → filter_weight()                       [median(5) + EMA(α=0.23)]
  → correction_weight()                   [4th-order precision correction]
  → − zero_correction_g                  [startup auto-zero]
  → Serial output / display
```

### 4a. First-stage polynomial — `calibrated_weight()`

Fitted from the original dataset mapping raw HX711 units to grams. Coefficients:

```
POLY_A4 =  2.3222629e-12
POLY_A3 = -2.1376726e-10
POLY_A2 = -2.0776874e-06
POLY_A1 =  0.99711933
POLY_A0 =  0.011400161
```

### 4b. Interactive 10-point calibration — `correction_weight()`

A Python script (`interactive_calibration.py`) was written to perform a stratified-random calibration:
- Divides 0–1000 g into 10 equal bands of 100 g each.
- Picks one random target per band, ensuring uniform coverage.
- Displays a live serial reading; user presses Enter when the reading is stable.
- User confirms or corrects the reference weight.
- Fits a polynomial of selectable order (4th order used here).
- Prints Arduino-ready `const float` declarations using Horner's method.

**Final calibration results (4th-order, 10 points):**

| Point | Scale reading (g) | Reference (g) | Error before | Error after |
|---|---|---|---|---|
| 1 | 58.02 | 58.00 | +0.02 | +0.01 |
| 2 | 122.92 | 123.00 | −0.08 | −0.02 |
| 3 | 228.82 | 229.00 | −0.18 | +0.02 |
| 4 | 370.63 | 371.00 | −0.37 | +0.03 |
| 5 | 411.58 | 412.10 | −0.52 | −0.06 |
| 6 | 549.35 | 550.00 | −0.65 | +0.01 |
| 7 | 679.31 | 680.00 | −0.69 | +0.10 |
| 8 | 722.20 | 723.10 | −0.90 | −0.10 |
| 9 | 858.30 | 859.00 | −0.70 | +0.01 |
| 10 | 958.55 | 959.00 | −0.45 | −0.00 |

| Metric | Before correction | After correction |
|---|---|---|
| RMSE | 0.534 g | **0.049 g** |
| Max absolute error | 0.901 g | **0.098 g** |

Fitted coefficients:
```
CORR_A4 = -4.038726222e-12
CORR_A3 =  3.803357929e-09
CORR_A2 = -6.355544816e-07
CORR_A1 =  1.001230991
CORR_A0 = -0.08763373116
```

---

## 5. Noise & Drift Measurement Tool

A dedicated analysis script (`record_zero_drift.py`) was written to characterise filter performance. The Arduino outputs two values per line:

```
W,<filtered_g>,<raw_cal_g>
```

The script records both channels simultaneously and produces:
1. **Time-series overlay** — raw vs filtered with trend lines.
2. **Histogram comparison** — distributions with Gaussian fits and noise-reduction percentage.
3. **Rolling 20-second σ** — verifies that noise is stationary over time.

Printed statistics: mean, σ, peak-to-peak, drift slope (g/min) and Pearson r for both channels.

---

## 6. Precision Test Requirements

Per the assignment, the scale was tested with more than 10 random weights in the 1–999 g range. The snap function was removed to ensure the displayed value reflects the true sensor output with no artificial rounding. The final calibrated scale achieves:

- **RMSE ≤ 0.05 g** across the calibration range
- **Max error ≤ 0.1 g** at any calibration point
- **Noise σ ≈ 0.075 g** at zero load

These figures are well within a ±1 g precision specification across the full 0–1000 g range.
