# Weighing Scale Project Report Template

## 1. Abstract
- Brief summary of goal, method, and key results.

## 2. System Overview
### 2.1 Hardware
- Load cell: CZL611N (1 kg)
- ADC module: HX711
- MCU: Arduino Mega (or specify)
- Output: Buzzer, Web monitor

### 2.2 Wiring
- Summarize key connections (DT/SCK, power, buzzer pin, etc.).

### 2.3 Software
- Arduino firmware for sampling, calibration, filtering
- Python scripts for calibration, zero-drift, live plot
- Web monitor for visualization

## 3. Calibration Data
### 3.1 Data Table
- Source file: M2weigh-calibration.csv

### 3.2 Calibration Method
- Fit polynomial from sensor output to true weight.
- Compare orders 1-5 and select 4th order.

### 3.3 Calibration Formula
Use the selected model:

$$
W = a_4 x^4 + a_3 x^3 + a_2 x^2 + a_1 x + a_0
$$

Where:
- $W$ is weight in grams
- $x$ is sensor output (scale reading)

**Selected coefficients (4th order):**
- $a_4 = 2.3222629e-12$
- $a_3 = -2.1376726e-10$
- $a_2 = -2.0776874e-06$
- $a_1 = 0.99711933$
- $a_0 = 0.011400161$

## 4. Filtering and Zero-Drift Suppression
### 4.1 EMA Filter
Equation:

$$
\hat{y}_k = \alpha x_k + (1-\alpha)\hat{y}_{k-1}
$$

Where:
- $x_k$ is the calibrated weight at sample $k$
- $\hat{y}_k$ is the filtered output
- $\alpha$ controls smoothing (larger = faster, smaller = smoother)

### 4.2 Zero Tracking + Deadband
- Only adjust zero bias near 0 g.
- Deadband to suppress jitter around zero.

### 4.3 Firmware Filtering Implementation (Example)
```cpp
const float EMA_ALPHA = 0.2f;
const float ZERO_DEADBAND_G = 0.2f;
const float ZERO_TRACK_LIMIT_G = 0.6f;
const float ZERO_TRACK_BETA = 0.005f;

float filter_weight(float grams) {
  static bool ema_init = false;
  static float ema = 0.0f;
  static float zero_bias = 0.0f;

  if (!ema_init) {
    ema = grams;
    ema_init = true;
  } else {
    ema = EMA_ALPHA * grams + (1.0f - EMA_ALPHA) * ema;
  }

  float compensated = ema - zero_bias;
  if (fabsf(compensated) < ZERO_TRACK_LIMIT_G) {
    zero_bias += ZERO_TRACK_BETA * compensated;
    compensated = ema - zero_bias;
  }

  if (fabsf(compensated) < ZERO_DEADBAND_G) {
    compensated = 0.0f;
  }
  return compensated;
}
```

## 5. Experiments and Results
### 5.1 Zero Drift (Creep + Noise)
- Procedure:
  - Tare at no load
  - Record for 140 s
  - Plot zero drift over time

**Insert figure:**

![Zero drift scatter](images/zero_drift.png)

### 5.2 Dynamic Response (Step Test)
- Step input: place/remove weight
- Metrics: rise time, settling time, overshoot
- Place step (0 g -> 500 g): baseline 0.131 g, rise 4.45 s, settle 7.72 s
- Remove step (500 g -> 0 g): baseline 499.81 g, rise 4.45 s, settle 7.42 s

**Insert figure:**

![Step response](images/step_response.png)

### 5.3 Real-Time Monitoring
- Live plot with current weight readout
- Saved frames every 100 updates

**Insert figure:**

![Live monitoring](images/live_plot.png)

## 6. Code Snippets
### 6.1 Arduino (Calibration + Filtering)
```cpp
// Example: apply 4th order calibration
float calibrated_weight(float x) {
  return ((((a4 * x + a3) * x + a2) * x + a1) * x + a0);
}
```

### 6.2 Python (Calibration Fit)
```python
coeffs = np.polyfit(x, y, 3)
```

### 6.3 Python (Zero Drift Recorder)
```python
data, elapsed = read_samples(port, baud, duration, warmup)
```

## 7. Discussion
- Compare fitting orders and justify 4th order selection.
- Effects of filtering on noise and response time.
- Limitations (sensor noise, drift, mechanical hysteresis).

## 8. Conclusion
- Final accuracy and stability achieved.
- Suggested improvements.

## 9. Appendix
### 9.1 Full Arduino Code
- Insert full firmware or reference file path.

### 9.2 Full Python Scripts
- Insert scripts or reference file paths.
