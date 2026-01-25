# -*- coding: utf-8 -*-
"""
Trajectory utilities for IRB2400 project (pure refactor):
- 7th-order S-curve time scaling
- synchronized joint-space move planner (time-opt under vel/acc limits)
- Hann smoothing helper

No behavior change intended vs original single-file script.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple, Optional

import numpy as np

def scurve7(u):
    u = float(np.clip(u, 0.0, 1.0))
    u2, u3, u4, u5, u6, u7 = u*u, u*u*u, u*u*u*u, u*u*u*u*u, u*u*u*u*u*u, u*u*u*u*u*u*u
    s = 35.0*u4 - 84.0*u5 + 70.0*u6 - 20.0*u7
    sd = 140.0*u3 - 420.0*u4 + 420.0*u5 - 140.0*u6
    sdd = 420.0*u2 - 1680.0*u3 + 2100.0*u4 - 840.0*u5
    return s, sd, sdd


# Precompute peak |ds/du| and |d2s/du2| for scurve7 (for time selection)
_u_grid = np.linspace(0.0, 1.0, 2001)
_sd_grid = np.array([scurve7(ui)[1] for ui in _u_grid])
_sdd_grid = np.array([scurve7(ui)[2] for ui in _u_grid])
SC7_SD_MAX = float(np.max(np.abs(_sd_grid)))
SC7_SDD_MAX = float(np.max(np.abs(_sdd_grid)))



_u_grid = np.linspace(0.0, 1.0, 2001)
_sd_grid = np.array([scurve7(ui)[1] for ui in _u_grid])
_sdd_grid = np.array([scurve7(ui)[2] for ui in _u_grid])
SC7_SD_MAX = float(np.max(np.abs(_sd_grid)))
SC7_SDD_MAX = float(np.max(np.abs(_sdd_grid)))


def plan_sync_scurve7(q0, q1, qd_max, qdd_max, dt=0.01):
    """
    Synchronized minimum-time (within this scaling family) move from q0->q1
    using scurve7 scaling, enforcing per-joint speed and acceleration limits.

    Returns:
      t (N,), q (N,6), qd (N,6), qdd (N,6), T_total
    """
    q0 = np.asarray(q0, dtype=float).reshape(6)
    q1 = np.asarray(q1, dtype=float).reshape(6)
    dq = q1 - q0

    # If trivial move
    if np.max(np.abs(dq)) < 1e-12:
        t = np.array([0.0])
        q = q0[None, :]
        qd = np.zeros((1, 6))
        qdd = np.zeros((1, 6))
        return t, q, qd, qdd, 0.0

    # Time needed per joint from speed and accel constraints:
    # qd_max >= |dq|*max|ds/du|/T  =>  T >= |dq|*SD_MAX/qd_max
    # qdd_max >= |dq|*max|d2s/du2|/T^2 => T >= sqrt(|dq|*SDD_MAX/qdd_max)
    T_req = 0.0
    for i in range(6):
        d = abs(float(dq[i]))
        if d < 1e-12:
            continue
        Tv = d * SC7_SD_MAX / float(qd_max[i])
        Ta = math.sqrt(d * SC7_SDD_MAX / float(qdd_max[i]))
        T_req = max(T_req, Tv, Ta)

    # Discretize timeline
    if T_req < dt:
        T_req = dt
    N = int(math.ceil(T_req / dt)) + 1
    t = np.linspace(0.0, T_req, N)

    q = np.zeros((N, 6), dtype=float)
    qd = np.zeros((N, 6), dtype=float)
    qdd = np.zeros((N, 6), dtype=float)

    for k, tk in enumerate(t):
        u = tk / T_req
        s, sd, sdd = scurve7(u)
        q[k, :] = q0 + dq * s
        qd[k, :] = dq * (sd / T_req)
        qdd[k, :] = dq * (sdd / (T_req**2))

    return t, q, qd, qdd, T_req


# ---------------------------
# Simple smoothing for numeric differentiation (optional)
# ---------------------------

def smooth_hann(x, win=21):
    """Hann-window smoothing along time axis (N,dim)."""
    x = np.asarray(x, dtype=float)
    if win <= 3 or win % 2 == 0:
        return x
    w = np.hanning(win)
    w = w / np.sum(w)
    pad = win // 2
    if x.ndim == 1:
        xp = np.pad(x, (pad, pad), mode='edge')
        return np.convolve(xp, w, mode='valid')
    # (N,dim)
    out = np.zeros_like(x)
    for j in range(x.shape[1]):
        xp = np.pad(x[:, j], (pad, pad), mode='edge')
        out[:, j] = np.convolve(xp, w, mode='valid')
    return out
# Newton-Euler dynamics (Craig-style, expressed in each link frame)
# ---------------------------


@dataclass
class TrajectoryPlanner:
    dt: float = 0.02

    def plan_move(self, q0: np.ndarray, q1: np.ndarray, qd_lim: np.ndarray, qdd_lim: np.ndarray):
        # keep return signature identical to plan_sync_scurve7
        return plan_sync_scurve7(q0, q1, qd_lim, qdd_lim, dt=self.dt)

    def smooth(self, x: np.ndarray, win: int = 17) -> np.ndarray:
        return smooth_hann(x, win=win)
