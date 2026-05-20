"""
plot_report_figures.py
======================
Standalone script that regenerates the three figures used in the report:

  1. dashboard.png         — 8-panel time-domain simulation dashboard
                             (one per design; call plot_dashboard() for each)
  2. sensitivity_sweep.png — 6-panel parameter sensitivity sweep
  3. saturation_analysis.png — B-H curve + effective permeability

Run:
    python plot_report_figures.py

All figures are saved to  ./motor_sim_outputs/
You must have already run motor_simulation.py at least once so that the
simulation helper functions are importable, OR you can copy the shared
data structures (BH_B, BH_H, COLORS, simulate, …) directly into this file.

Alternatively, just do:
    import motor_simulation as ms
and call the functions below.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")           # non-interactive backend (safe on any system)
import matplotlib.pyplot as plt
import os, sys

# ── output folder ─────────────────────────────────────────────────────────────
OUT_DIR = "motor_sim_outputs"
os.makedirs(OUT_DIR, exist_ok=True)

# ── colour palette (used in dashboard) ────────────────────────────────────────
COLORS = {
    "speed":   "#2196F3",   # blue
    "current": "#FF5722",   # deep orange
    "emf":     "#9C27B0",   # purple
    "P_in":    "#607D8B",   # blue-grey
    "P_out":   "#4CAF50",   # green
    "P_cu":    "#F44336",   # red
    "P_eddy":  "#FF9800",   # orange
    "P_hyst":  "#FFEB3B",   # yellow
    "P_mec":   "#795548",   # brown
    "eta":     "#009688",   # teal
}

# ── M19 laminated electrical-steel B-H data ───────────────────────────────────
BH_B = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
                 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3,
                 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0])
BH_H = np.array([    0,   28,   40,   50,   60,   70,   85,
                   105,  135,  175,  240,  340,  530,  900,
                  1600, 3000, 6000, 12000, 24000, 45000, 80000])


# ═══════════════════════════════════════════════════════════════════════════════
#  FIGURE 1 — 8-PANEL SIMULATION DASHBOARD
#  Called once per design.  `res` is the dict returned by motor_simulation.simulate()
# ═══════════════════════════════════════════════════════════════════════════════

def plot_dashboard(res, mat_name, out_prefix):
    """
    Parameters
    ----------
    res        : dict with keys 't','w','I','emf','T_em','T_fan',
                 'P_out','P_cu','P_eddy','P_hyst','P_mec','P_in','eta'
                 (all 1-D numpy arrays of the same length)
    mat_name   : str  label shown in the figure title, e.g. "NdFeB N52"
    out_prefix : str  filename prefix, e.g. "High_Power_NdFeB"
    """
    t   = res["t"]
    rpm = res["w"] * 60.0 / (2.0 * np.pi)

    # ── canvas: 4 rows × 2 columns ────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 18))
    fig.patch.set_facecolor("#F8F9FA")
    gs  = fig.add_gridspec(4, 2, hspace=0.45, wspace=0.35,
                           left=0.08, right=0.94, top=0.95, bottom=0.04)
    fig.suptitle(f"PM Brushed DC Motor Simulation — {mat_name}",
                 fontsize=14, fontweight="bold", y=0.97)

    def _ax(row, col, title):
        """Create a styled subplot."""
        ax = fig.add_subplot(gs[row, col])
        ax.set_facecolor("white")
        ax.set_title(title, fontsize=10, fontweight="semibold", pad=6)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.grid(True, color="#DDDDDD", linewidth=0.6, zorder=0)
        ax.spines[["top", "right"]].set_visible(False)
        return ax

    def _ss(arr):
        """Mean of the last 20 % of an array (steady-state estimate)."""
        return float(np.mean(arr[int(0.8 * len(arr)):]))

    # ── Panel 1: Rotor Speed ──────────────────────────────────────────────────
    ax = _ax(0, 0, "Rotor Speed")
    ax.plot(t, rpm, color=COLORS["speed"], linewidth=1.6, zorder=3)
    ax.fill_between(t, rpm, alpha=0.08, color=COLORS["speed"])
    ax.set_ylabel("Speed (RPM)", fontsize=8)
    ss_rpm = _ss(rpm)
    ax.axhline(ss_rpm, color=COLORS["speed"], lw=0.8, ls=":", alpha=0.8)
    ax.text(t[-1] * 0.97, ss_rpm, f" {ss_rpm:.0f} RPM",
            color=COLORS["speed"], fontsize=7, va="bottom", ha="right")

    # ── Panel 2: Current (left y) & Back-EMF (right y) ───────────────────────
    ax = _ax(0, 1, "Current  &  Back-EMF")
    lc, = ax.plot(t, res["I"], color=COLORS["current"], lw=1.6,
                  label="Current (A)", zorder=3)
    ax.set_ylabel("Current (A)", color=COLORS["current"], fontsize=8)
    ax.tick_params(axis="y", labelcolor=COLORS["current"])

    ax2 = ax.twinx()
    ax2.spines["right"].set_visible(True)
    ax2.spines["right"].set_color(COLORS["emf"])
    le, = ax2.plot(t, res["emf"], color=COLORS["emf"], lw=1.6,
                   linestyle="--", label="Back-EMF (V)", zorder=3)
    ax2.set_ylabel("Back-EMF (V)", color=COLORS["emf"], fontsize=8)
    ax2.tick_params(axis="y", labelcolor=COLORS["emf"])

    ss_I   = _ss(res["I"])
    ss_emf = _ss(res["emf"])
    ax.axhline(ss_I,   color=COLORS["current"], lw=0.7, ls=":", alpha=0.7)
    ax.text(t[-1] * 0.03, ss_I,  f"{ss_I:.2f} A",
            color=COLORS["current"], fontsize=7, va="bottom")
    ax2.axhline(ss_emf, color=COLORS["emf"], lw=0.7, ls=":", alpha=0.7)
    ax2.text(t[-1] * 0.97, ss_emf, f"{ss_emf:.3f} V ",
             color=COLORS["emf"], fontsize=7, va="top", ha="right")
    ax.legend(handles=[lc, le], fontsize=7, loc="center right",
              framealpha=0.85, edgecolor="#CCCCCC")

    # ── Panel 3: Major losses — linear scale ─────────────────────────────────
    ax = _ax(1, 0, "Input Power  &  Major Losses")
    ax.plot(t, res["P_in"],  color=COLORS["P_in"],  lw=1.8,
            label="P_input",     zorder=4)
    ax.plot(t, res["P_cu"],  color=COLORS["P_cu"],  lw=1.6,
            label="P_copper",    zorder=3)
    ax.plot(t, res["P_mec"], color=COLORS["P_mec"], lw=1.4,
            linestyle="--", label="P_mech loss", zorder=3)
    ax.set_ylabel("Power (W)", fontsize=8)
    ax.legend(fontsize=7, loc="upper right", framealpha=0.85, edgecolor="#CCCCCC")

    # ── Panel 4: Small losses — log scale ────────────────────────────────────
    ax = _ax(1, 1, "Small Losses  (log scale)")
    # floor at 1 µW so log(0) is avoided
    p_out  = np.maximum(res["P_out"],  1e-6)
    p_eddy = np.maximum(res["P_eddy"], 1e-6)
    p_hyst = np.maximum(res["P_hyst"], 1e-6)
    p_mec  = np.maximum(res["P_mec"],  1e-6)
    ax.semilogy(t, p_out,  color=COLORS["P_out"],  lw=1.6,
                label="P_fan output",  zorder=4)
    ax.semilogy(t, p_eddy, color=COLORS["P_eddy"], lw=1.4,
                linestyle="--", label="P_eddy",       zorder=3)
    ax.semilogy(t, p_hyst, color="#FFC107",         lw=1.4,
                linestyle="-.", label="P_hysteresis",  zorder=3)
    ax.semilogy(t, p_mec,  color=COLORS["P_mec"],  lw=1.4,
                linestyle=":",  label="P_mech loss",   zorder=3)
    ax.set_ylabel("Power (W) — log scale", fontsize=8)
    ax.legend(fontsize=7, loc="lower right", framealpha=0.85, edgecolor="#CCCCCC")

    # ── Panel 5: Efficiency ───────────────────────────────────────────────────
    ax = _ax(2, 0, "Efficiency")
    ax.plot(t, res["eta"] * 100, color=COLORS["eta"], lw=1.6, zorder=3)
    ax.fill_between(t, res["eta"] * 100, alpha=0.10, color=COLORS["eta"])
    ax.set_ylabel("Efficiency (%)", fontsize=8)
    ax.set_ylim(0, max(res["eta"].max() * 110, 5))
    ss_eta = _ss(res["eta"]) * 100
    ax.axhline(ss_eta, color=COLORS["eta"], lw=0.8, ls=":", alpha=0.8)
    ax.text(t[-1] * 0.97, ss_eta, f" {ss_eta:.2f}%",
            color=COLORS["eta"], fontsize=7, va="bottom", ha="right")

    # ── Panel 6: Torque ───────────────────────────────────────────────────────
    ax = _ax(2, 1, "Electromagnetic  &  Fan Torque")
    ax.plot(t, res["T_em"]  * 1e3, color="#3F51B5", lw=1.6,
            label="T_electromagnetic (mN·m)", zorder=4)
    ax.plot(t, res["T_fan"] * 1e3, color="#E91E63", lw=1.4,
            linestyle="--", label="T_fan load (mN·m)", zorder=3)
    ax.set_ylabel("Torque (mN·m)", fontsize=8)
    ax.legend(fontsize=7, loc="upper right", framealpha=0.85, edgecolor="#CCCCCC")

    # ── Panel 7: Steady-state power balance bar chart (log y) ────────────────
    ax = _ax(3, 0, "Steady-State Power Balance")
    idx = int(0.8 * len(t))
    bar_labels = ["P_input", "P_copper", "P_fan\n(output)",
                  "P_mech\nloss", "P_eddy", "P_hysteresis"]
    bar_vals   = [np.mean(res["P_in"] [idx:]),
                  np.mean(res["P_cu"] [idx:]),
                  np.mean(res["P_out"][idx:]),
                  np.mean(res["P_mec"][idx:]),
                  np.mean(res["P_eddy"][idx:]),
                  np.mean(res["P_hyst"][idx:])]
    bar_colors = [COLORS["P_in"], COLORS["P_cu"],  COLORS["P_out"],
                  COLORS["P_mec"], COLORS["P_eddy"], "#FFC107"]
    bars = ax.bar(bar_labels, bar_vals, color=bar_colors,
                  edgecolor="white", linewidth=0.8, zorder=3)
    ax.set_ylabel("Power (W)", fontsize=8)
    ax.set_yscale("log")
    ax.set_xlabel("")
    ax.tick_params(axis="x", labelsize=7)
    for bar, v in zip(bars, bar_vals):
        if v > 1e-5:
            label_str = (f"{v*1e3:.2f}\nmW" if v < 1 else f"{v:.2f}\nW")
            ax.text(bar.get_x() + bar.get_width() / 2, v * 1.4,
                    label_str, ha="center", va="bottom",
                    fontsize=6.5, fontweight="bold")

    # ── Panel 8: Cumulative fan energy & equivalent lift height ──────────────
    ax = _ax(3, 1, "Fan Work Output (Cumulative)")
    dt_arr = np.diff(t, prepend=t[0])
    E_fan  = np.cumsum(res["P_out"] * dt_arr)        # [J]
    h_lift = E_fan / (0.01 * 9.81)                   # [m], reference mass 10 g

    lE, = ax.plot(t, E_fan * 1e3, color=COLORS["P_out"], lw=1.6,
                  label="Fan energy (mJ)", zorder=4)
    ax.set_ylabel("Cumulative energy (mJ)", color=COLORS["P_out"], fontsize=8)
    ax.tick_params(axis="y", labelcolor=COLORS["P_out"])

    ax2 = ax.twinx()
    ax2.spines["right"].set_visible(True)
    ax2.spines["right"].set_color("#FF5722")
    lH, = ax2.plot(t, h_lift, color="#FF5722", lw=1.4,
                   linestyle="--", label="Equiv. lift height (m)", zorder=3)
    ax2.set_ylabel("Equiv. lift height (m)", color="#FF5722", fontsize=8)
    ax2.tick_params(axis="y", labelcolor="#FF5722")
    ax.legend(handles=[lE, lH], fontsize=7, loc="upper left",
              framealpha=0.85, edgecolor="#CCCCCC")

    # ── save ─────────────────────────────────────────────────────────────────
    path = os.path.join(OUT_DIR, f"{out_prefix}_dashboard.png")
    fig.savefig(path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Saved → {path}")


# ═══════════════════════════════════════════════════════════════════════════════
#  FIGURE 2 — PARAMETER SENSITIVITY SWEEP (6 panels)
#  Requires motor_simulation.py to be importable as `ms`
# ═══════════════════════════════════════════════════════════════════════════════

def plot_sensitivity(base_args, ms):
    """
    Parameters
    ----------
    base_args : tuple  (mat_no, L_mm, D_mm, N, wd_mm, lt_mm, V)
                       — the baseline design point
    ms        : module — motor_simulation module, already imported
    """
    # layout: base_args indices
    # 0=mat_no  1=L_mm  2=D_mm  3=N  4=wd_mm  5=lt_mm  6=V

    params_sweep = [
        # (panel title,  arg_index,  values_to_test,  is_int)
        ("Axial Depth (mm)",   1, [12, 15, 20, 25, 30],            False),
        ("Core Dia (mm)",      2, [7.0, 8.0, 9.0, 10.0, 11.0],    False),
        ("Turns N",            3, [15, 20, 25, 30, 40, 50],         True),
        ("Wire Dia (mm)",      4, [0.10, 0.15, 0.20, 0.25, 0.30],  False),
        ("Lam Thick (mm)",     5, [0.1, 0.2, 0.3, 0.5, 1.0],       False),
        ("Voltage (V)",        6, [1.5, 2.0, 3.0, 4.5, 6.0],       False),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.patch.set_facecolor("#F8F9FA")
    fig.suptitle(
        "Parameter Sensitivity Analysis — Fan Output Power & Efficiency\n"
        f"(Base: mat={base_args[0]}, L={base_args[1]}mm, "
        f"D={base_args[2]}mm, N={base_args[3]}, "
        f"wd={base_args[4]}mm, lam={base_args[5]}mm, V={base_args[6]}V)",
        fontsize=10, fontweight="bold")

    col_P = "#2196F3"   # blue  → fan power
    col_e = "#E91E63"   # pink  → efficiency

    for ax, (pname, idx, vals, is_int) in zip(axes.flat, params_sweep):
        Ps, Es = [], []
        for v in vals:
            args = list(base_args)
            args[idx] = int(v) if is_int else float(v)
            try:
                ss, *_ = ms.run_one(*args, label="_tmp", verbose=False)
                Ps.append(ss["P_out"] * 1e3)
                Es.append(ss["eta"]   * 100)
            except Exception:
                Ps.append(np.nan)
                Es.append(np.nan)

        # ── left y-axis: fan power ─────────────────────────────────────────
        ax.set_facecolor("white")
        ax.spines[["top", "right"]].set_visible(False)
        lP, = ax.plot(vals, Ps, "o-", color=col_P, lw=2, ms=6,
                      label="$P_\\mathrm{fan}$ (mW)", zorder=4)
        ax.set_ylabel("Fan Output Power (mW)", color=col_P, fontsize=8)
        ax.tick_params(axis="y", labelcolor=col_P)

        # ── right y-axis: efficiency ───────────────────────────────────────
        ax2 = ax.twinx()
        ax2.spines[["top"]].set_visible(False)
        lE, = ax2.plot(vals, Es, "s--", color=col_e, lw=2, ms=6,
                       label=r"$\eta$ (%)", zorder=3)
        ax2.set_ylabel("Efficiency (%)", color=col_e, fontsize=8)
        ax2.tick_params(axis="y", labelcolor=col_e)

        ax.set_xlabel(pname, fontsize=9)
        ax.set_title(pname, fontsize=9, fontweight="semibold")
        ax.grid(True, color="#DDDDDD", lw=0.6)

        # mark the baseline value with a grey dotted line
        ax.axvline(float(base_args[idx]), color="gray",
                   lw=1.2, ls=":", alpha=0.8)

        # legend on the first panel only
        if ax == axes.flat[0]:
            ax.legend([lP, lE], [lP.get_label(), lE.get_label()],
                      fontsize=7, loc="upper left", framealpha=0.85)

    plt.tight_layout(rect=[0, 0, 1, 0.91])
    path = os.path.join(OUT_DIR, "sensitivity_sweep.png")
    fig.savefig(path, dpi=150, facecolor="#F8F9FA")
    plt.close(fig)
    print(f"  Saved → {path}")


# ═══════════════════════════════════════════════════════════════════════════════
#  FIGURE 3 — B-H CURVE & EFFECTIVE PERMEABILITY (saturation analysis)
#  Self-contained; no dependency on motor_simulation.py
# ═══════════════════════════════════════════════════════════════════════════════

def plot_saturation():
    """
    Two-panel figure:
      Left  — B-H curve of M19 steel with design operating points marked
      Right — Effective relative permeability vs. B
    """
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    fig.patch.set_facecolor("#F8F9FA")

    # ── Left panel: B-H curve ─────────────────────────────────────────────────
    ax = axes[0]
    ax.set_facecolor("white")
    ax.spines[["top", "right"]].set_visible(False)

    ax.plot(BH_H, BH_B, "b-", lw=2.5, label="M19 Electrical Steel")
    ax.axhspan(1.5, 2.05, alpha=0.08, color="red",
               label="Saturation region (B > 1.5 T)")
    ax.axhline(1.5, color="red", lw=1.2, ls="--", alpha=0.8)
    ax.set_xscale("log")
    ax.set_xlim(10, 1e5)
    ax.set_ylim(0, 2.05)
    ax.set_xlabel("H (A/m)  — log scale", fontsize=9)
    ax.set_ylabel("B (T)", fontsize=9)
    ax.set_title("B–H Curve: Laminated Electrical Steel",
                 fontsize=10, fontweight="semibold")
    ax.legend(fontsize=8)
    ax.grid(True, color="#DDDDDD", lw=0.6)

    # operating points for three designs
    design_points = [
        ("Ferrite base\n$B_\\mathrm{core}=0.40$ T", 0.403, "#4CAF50"),
        ("NdFeB L20\n$B_\\mathrm{core}=1.61$ T",    1.609, "#F44336"),
        ("NdFeB L15\n$B_\\mathrm{core}=1.22$ T",    1.220, "#FF9800"),
    ]
    for label, B_val, color in design_points:
        H_val = float(np.interp(B_val, BH_B, BH_H))
        ax.plot(H_val, B_val, "o", color=color, ms=9, zorder=5)
        ax.annotate(label, (H_val, B_val),
                    textcoords="offset points", xytext=(12, -4),
                    fontsize=7, color=color,
                    arrowprops=dict(arrowstyle="-", color=color, lw=0.8))

    # ── Right panel: effective permeability ───────────────────────────────────
    ax2 = axes[1]
    ax2.set_facecolor("white")
    ax2.spines[["top", "right"]].set_visible(False)

    B_range  = np.linspace(0.01, 1.95, 300)
    H_range  = np.interp(B_range, BH_B, BH_H)
    MU0      = 4e-7 * np.pi
    mu_eff   = B_range / (H_range * MU0 + 1e-12)   # relative permeability

    ax2.plot(B_range, mu_eff, color="#9C27B0", lw=2.5)
    ax2.axvspan(1.5, 2.0, alpha=0.08, color="red")
    ax2.axvline(1.5, color="red", lw=1.2, ls="--", alpha=0.8,
                label="Saturation onset (~1.5 T)")
    ax2.set_xlabel("B (T)", fontsize=9)
    ax2.set_ylabel(r"Relative permeability $\mu_r$", fontsize=9)
    ax2.set_title(r"Effective $\mu_r$ vs. Flux Density",
                  fontsize=10, fontweight="semibold")
    ax2.legend(fontsize=8)
    ax2.grid(True, color="#DDDDDD", lw=0.6)

    for label, B_val, color in design_points:
        mu_val = float(np.interp(B_val, B_range, mu_eff))
        ax2.plot(B_val, mu_val, "o", color=color, ms=9, zorder=5)
        short_label = label.split("\n")[0] + f"\n$\\mu_r$={mu_val:.0f}"
        ax2.annotate(short_label, (B_val, mu_val),
                     textcoords="offset points", xytext=(5, 10),
                     fontsize=7, color=color)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, "saturation_analysis.png")
    fig.savefig(path, dpi=150, facecolor="#F8F9FA")
    plt.close(fig)
    print(f"  Saved → {path}")


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN — reproduce all three report figures
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":

    # Make sure motor_simulation.py is on the path
    sys.path.insert(0, ".")
    import motor_simulation as ms

    print("\n=== Figure 3: Saturation Analysis ===")
    plot_saturation()

    # ── designs to plot dashboards for ────────────────────────────────────────
    dashboard_designs = [
        # (label shown in title,   input string for parse_input)
        ("Ferrite Ceramic 8",  "5, 12, 10, 45, 0.15, 0.5, 3.0"),   # D1
        ("NdFeB N52",          "1, 20, 10, 30, 0.20, 0.5, 3.0"),   # D2 high power
        ("NdFeB N52",          "1, 12, 10, 45, 0.15, 0.2, 3.0"),   # D6 thin lam
    ]
    prefixes = ["Base_Ferrite_3V",
                "High_Power_NdFeB_more_L",
                "Thin_Lam_low_eddy"]

    for (mat_name, param_str), prefix in zip(dashboard_designs, prefixes):
        print(f"\n=== Figure 1 Dashboard: {prefix} ===")
        args = ms.parse_input(param_str)
        _, _, _, res, _ = ms.run_one(*args, label=prefix, verbose=False)
        plot_dashboard(res, mat_name, prefix)

    # ── sensitivity sweep around the high-power baseline (D2) ─────────────────
    print("\n=== Figure 2: Sensitivity Sweep ===")
    base_args = ms.parse_input("1, 20, 10, 30, 0.20, 0.5, 3.0")
    plot_sensitivity(base_args, ms)

    print("\nAll figures saved to:", os.path.abspath(OUT_DIR))