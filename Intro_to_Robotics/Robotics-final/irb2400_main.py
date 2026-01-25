# -*- coding: utf-8 -*-
"""
Entry point for the refactored IRB2400 project.
This file preserves the original single-file behavior, but delegates to:
- irb2400_model.py
- irb2400_planning.py
- irb2400_dynamics.py
"""
from __future__ import annotations

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from irb2400_core import IRB2400, R_from_euler_xyz, T_inverse, unwrap_to_near
from irb2400_planning import TrajectoryPlanner, scurve7
from irb2400_planning import smooth_hann  # keep same behavior



class ProjectRunner:
    def __init__(self, dt: float = 0.02):
        self.dt = dt
        self.robot = IRB2400()
        self.planner = TrajectoryPlanner(dt=dt)
        

        # Joint limits (project statement)
        self.joint_speed_deg_per_s = np.array([150.0, 150.0, 150.0, 360.0, 360.0, 450.0], dtype=float)
        self.joint_accel_deg_per_s2 = np.array([150.0, 150.0, 150.0, 300.0, 300.0, 400.0], dtype=float)
        self.qd_lim = np.deg2rad(self.joint_speed_deg_per_s)
        self.qdd_lim = np.deg2rad(self.joint_accel_deg_per_s2)

    def run(self):
        # ---------- Task 1: Print DH table ----------
        print("=== Task 1: MDH table (a_{i-1}, alpha_{i-1}, d_i, theta relation) ===")
        print("Link 1: a0=0,   alpha0=0,      d1=0.615, theta1=q1")
        print("Link 2: a1=0.1, alpha1=-pi/2,  d2=0,     theta2=q2-pi/2")
        print("Link 3: a2=0.705,alpha2=0,     d3=0,     theta3=q3-q2")
        print("Link 4: a3=0.135,alpha3=-pi/2, d4=0.755, theta4=q4")
        print("Link 5: a4=0,   alpha4=pi/2,   d5=0,     theta5=q5+pi")
        print("Link 6: a5=0,   alpha5=pi/2,   d6=0,     theta6=q6")
        print("Tool spacer: a6=0, alpha6=0,   d7=0.085, theta7=0")
        print("Tool TCP extra: R_T7=Rot_y(pi/2), p_T7 configurable (default 0)\n")

        # ---------- Task 2-3: Transform and IK for original pose ----------
        p_orig = np.array([0.4, 0.4, 1.5], dtype=float)
        R_orig = R_from_euler_xyz(0.0, 0.0, 0.0)

        q_guess = np.zeros(6, dtype=float)
        q_orig, ok1, info1 = self.robot.ik_solve(q_guess, p_orig, R_orig, max_iter=300)
        print("=== Task 3: IK for original pose ===")
        print("q_orig [deg]:", np.rad2deg(q_orig))
        print("IK success:", ok1, "iters:", info1["iter"])

        _, _, T_S_T_orig = self.robot.forward_kinematics_full(q_orig)
        T_T_S_orig = T_inverse(T_S_T_orig)
        print("\n=== Task 2: Transform Tool->Station at original pose ===")
        print("T_S_T (station->tool):\n", T_S_T_orig)
        print("T_T_S (tool->station):\n", T_T_S_orig, "\n")

        # ---------- Welding circle definition ----------
        xc, yc, zc = 0.5, 0.5, 1.5
        r = 0.3
        v = 0.2  # m/s tangential (clockwise)
        omega = v / r
        T_circle = 2.0 * np.pi / omega

        p_start = np.array([xc + r, yc, zc], dtype=float)
        R_weld = R_orig.copy()

        q_start, ok2, info2 = self.robot.ik_solve(q_orig.copy(), p_start, R_weld, max_iter=300)
        q_start = unwrap_to_near(q_start, q_orig)
        print("=== Task 4: IK for start point ===")
        print("q_start [deg]:", np.rad2deg(q_start))
        print("IK success:", ok2, "iters:", info2["iter"], "\n")

        # ---------- Task 4: plan fastest move to start (sync scurve7) ----------
        t_move, q_move, qd_move, qdd_move, _ = self.planner.plan_move(q_orig, q_start, self.qd_lim, self.qdd_lim)
        print("=== Task 4: Move timing (home/orig -> start) ===")
        print("T_move [s]:", float(t_move[-1]))

        # ---------- Task 5: welding joint positions/velocities ----------
        ramp_time = 0.2
        if 2.0 * ramp_time >= T_circle:
            ramp_time = 0.0

        t_circle = np.arange(0.0, T_circle + 1e-12, self.dt)
        N = t_circle.size

        f = np.ones(N, dtype=float)
        if ramp_time > 0.0:
            for k, t in enumerate(t_circle):
                if t < ramp_time:
                    u = t / ramp_time
                    f[k] = scurve7(u)[0]
                elif t > (T_circle - ramp_time):
                    u = (T_circle - t) / ramp_time
                    f[k] = scurve7(u)[0]
                else:
                    f[k] = 1.0

        dtheta = omega * f * self.dt
        theta = np.cumsum(dtheta)
        if theta[-1] > 1e-12:
            theta *= (2.0 * np.pi / theta[-1])
        theta = -theta  # clockwise
        theta[0] = 0.0

        q_weld = np.zeros((N, 6))
        q_prev = q_start.copy()
        for k in range(N):
            ang = theta[k]
            p = np.array([xc + r * math.cos(ang), yc + r * math.sin(ang), zc], dtype=float)
            q_sol, ok, _ = self.robot.ik_solve(q_prev, p, R_weld, max_iter=120)
            q_sol = unwrap_to_near(q_sol, q_prev)
            q_weld[k] = q_sol
            q_prev = q_sol

        q_weld = smooth_hann(q_weld, win=17)
        qd_weld = np.gradient(q_weld, self.dt, axis=0, edge_order=2)
        qdd_weld = np.gradient(qd_weld, self.dt, axis=0, edge_order=2)

        if ramp_time > 0.0:
            k0 = int(max(2, round(ramp_time / self.dt)))
            qd_weld[:k0] *= 0.0
            qdd_weld[:k0] *= 0.0
            qd_weld[-k0:] *= 0.0
            qdd_weld[-k0:] *= 0.0

        # ---------- Concatenate full motion ----------
        q_full = np.vstack([q_move, q_weld[1:]])
        qd_full = np.vstack([qd_move, qd_weld[1:]])
        qdd_full = np.vstack([qdd_move, qdd_weld[1:]])
        t_full = np.hstack([t_move, t_move[-1] + t_circle[1:]])

        vmax = np.max(np.abs(qd_full), axis=0)
        amax = np.max(np.abs(qdd_full), axis=0)

        print("\n=== Max joint speed/acc over full motion ===")
        print("max |qd| [deg/s]:", np.rad2deg(vmax))
        print("limits [deg/s]:   ", self.joint_speed_deg_per_s)
        print("max |qdd| [deg/s^2]:", np.rad2deg(amax))
        print("limits [deg/s^2]:    ", self.joint_accel_deg_per_s2)

        # ---------- Task 4: end-effector in Cartesian space ----------
        ee_full = np.zeros((q_full.shape[0], 3))
        joints_full = np.zeros((q_full.shape[0], 8, 3))
        for k in range(q_full.shape[0]):
            _, Ps, T_ST = self.robot.forward_kinematics_full(q_full[k])
            joints_full[k] = Ps
            ee_full[k] = T_ST[:3, 3]

        # ---------- Task 6: torques ----------
        tau_full = np.zeros_like(q_full)
        for k in range(q_full.shape[0]):
            tau_full[k] = self.robot.newton_euler_tau(q_full[k], qd_full[k], qdd_full[k])

        tau_peak = np.max(np.abs(tau_full), axis=0)
        print("\n=== Task 6: Peak torque (approx model) ===")
        print("max |tau| [Nm]:", tau_peak)

        # ---------- Plots ----------
        fig2 = plt.figure(figsize=(11, 8))
        ax1 = fig2.add_subplot(411)
        ax2 = fig2.add_subplot(412)
        ax3 = fig2.add_subplot(413)
        ax4 = fig2.add_subplot(414)

        for j in range(6):
            ax1.plot(t_full, np.rad2deg(q_full[:, j]), label=f"J{j+1}")
        ax1.set_ylabel("Angle [deg]"); ax1.grid(True); ax1.legend(ncol=3, fontsize=8, loc="upper right")

        for j in range(6):
            ax2.plot(t_full, np.rad2deg(qd_full[:, j]), label=f"J{j+1}")
        ax2.set_ylabel("Angle Velocity [deg/s]"); ax2.grid(True); ax2.legend(ncol=3, fontsize=8, loc="upper right")

        for j in range(6):
            ax3.plot(t_full, np.rad2deg(qdd_full[:, j]), label=f"J{j+1}")
        ax3.set_ylabel("Angle Acceleration [deg/s²]"); ax3.grid(True); ax3.legend(ncol=3, fontsize=8, loc="upper right")

        for j in range(6):
            ax4.plot(t_full, tau_full[:, j], label=f"J{j+1}")
        ax4.set_ylabel("tau [Nm]"); ax4.set_xlabel("t [s]"); ax4.grid(True); ax4.legend(ncol=3, fontsize=8, loc="upper right")

        fig2.tight_layout()

        # ---------- Task 7: Animation ----------
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlim([-0.3, 1.2])
        ax.set_ylim([-0.3, 1.2])
        ax.set_zlim([0.0, 2.0])
        ax.set_title("ABB IRB2400/10 (fixed kinematics)")

        xs = ee_full[:, 0]; ys = ee_full[:, 1]; zs = ee_full[:, 2]
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

        link_colors = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown", "tab:pink"]
        robot_lines = []
        joint_pts = []
        ee_line, = ax.plot([], [], [], linewidth=2)
        ee_traj = []

        def init():
            return []

        def update(frame):
            nonlocal robot_lines, joint_pts, ee_traj
            for ln in robot_lines:
                ln.remove()
            for pt in joint_pts:
                pt.remove()
            robot_lines = []
            joint_pts = []

            Ps = joints_full[frame]
            for i in range(Ps.shape[0] - 1):
                p1, p2 = Ps[i], Ps[i+1]
                ln, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                              linewidth=3, color=link_colors[i])
                robot_lines.append(ln)
            for p in Ps:
                pt = ax.scatter(p[0], p[1], p[2], c="k", s=10)
                joint_pts.append(pt)

            ee_traj.append(ee_full[frame].copy())
            if len(ee_traj) > 1:
                A = np.array(ee_traj)
                ee_line.set_data(A[:, 0], A[:, 1])
                ee_line.set_3d_properties(A[:, 2])

            ax.view_init(elev=25, azim=45 + 0.2 * frame)
            return robot_lines + joint_pts + [ee_line]

        _ani = animation.FuncAnimation(
            fig, update, frames=joints_full.shape[0],
            interval=int(self.dt * 1000), blit=False, init_func=init
        )
        plt.show()


def main():
    ProjectRunner(dt=0.02).run()


if __name__ == "__main__":
    main()
