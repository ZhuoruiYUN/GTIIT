# =========================
# ABB IRB2400/10 Project - Full Reference Script
# (FK, IK, joint-space trajectory, circle welding path, dynamics (numeric), animation)
# =========================
# Source: Project_25.pdf (provided by you)   [oai_citation:0‡Project_25.pdf](sediment://file_000000006d7c722f9baef8fc7d4cd63d)

import numpy as np
import math

import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ---------------------------
# DH (thesis-style alpha signs) & ABB limits (from your PDF snippet)
# ---------------------------
DH = np.array([
    [0.0,   0.0,        0.615, 0.0],
    [0.100, np.pi/2,   0.0,   0.0],
    [0.705, 0.0,        0.0,   0.0],
    [0.135, np.pi/2,    0.755, 0.0],
    [0.0,   -np.pi/2,    0.0,   0.0],
    [0.0,  np.pi/2,    0.085, 0.0],
], dtype=float)

joint_speed_deg_per_s = np.array([150.0, 150.0, 150.0, 360.0, 360.0, 450.0])
joint_speed_limit = np.deg2rad(joint_speed_deg_per_s)  # rad/s
g = 9.81


# ---------------------------
# Link mass/COM/inertia (simple approximations for numeric dynamics)
# ---------------------------
link_masses = np.array([60.0, 40.0, 30.0, 20.0, 10.0, 5.0], dtype=float)

# approximate link lengths suggested by your PDF note
link_lengths = np.array([DH[0, 0], DH[1, 0], DH[2, 0], DH[3, 0], 0.08, 0.05], dtype=float)

# COM in each link local frame (very rough): half-length along +x
link_com_local = np.array([[L * 0.5, 0.0, 0.0] for L in link_lengths], dtype=float)

# Diagonal inertia approximation around COM: (m L^2 / 12) * I
inertias = []
for m, L in zip(link_masses, link_lengths):
    I = (m * (L ** 2) / 12.0) * np.eye(3)
    inertias.append(I)
inertias = np.array(inertias, dtype=float)


# ---------------------------
# Kinematics
# ---------------------------
def dh_transform(a_prev, alpha_prev, d, theta):
    """
    Build modified DH homogeneous transform A_i(a_prev, alpha_prev, d, theta)
    Returns 4x4 mapping frame i-1 to frame i.
    """
    ca = np.cos(alpha_prev)
    sa = np.sin(alpha_prev)
    ct = np.cos(theta)
    st = np.sin(theta)

    return np.array([
        [ct,        -st,        0.0,  a_prev],
        [st * ca,   ct * ca,   -sa,  -sa * d],
        [st * sa,   ct * sa,    ca,   ca * d],
        [0.0,        0.0,       0.0,  1.0],
    ], dtype=float)


def forward_kinematics_full(q):
    """
    Compute full chain transforms for q (6,).
    Returns:
      Ts        : list of 4x4 transforms T_0_i for i=0..6 (T_0_0 = I)
      positions : (7,3) origins of each frame
      R_end     : (3,3) end-effector rotation
    """
    T = np.eye(4)
    Ts = [T.copy()]
    for i in range(6):
        a, alpha, d, theta_offset = DH[i]
        theta = q[i] + theta_offset
        A = dh_transform(a, alpha, d, theta)
        T = T @ A
        Ts.append(T.copy())
    positions = np.array([Ti[:3, 3] for Ti in Ts], dtype=float)
    return Ts, positions, Ts[-1][:3, :3]


def pose_to_vector(T):
    """
    Convert 4x4 transform -> 6-vector [x,y,z, rvec] where rvec is axis-angle vector.
    """
    p = T[:3, 3]
    R = T[:3, :3]
    angle = np.arccos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2.0)))
    if abs(angle) < 1e-8:
        rvec = np.zeros(3)
    else:
        rx = (R[2, 1] - R[1, 2]) / (2 * np.sin(angle))
        ry = (R[0, 2] - R[2, 0]) / (2 * np.sin(angle))
        rz = (R[1, 0] - R[0, 1]) / (2 * np.sin(angle))
        rvec = angle * np.array([rx, ry, rz], dtype=float)
    return np.hstack((p, rvec))


def numeric_jacobian(q, eps=1e-6):
    """
    Numeric 6x6 Jacobian via finite differences: qdot -> [v, rvec_dot] approx.
    """
    Ts, _, _ = forward_kinematics_full(q)
    y0 = pose_to_vector(Ts[-1])
    J = np.zeros((6, 6), dtype=float)
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        Tq, _, _ = forward_kinematics_full(dq)
        y1 = pose_to_vector(Tq[-1])
        J[:, i] = (y1 - y0) / eps
    return J


def ik_solve(q_init, target_pos, target_R, steps=200, K=1.2, eps_jac=1e-6):
    """
    Damped least squares IK (position + orientation).
    Returns (q, success).
    """
    q = q_init.copy()
    lam = 1e-3

    # target 6-vector [pos, rvec(target_R)]
    Tt = np.eye(4)
    Tt[:3, :3] = target_R
    Tt[:3, 3] = np.array([0.0, 0.0, 0.0])
    ytar = np.hstack((target_pos, pose_to_vector(Tt)[-3:]))

    for _ in range(steps):
        Ts, _, _ = forward_kinematics_full(q)
        ycur = pose_to_vector(Ts[-1])
        err = ytar - ycur

        if np.linalg.norm(err[:3]) < 1e-6 and np.linalg.norm(err[3:]) < 1e-4:
            return q, True

        J = numeric_jacobian(q, eps=eps_jac)
        JJ = J @ J.T + lam * np.eye(6)
        dq = J.T @ np.linalg.solve(JJ, err)
        q = q + K * dq

    return q, False


# ---------------------------
# Quintic joint trajectory helpers
# ---------------------------
def quintic_coeffs(q0, qf, v0, vf, a0, af, T):
    M = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5],
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ], dtype=float)
    b = np.array([q0, v0, a0, qf, vf, af], dtype=float)
    return np.linalg.solve(M, b)


def quintic_eval(a, t):
    return a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5


def quintic_eval_dot(a, t):
    return a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4


def quintic_eval_ddot(a, t):
    return 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3


# ---------------------------
# Numeric dynamics (from your PDF snippet; fixed indentation & symmetry placement)
# ---------------------------
def compute_M_numeric(q):
    """
    Numerically estimate inertia matrix M(q) by finite differences + kinetic energy.
    Very expensive but OK for a demo/assignment.
    """
    n = 6
    M = np.zeros((n, n), dtype=float)
    dt = 1e-6

    Ts, _, _ = forward_kinematics_full(q)

    # Precompute KE for each basis e_i (needed for cross-term)
    KE_basis = np.zeros(n, dtype=float)
    for i in range(n):
        qd_i = np.zeros(n); qd_i[i] = 1.0
        q_plus = q + qd_i * dt
        Ts_p, _, _ = forward_kinematics_full(q_plus)

        KE_i = 0.0
        for k in range(6):
            Tlk = Ts[k + 1]
            Tlkp = Ts_p[k + 1]

            Rl = Tlk[:3, :3]
            Rlp = Tlkp[:3, :3]
            Rrel = Rl.T @ Rlp

            ang = 0.5 * np.array([
                Rrel[2, 1] - Rrel[1, 2],
                Rrel[0, 2] - Rrel[2, 0],
                Rrel[1, 0] - Rrel[0, 1],
            ], dtype=float) / dt

            com_local = np.hstack((link_com_local[k], 1.0))
            com_w  = Tlk  @ com_local
            com_wp = Tlkp @ com_local
            vcom = (com_wp[:3] - com_w[:3]) / dt

            m = link_masses[k]
            Ilink = inertias[k]
            KE_i += 0.5 * m * (vcom @ vcom) + 0.5 * (ang @ (Ilink @ ang))

        KE_basis[i] = KE_i

    # Cross-term trick for M_ij
    for i in range(n):
        for j in range(n):
            qd_ij = np.zeros(n); qd_ij[i] = 1.0; qd_ij[j] += 1.0
            q_plus2 = q + qd_ij * dt
            Ts_p2, _, _ = forward_kinematics_full(q_plus2)

            KE_ij = 0.0
            for k in range(6):
                Tlk = Ts[k + 1]
                Tlk2 = Ts_p2[k + 1]

                Rl = Tlk[:3, :3]
                Rl2 = Tlk2[:3, :3]
                Rrel2 = Rl.T @ Rl2

                ang2 = 0.5 * np.array([
                    Rrel2[2, 1] - Rrel2[1, 2],
                    Rrel2[0, 2] - Rrel2[2, 0],
                    Rrel2[1, 0] - Rrel2[0, 1],
                ], dtype=float) / dt

                com_local = np.hstack((link_com_local[k], 1.0))
                com_w  = Tlk  @ com_local
                com_w2 = Tlk2 @ com_local
                vcom2 = (com_w2[:3] - com_w[:3]) / dt

                m = link_masses[k]
                Ilink = inertias[k]
                KE_ij += 0.5 * m * (vcom2 @ vcom2) + 0.5 * (ang2 @ (Ilink @ ang2))

            M[i, j] = (KE_ij - KE_basis[i] - KE_basis[j]) * 2.0

    M = 0.5 * (M + M.T)
    return M


def compute_gravity_torques(q):
    """
    Gravity generalized forces G(q) via numerical gradient of potential energy.
    """
    Ts, _, _ = forward_kinematics_full(q)
    U = 0.0
    for k in range(6):
        com_local = np.hstack((link_com_local[k], 1.0))
        com_w = Ts[k + 1] @ com_local
        U += link_masses[k] * g * com_w[2]

    n = 6
    Gq = np.zeros(n, dtype=float)
    eps = 1e-6
    for i in range(n):
        dq = np.zeros(n); dq[i] = eps
        Ts_p, _, _ = forward_kinematics_full(q + dq)
        Uplus = 0.0
        for k in range(6):
            com_local = np.hstack((link_com_local[k], 1.0))
            com_w = Ts_p[k + 1] @ com_local
            Uplus += link_masses[k] * g * com_w[2]
        Gq[i] = (Uplus - U) / eps
    return Gq


def compute_Cqd_numeric(q, qd):
    """
    Approximate C(q,qd) by dp/dt where p = M(q) qd, using M(q+qd*dt).
    """
    dt = 1e-6
    M0 = compute_M_numeric(q)
    p0 = M0 @ qd
    q_plus = q + qd * dt
    M1 = compute_M_numeric(q_plus)
    p1 = M1 @ qd
    dpdt = (p1 - p0) / dt
    return dpdt


def compute_tau(q, qd, qdd):
    """
    tau = M(q) qdd + C(q,qd) + G(q)
    """
    Mq = compute_M_numeric(q)
    Gq = compute_gravity_torques(q)
    Cqd = compute_Cqd_numeric(q, qd)
    return Mq @ qdd + Cqd + Gq


# ---------------------------
# Animation helpers (from your PDF snippet)
# ---------------------------
def remove_artists(artists_list):
    for art in artists_list:
        try:
            art.remove()
        except Exception:
            try:
                ax.collections.remove(art)
            except Exception:
                pass
    artists_list.clear()


def draw_robot_at_pos(Ps):
    created = []
    for i in range(len(Ps) - 1):
        p1 = Ps[i]
        p2 = Ps[i + 1]
        line, = ax.plot(
            [p1[0], p2[0]],
            [p1[1], p2[1]],
            [p1[2], p2[2]],
            color=link_colors[i],
            linewidth=3
        )
        created.append(line)

    for p in Ps:
        sc = ax.scatter(p[0], p[1], p[2], c='k', s=10)
        created.append(sc)

    return created


def update_anim(frame):
    remove_artists(current_artists)

    step = min(frame, positions_full.shape[0] - 1)
    Ps = positions_full[step]

    new_arts = draw_robot_at_pos(Ps)
    current_artists.extend(new_arts)

    ee = Ps[-1]
    ee_traj.append(ee.copy())
    if len(ee_traj) > 1:
        eeA = np.array(ee_traj)
        line, = ax.plot(eeA[:, 0], eeA[:, 1], eeA[:, 2], 'k-', linewidth=1.2)
        current_artists.append(line)

    ax.view_init(elev=25, azim=45 + frame * view_rotation_velo)
    return current_artists


def rotm_to_euler_xyz(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0.0
    return np.array([x, y, z], dtype=float)


# ---------------------------
# Main: build trajectory (home -> circle start) + welding circle
# ---------------------------
def main():
    # ---------- Task definitions (from your PDF statement) ----------
    global ani
    # Original pose in station frame:
    # [x,y,z, α,β,γ] = [0.4, 0.4, 1.5, 0,0,0]
    # We'll use identity orientation (R=I) as a reasonable default for α=β=γ=0.
    p_orig = np.array([0.4, 0.4, 1.5], dtype=float)
    R_orig = np.eye(3)

    # Circle in xy plane:
    xc, yc, zc = 0.5, 0.5, 1.5
    r = 0.3
    v = 0.2  # m/s (tangential), clockwise
    omega = v / r  # rad/s

    # start point = right-most point
    p_start = np.array([xc + r, yc, zc], dtype=float)
    R_weld = np.eye(3)

    # ---------- IK for original & start ----------
    q_guess = np.zeros(6, dtype=float)

    q_orig, ok1 = ik_solve(q_guess, p_orig, R_orig, steps=400)
    if not ok1:
        print("[WARN] IK for original pose did not fully converge. Using best effort.")

    q_start, ok2 = ik_solve(q_orig.copy(), p_start, R_weld, steps=400)
    if not ok2:
        print("[WARN] IK for start pose did not fully converge. Using best effort.")

    # ---------- Plan joint-space quintic from q_orig to q_start as fast as possible under speed limits ----------
    dq = np.abs(q_start - q_orig)
    # Minimal time to satisfy |qd| <= limit (rough lower bound)
    T_min = np.max(dq / (joint_speed_limit + 1e-12))
    T_move = max(1.0, 1.10 * T_min)  # add 10% margin, at least 1s

    dt = 0.05  # sampling
    t_move = np.arange(0.0, T_move + 1e-12, dt)

    coeffs = np.zeros((6, 6), dtype=float)
    for i in range(6):
        coeffs[i] = quintic_coeffs(q_orig[i], q_start[i], 0.0, 0.0, 0.0, 0.0, T_move)

    q_move = np.zeros((len(t_move), 6), dtype=float)
    qd_move = np.zeros_like(q_move)
    qdd_move = np.zeros_like(q_move)
    for k, t in enumerate(t_move):
        for i in range(6):
            a = coeffs[i]
            q_move[k, i] = quintic_eval(a, t)
            qd_move[k, i] = quintic_eval_dot(a, t)
            qdd_move[k, i] = quintic_eval_ddot(a, t)

    # ---------- Welding circle path in Cartesian then IK sequentially ----------
    T_circle = 2.0 * np.pi / omega
    t_circle = np.arange(0.0, T_circle + 1e-12, dt)

    q_weld = np.zeros((len(t_circle), 6), dtype=float)
    qd_weld = np.zeros_like(q_weld)
    qdd_weld = np.zeros_like(q_weld)

    q_prev = q_start.copy()
    for k, t in enumerate(t_circle):
        # clockwise: angle decreases
        ang = -omega * t
        px = xc + r * np.cos(ang)
        py = yc + r * np.sin(ang)
        p = np.array([px, py, zc], dtype=float)

        q_sol, ok = ik_solve(q_prev, p, R_weld, steps=200)
        if not ok and k == 0:
            print("[WARN] IK on welding path start not converged; continuing best effort.")
        q_weld[k] = q_sol
        q_prev = q_sol

    # finite difference qd/qdd for welding (numerical)
    qd_weld[1:] = (q_weld[1:] - q_weld[:-1]) / dt
    qd_weld[0] = qd_weld[1]
    qdd_weld[1:] = (qd_weld[1:] - qd_weld[:-1]) / dt
    qdd_weld[0] = qdd_weld[1]

    # ---------- Concatenate full motion ----------
    q_full = np.vstack([q_move, q_weld])
    qd_full = np.vstack([qd_move, qd_weld])
    qdd_full = np.vstack([qdd_move, qdd_weld])
    t_full = np.arange(0.0, q_full.shape[0]*dt, dt)

    # ---------- FK positions for animation ----------
    global positions_full
    positions_full = np.zeros((q_full.shape[0], 7, 3), dtype=float)
    for k in range(q_full.shape[0]):
        _, Ps, _ = forward_kinematics_full(q_full[k])
        positions_full[k] = Ps

    # ---------- Quick check max speed/acc ----------
    vmax = np.max(np.abs(qd_full), axis=0)
    amax = np.max(np.abs(qdd_full), axis=0)
    print("Move time (home->start):", T_move, "s")
    print("Circle time:", T_circle, "s")
    print("Max joint speed (deg/s):", np.rad2deg(vmax))
    print("Speed limits (deg/s):    ", joint_speed_deg_per_s)
    print("Max joint accel (deg/s^2):", np.rad2deg(amax))

    # ---------- (Optional) Dynamics torques (VERY SLOW) ----------
    # If you want torques, set this True (expect heavy compute).
    compute_torque = True
    tau_full = None
    if compute_torque:
        tau_full = np.zeros_like(q_full)
        for k in range(q_full.shape[0 ]):
            tau_full[k] = compute_tau(q_full[k], qd_full[k], qdd_full[k])
        print("Torque computed.")

    # ---------- Plot (angles / speeds / torques placeholder) ----------
    fig2 = plt.figure(figsize=(10, 7))
    ax1 = fig2.add_subplot(311)
    ax2 = fig2.add_subplot(312)
    ax3 = fig2.add_subplot(313)

    ax1.plot(t_full, np.rad2deg(q_full))
    ax1.set_ylabel("Angle [deg]")
    ax1.grid(True)

    ax2.plot(t_full, np.rad2deg(qd_full))
    ax2.set_ylabel("Angle vel [deg/s]")
    ax2.grid(True)

    if tau_full is None:
        ax3.plot(t_full, np.zeros_like(t_full))
        ax3.set_ylabel("Torque [Nm] (not computed)")
    else:
        ax3.plot(t_full, tau_full)
        ax3.set_ylabel("Torque [Nm]")
    ax3.set_xlabel("time [s]")
    ax3.grid(True)

    fig2.tight_layout()

    # ---------- Animation ----------
    global fig, ax, link_colors, view_rotation_velo, current_artists, ee_traj

    link_colors = ['r', 'g', 'b', 'c', 'm', 'y']
    view_rotation_velo = 0.5
    current_artists = []
    ee_traj = []

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_xlim([-0.2, 1.2])
    ax.set_ylim([-0.2, 1.2])
    ax.set_zlim([0.0, 2.0])
    ax.set_title('ABB IRB2400 Motion Animation')

    ani = animation.FuncAnimation(
        fig,
        update_anim,
        frames=positions_full.shape[0],
        interval=int(dt * 1000),
        blit=False
    )

    print("正在保存 GIF，请稍候...")
    ani.save('irb2400_welding_task.gif', writer='pillow', fps=int(1 / dt))
    print("GIF 已保存为 'irb2400_welding_task.gif'")

    plt.show()


if __name__ == "__main__":
    main()
