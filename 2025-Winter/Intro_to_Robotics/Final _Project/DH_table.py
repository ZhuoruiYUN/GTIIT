import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 1. 机器人参数与 DH 模型 ---
# IRB 2400 Modified DH (a, alpha, d, theta_offset)
DH_PARAMS = np.array([
    [0.0, 0.0, 0.615, 0.0],
    [0.100, np.pi / 2, 0.0, 0.0],
    [0.705, 0.0, 0.0, 0.0],
    [0.135, np.pi / 2, 0.755, 0.0],
    [0.0, -np.pi / 2, 0.0, 0.0],
    [0.0,  np.pi / 2, 0.085, 0.0]
])

JOINT_SPEED_LIMITS = np.deg2rad([150.0, 150.0, 150.0, 360.0, 360.0, 450.0])
DT = 0.02  # 提高采样频率以获得更精确的加速度


def dh_matrix(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0, 0, 0, 1]
    ])


def get_fk(q):
    T = np.eye(4)
    positions = [T[:3, 3]]
    for i in range(6):
        T = T @ dh_matrix(DH_PARAMS[i, 0], DH_PARAMS[i, 1], DH_PARAMS[i, 2], q[i] + DH_PARAMS[i, 3])
        positions.append(T[:3, 3])
    return T, np.array(positions)


def ik_dls(target_pos, q_guess):
    q = q_guess.copy()
    for _ in range(100):
        T, _ = get_fk(q)
        err = target_pos - T[:3, 3]
        if np.linalg.norm(err) < 1e-5: break

        # 数值雅可比
        J = np.zeros((3, 6))
        eps = 1e-8
        for i in range(6):
            q_eps = q.copy();
            q_eps[i] += eps
            T_eps, _ = get_fk(q_eps)
            J[:, i] = (T_eps[:3, 3] - T[:3, 3]) / eps

        # Damped Least Squares
        dq = J.T @ np.linalg.inv(J @ J.T + 0.01 ** 2 * np.eye(3)) @ err
        q += dq
    return q


# --- 2. 任务定义 ---
P_orig = np.array([0.4, 0.4, 1.5])
C_center = np.array([0.5, 0.5, 1.5])
R = 0.3
P_start = C_center + np.array([R, 0, 0])  # 右侧起始点: [0.8, 0.5, 1.5]

# --- 3. 阶段 1: 初始点 -> 起始点 (Time Optimal LIN) ---
q_orig = ik_dls(P_orig, np.array([0, 0.1, 0.1, 0, 0, 0]))
q_start = ik_dls(P_start, q_orig)

# 计算“尽快到达”的时间 (根据关节速度限制)
dq_needed = np.abs(q_start - q_orig)
t_min = np.max(dq_needed / JOINT_SPEED_LIMITS)
T_lin = t_min * 1.2  # 预留20%余量保证加速度可行性

t_steps_lin = np.arange(0, T_lin, DT)
traj_q_lin = []
traj_p_lin = []

for t in t_steps_lin:
    s = t / T_lin
    p_interp = P_orig + s * (P_start - P_orig)
    q_interp = ik_dls(p_interp, q_orig if t == 0 else traj_q_lin[-1])
    traj_q_lin.append(q_interp)
    traj_p_lin.append(p_interp)

# --- 4. 阶段 2: 顺时针圆周焊接 ---
v_weld = 0.2
t_circle_total = (2 * np.pi * R) / v_weld
t_steps_weld = np.arange(0, t_circle_total, DT)

traj_q_weld = []
traj_p_weld = []

for t in t_steps_weld:
    # 顺时针: angle 从 0 开始减小
    angle = -(v_weld / R) * t
    p_circle = np.array([
        C_center[0] + R * np.cos(angle),
        C_center[1] + R * np.sin(angle),
        C_center[2]
    ])
    q_circle = ik_dls(p_circle, traj_q_lin[-1] if t == 0 else traj_q_weld[-1])
    traj_q_weld.append(q_circle)
    traj_p_weld.append(p_circle)

# --- 5. 数据分析 (速度与加速度) ---
full_q = np.vstack((traj_q_lin, traj_q_weld))
full_v = np.diff(full_q, axis=0) / DT
full_a = np.diff(full_v, axis=0) / DT

print(f"初始点关节角 (deg): {np.rad2deg(q_orig)}")
print(f"到达起点所需时间: {T_lin:.2f} s")
print(f"最大关节速度 (rad/s): {np.max(np.abs(full_v), axis=0)}")
print(f"最大关节加速度 (rad/s²): {np.max(np.abs(full_a), axis=0)}")

# --- 6. 动画制作 ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([0, 1.2]);
ax.set_ylim([0, 1.2]);
ax.set_zlim([0, 1.8])
ax.view_init(elev=30, azim=45)

links, = ax.plot([], [], [], 'o-', lw=4, color='orange')
trail, = ax.plot([], [], [], 'k--', alpha=0.4)
history_p = []


def update(i):
    T, Ps = get_fk(full_q[i])
    links.set_data(Ps[:, 0], Ps[:, 1])
    links.set_3d_properties(Ps[:, 2])

    history_p.append(Ps[-1])
    h_arr = np.array(history_p)
    trail.set_data(h_arr[:, 0], h_arr[:, 1])
    trail.set_3d_properties(h_arr[:, 2])
    return links, trail


ani = FuncAnimation(fig, update, frames=len(full_q), interval=DT * 1000, blit=True)
plt.title("IRB 2400 Clockwise Circular Welding Task")
plt.show()