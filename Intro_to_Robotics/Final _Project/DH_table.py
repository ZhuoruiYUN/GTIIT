import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 1. 结构参数与速度限制 (Modified DH)
DH_PARAMS = np.array([
    [0.0, 0.0, 0.615, 0.0],
    [0.100, np.pi / 2, 0.0, 0.0],
    [0.705, 0.0, 0.0, 0.0],
    [0.135, np.pi / 2, 0.755, 0.0],
    [0.0, -np.pi / 2, 0.0, 0.0],
    [0.0, np.pi / 2, 0.085, 0.0]
])

# 关节速度限制 (rad/s)
joint_speed_limits = np.deg2rad([150.0, 150.0, 150.0, 360.0, 360.0, 450.0])
link_colors = ['red', 'blue', 'green', 'orange', 'purple', 'yellow']


def dh_transform(a_prev, alpha_prev, d, theta):
    ca, sa = np.cos(alpha_prev), np.sin(alpha_prev)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([[ct, -st, 0, a_prev],
                     [st * ca, ct * ca, -sa, -sa * d],
                     [st * sa, ct * sa, ca, ca * d],
                     [0.0, 0.0, 0.0, 1.0]])


def forward_kinematics_full(q):
    T = np.eye(4)
    joints_pos = [T[:3, 3]]
    for i in range(6):
        a, alpha, d, offset = DH_PARAMS[i]
        T = T @ dh_transform(a, alpha, d, q[i] + offset)
        joints_pos.append(T[:3, 3])
    return T, np.array(joints_pos)


def ik_dls(target_pos, q_init):
    q = q_init.copy()
    lam = 0.02
    for _ in range(150):
        T, _ = forward_kinematics_full(q)
        error = target_pos - T[:3, 3]
        if np.linalg.norm(error) < 1e-4: break
        J = np.zeros((3, 6))
        eps = 1e-7
        for i in range(6):
            q_eps = q.copy();
            q_eps[i] += eps
            T_eps, _ = forward_kinematics_full(q_eps)
            J[:, i] = (T_eps[:3, 3] - T[:3, 3]) / eps
        dq = J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(3)) @ error
        q += dq
    return q


# --- 轨迹规划 ---
# 1. 初始点到起点 (限速 PTP)
P_orig = np.array([0.4, 0.4, 1.5])
P_weld_start = np.array([0.8, 0.5, 1.5])  # 圆起点 (0.5+0.3)
q_orig = ik_dls(P_orig, np.zeros(6))
q_weld_start = ik_dls(P_weld_start, q_orig)

T_ptp = np.max(np.abs(q_weld_start - q_orig) / joint_speed_limits)
dt = 0.05
traj_ptp = np.array([q_orig + (t / T_ptp) * (q_weld_start - q_orig) for t in np.arange(0, T_ptp, dt)])

# 2. 圆周焊接 (匀速 0.2 m/s)
C_center = np.array([0.5, 0.5, 1.5])
Radius = 0.3
V_target = 0.2
T_circle = (2 * np.pi * Radius) / V_target
traj_weld = []
q_curr = q_weld_start
for t in np.arange(0, T_circle, dt):
    angle = (V_target / Radius) * t
    target = np.array([C_center[0] + Radius * np.cos(angle), C_center[1] - Radius * np.sin(angle), 1.5])
    q_curr = ik_dls(target, q_curr)
    traj_weld.append(q_curr)
traj_weld = np.array(traj_weld)

full_traj = np.vstack((traj_ptp, traj_weld))

# --- 绘图与动画 ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
# 修改绘图范围为正常顺序，通过 view_init 调整视角
ax.set_xlim([-1.2, 1.2])
ax.set_ylim([-1.2, 1.2]) # 回复正常顺序
ax.set_zlim([0, 1.8])

# 关键：通过调整视角（Elevation 和 Azimuth）来获得正确的观察感
# elev=30 (仰角), azim=-60 (方位角) 是观察机器人的常用交互视角
ax.view_init(elev=30, azim=-60)

current_artists = []


def draw_robot(Ps, frame_idx):
    arts = []
    # 绘制多色连杆
    for i in range(len(Ps) - 1):
        line, = ax.plot(Ps[i:i + 2, 0], Ps[i:i + 2, 1], Ps[i:i + 2, 2], color=link_colors[i], lw=4)
        arts.append(line)
    # 绘制关节原点 (散点)
    sc = ax.scatter(Ps[:, 0], Ps[:, 1], Ps[:, 2], c='k', s=20)
    arts.append(sc)
    return arts


ee_trail = []


def update(i):
    for a in current_artists: a.remove()
    current_artists.clear()

    T, Ps = forward_kinematics_full(full_traj[i])
    current_artists.extend(draw_robot(Ps, i))

    ee_trail.append(T[:3, 3])
    trail_arr = np.array(ee_trail)
    t_line, = ax.plot(trail_arr[:, 0], trail_arr[:, 1], trail_arr[:, 2], 'k--', lw=1, alpha=0.4)
    current_artists.append(t_line)
    return current_artists


print(f"正在生成并保存 GIF (PTP时间: {T_ptp:.2f}s, 焊接时间: {T_circle:.2f}s)...")
ani = FuncAnimation(fig, update, frames=len(full_traj), interval=50)
ani.save('irb2400_full_task.gif', writer='pillow', fps=20)
plt.show()