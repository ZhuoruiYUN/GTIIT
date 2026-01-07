import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


# 1. 定义改进型 DH 变换矩阵函数 (Modified DH)
def mdh_matrix(alpha, a, d, theta):
    """
    根据 Craig 的改进型 DH 定义计算变换矩阵
    """
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])


# 2. IRB 2400 正运动学模型 (根据你的 DH Table)
def forward_kinematics(joint_angles):
    # theta_2 需要补偿 90 度
    q = joint_angles.copy()
    q[1] += np.radians(90)  # 转换为弧度

    # DH 参数表映射
    # alpha_{i-1}, a_{i-1}, d_i, theta_i
    params = [
        (0, 0, 0.615, q[0]),
        (np.pi / 2, 0.1, 0, q[1]),
        (0, 0.705, 0, q[2]),
        (np.pi / 2, 0.135, 0, q[3]),
        (-np.pi / 2, 0, 0.755, q[4]),
        (np.pi / 2, 0, 0, q[5])
    ]

    T = np.eye(4)
    joints_pos = [T[:3, 3]]  # 存储每个关节坐标系的原点
    for p in params:
        T = T @ mdh_matrix(*p)
        joints_pos.append(T[:3, 3])

    return T, np.array(joints_pos)


# 3. 简单的逆运动学数值解 (用于演示)
# 注意：这个 IK 非常简单，可能不够精确或收敛慢，仅用于演示动画。
# 对于实际应用，需要更鲁棒的 IK 算法（如解析解或更高级的数值解）。
def simple_ik(target_pos, current_q, learning_rate=0.01, iterations=500):
    q = current_q.copy()

    for _ in range(iterations):
        T, _ = forward_kinematics(q)
        current_pos = T[:3, 3]
        error = target_pos - current_pos

        if np.linalg.norm(error) < 0.001:  # 足够接近
            break

        # 估算雅可比矩阵（非常粗略的数值雅可比近似）
        # 实际应计算精确雅可比或使用解析 IK
        J_approx = np.zeros((3, 6))
        delta_q = np.radians(0.01)  # 微小关节角度变化
        for i in range(6):
            q_plus = q.copy()
            q_plus[i] += delta_q
            T_plus, _ = forward_kinematics(q_plus)

            q_minus = q.copy()
            q_minus[i] -= delta_q
            T_minus, _ = forward_kinematics(q_minus)

            J_approx[:3, i] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2 * delta_q)

        # 使用雅可比伪逆更新关节角度
        # J_inv = np.linalg.pinv(J_approx)
        # q += J_inv @ error * learning_rate

        # 更简化的更新（仅针对前三个关节，效果有限）
        q[:3] += error * learning_rate

    return q


# 4. 生成轨迹
# 初始点 (Original Point)
P_orig_cartesian = np.array([0.4, 0.4, 1.5])
q_orig = np.radians(np.array([0, 0, 0, 0, 0, 0]))  # 假设初始关节角度为0

# 圆周中心和半径
circle_center = np.array([0.5, 0.5, 1.5])
circle_radius = 0.3

# 圆周起点 (右侧最远点)
P_start_cartesian = np.array([circle_center[0] + circle_radius, circle_center[1], circle_center[2]])

# 计算从初始点到圆周起点的关节角度（需要更精确的IK）
# 这里我们用数值IK，它需要一个良好的初始猜测
# 由于simple_ik很粗糙，我们假设q_orig到q_start是直接的插值
q_start_ik = simple_ik(P_start_cartesian, q_orig)

# PTP 轨迹 (从 q_orig 到 q_start_ik)
num_ptp_steps = 50
ptp_traj_q = np.linspace(q_orig, q_start_ik, num_ptp_steps)

# 圆周焊接轨迹
num_weld_steps = 100
weld_traj_q = []
current_q_for_weld = q_start_ik.copy()  # 圆周运动的起始关节角度

for i in range(num_weld_steps):
    # 顺时针圆周
    angle = (2 * np.pi / num_weld_steps) * i
    x = circle_center[0] + circle_radius * np.cos(angle)
    y = circle_center[1] - circle_radius * np.sin(angle)  # -sin 实现顺时针
    z = circle_center[2]
    target_pos = np.array([x, y, z])

    # 每次迭代都以上一步的关节角度作为初始猜测，帮助IK收敛
    current_q_for_weld = simple_ik(target_pos, current_q_for_weld)
    weld_traj_q.append(current_q_for_weld)

weld_traj_q = np.array(weld_traj_q)

# 合并所有关节轨迹
full_joint_trajectory = np.vstack((ptp_traj_q, weld_traj_q))

# 5. 绘图与动画保存为 GIF
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-0.5, 1.5]);
ax.set_ylim([-1, 1]);
ax.set_zlim([0, 2])
ax.set_xlabel('X (m)');
ax.set_ylabel('Y (m)');
ax.set_zlabel('Z (m)')
ax.set_title('IRB 2400 Welding Simulation')

# 初始化机器人连杆线
robot_line, = ax.plot([], [], [], 'ro-', linewidth=3, markersize=5, label='IRB 2400')
# 初始化轨迹线（末端执行器路径）
end_effector_path, = ax.plot([], [], [], 'b--', alpha=0.7, label='End-Effector Path')
# 绘制目标焊接圆
circle_x = [circle_center[0] + circle_radius * np.cos(a) for a in np.linspace(0, 2 * np.pi, 100)]
circle_y = [circle_center[1] - circle_radius * np.sin(a) for a in np.linspace(0, 2 * np.pi, 100)]
circle_z = [circle_center[2]] * 100
ax.plot(circle_x, circle_y, circle_z, 'g--', alpha=0.5, label='Welding Circle')

# 记录末端执行器的轨迹
end_effector_points = []


def update(frame):
    q = full_joint_trajectory[frame]
    end_effector_T, joints_pos = forward_kinematics(q)

    robot_line.set_data(joints_pos[:, 0], joints_pos[:, 1])
    robot_line.set_3d_properties(joints_pos[:, 2])

    end_effector_points.append(end_effector_T[:3, 3])
    path_points = np.array(end_effector_points)
    end_effector_path.set_data(path_points[:, 0], path_points[:, 1])
    end_effector_path.set_3d_properties(path_points[:, 2])

    # 动态调整视图以跟踪机器人
    # ax.autoscale_view(True, True, True) # 如果需要自动缩放

    return robot_line, end_effector_path


print("正在生成动画帧...")
ani = FuncAnimation(fig, update, frames=len(full_joint_trajectory), blit=True)

print("正在保存 GIF 文件...")
# 保存动画为 GIF
# fps: 每秒帧数 (frame rate)
# dpi: 分辨率 (dots per inch)
ani.save('irb2400_welding_simulation.gif', writer='pillow', fps=20, dpi=100)
print("GIF 文件已保存为 'irb2400_welding_simulation.gif'")

plt.show()  # 显示最后一个帧（可选，保存后通常不需要实时显示）