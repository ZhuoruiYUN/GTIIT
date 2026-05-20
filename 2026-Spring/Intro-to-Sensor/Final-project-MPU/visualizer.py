import sys, serial, threading, queue, time
import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

PORT = '/dev/tty.usbmodem31201'   #Port for Mac, for Windows it might be 'COM3', 'COM4' etc. Check your device manager
BAUD = 115200

# ─── IMU data serial ───────────────────────────────────────────────
data_q = queue.Queue()

def serial_reader():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode().strip()
                vals = list(map(int, line.split(',')))
                if len(vals) == 6:
                    data_q.put(vals)
            except:
                pass

threading.Thread(target=serial_reader, daemon=True).start()

# ─── mathematical tools ───────────────────────────────────────────────────
def skew(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

def exp_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-8:
        return np.eye(3) + skew(w)
    K = skew(w/th)
    return np.eye(3) + np.sin(th)*K + (1-np.cos(th))*(K@K)

def rot_between(a, b):
    a, b = a/np.linalg.norm(a), b/np.linalg.norm(b)
    v = np.cross(a, b)
    c = np.dot(a, b)
    if abs(c+1) < 1e-8:
        return -np.eye(3)
    K = skew(v)
    return np.eye(3) + K + K@K/(1+c)


def load_obj(filename):
    vertices = []
    faces = []
    try:
        for line in open(filename, "r"):
            if line.startswith('#'): continue
            values = line.split()
            if not values: continue

            # 读取顶点坐标
            if values[0] == 'v':
                vertices.append(list(map(float, values[1:4])))
            # 读取面（由顶点的索引组成）
            elif values[0] == 'f':
                # OBJ文件的索引是从1开始的，Python列表是从0开始，所以要 -1
                face_vertices = [int(v.split('/')[0]) - 1 for v in values[1:]]
                # 关键修复1：将多边形拆分为三角形 (扇形三角剖分)
                for i in range(1, len(face_vertices) - 1):
                    faces.append([face_vertices[0], face_vertices[i], face_vertices[i+1]])
        return vertices, faces
    except FileNotFoundError:
        print(f"找不到模型文件: {filename}，请检查路径！")
        return [], []

# ─── IMU 状态 ───────────────────────────────────────────────────
GRAVITY = np.array([0,0,-9.80665])
N_CALIB = 250
calib_buf = []
calibrated = False
R = np.eye(3); vel = np.zeros(3); pos = np.zeros(3)
acc_bias = np.zeros(3); gyro_bias = np.zeros(3)
acc_filt = np.zeros(3)
t_prev = time.time()
trajectory = []

def process_sample(raw):
    global R, vel, pos, acc_bias, gyro_bias, acc_filt, t_prev, calibrated, calib_buf

    ax,ay,az,gx,gy,gz = raw
    acc_raw = np.array([ax,ay,az]) * 9.80665/16384
    gyro_raw = np.array([gx,gy,gz]) * np.pi/180/131

    if not calibrated:
        calib_buf.append((acc_raw, gyro_raw))
        if len(calib_buf) >= N_CALIB:
            am = np.mean([x[0] for x in calib_buf], axis=0)
            gm = np.mean([x[1] for x in calib_buf], axis=0)
            gyro_bias = gm
            R = rot_between(am, -GRAVITY)
            acc_bias = am - R.T@(-GRAVITY)
            calibrated = True
        return

    t_now = time.time()
    dt = min(max(t_now - t_prev, 1e-4), 0.02)
    t_prev = t_now

    acc = acc_raw - acc_bias
    gyro = gyro_raw - gyro_bias

    R = R @ exp_so3(gyro*dt)

    # 重力修正
    if abs(np.linalg.norm(acc)-9.80665) < 0.8 and np.linalg.norm(gyro) < 25*np.pi/180:
        up_measured = R @ acc / np.linalg.norm(acc)
        up_target = -GRAVITY/np.linalg.norm(GRAVITY)
        axis = np.cross(up_measured, up_target)
        gain = min(2.5*dt, 0.08)
        R = exp_so3(axis*gain) @ R

    # 加速度积分
    a_world = R @ acc + GRAVITY
    tau = 0.08
    alpha = np.exp(-dt/tau)
    acc_filt = (1-alpha)*a_world
    if np.linalg.norm(acc_filt) < 0.08:
        acc_filt = np.zeros(3)

    pos = pos + vel*dt + 0.5*acc_filt*dt**2
    vel = 0.995*vel + acc_filt*dt
    trajectory.append(pos.copy())
    if len(trajectory) > 300:
        trajectory.pop(0)


# ─── OpenGL virsualize ────────────────────────────────────────────────
class GLView(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        # 1. 在类初始化时加载模型，避免每帧重复读取硬盘，造成极度卡顿
        self.plane_vertices, self.plane_faces = load_obj("helicopter.obj")

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)  # 启用背面剔除
        glCullFace(GL_BACK)
        glClearColor(0.1, 0.1, 0.15, 1)
        
        # 启用光照以提升质感
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # 设置光源参数
        glLight(GL_LIGHT0, GL_POSITION, [1, 2, 1, 0])
        glLight(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1])
        glLight(GL_LIGHT0, GL_DIFFUSE, [0.7, 0.7, 0.7, 1])
        
        # 创建显示列表 (Display List) 缓存模型，极大提升性能
        self.plane_list = glGenLists(1)
        glNewList(self.plane_list, GL_COMPILE)
        self._compile_plane_geometry()
        glEndList()

    def _compile_plane_geometry(self):
        """只在初始化时编译一次模型几何体，存入显存"""
        glColor3f(0.25, 0.25, 0.25)  # (Space Gray)
        
        glEnable(GL_NORMALIZE) # 自动归一化法线
        
        glBegin(GL_TRIANGLES)
        for face in self.plane_faces:
            if len(face) >= 3:
                # 关键修复2：每次绘制面的时候计算光照法向量
                v0 = np.array(self.plane_vertices[face[0]])
                v1 = np.array(self.plane_vertices[face[1]])
                v2 = np.array(self.plane_vertices[face[2]])
                
                # 计算法线 (向量叉乘)
                normal = np.cross(v1 - v0, v2 - v0)
                norm_len = np.linalg.norm(normal)
                if norm_len > 1e-6:
                    normal = normal / norm_len
                    glNormal3f(*normal)
                
                for vertex_i in face:
                    if vertex_i < len(self.plane_vertices):
                        glVertex3f(*self.plane_vertices[vertex_i])
        glEnd()
        
        glDisable(GL_NORMALIZE)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity()
        gluPerspective(45, w / max(h, 1), 0.01, 100)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        disp_pos = pos * 3
        gluLookAt(disp_pos[0], disp_pos[1] - 5, disp_pos[2] + 3,
                  disp_pos[0], disp_pos[1], disp_pos[2], 0, 0, 1)

        # keep grid on the ground for better spatial perception
        glColor3f(0.3, 0.3, 0.3)
        glBegin(GL_LINES)
        for i in range(-5, 6):
            glVertex3f(i, -5, 0);
            glVertex3f(i, 5, 0)
            glVertex3f(-5, i, 0);
            glVertex3f(5, i, 0)
        glEnd()

        # 绘制模型
        if calibrated and self.plane_vertices:
            glPushMatrix()
            glTranslatef(*disp_pos)

            # 应用 MPU6050 算出来的旋转矩阵
            m = np.eye(4);
            m[:3, :3] = R
            glMultMatrixf(m.T.flatten())

            # --- 关键对齐与缩放区 ---
            # model scale and orientation adjustments go here
            #glScalef(0.0007, 0.0007, 0.0007)
            glScalef(0.3, 0.3, 0.3)
            # 如果飞机的机头没有对准 MPU 的 X 轴，在这里旋转它
            # 例如：绕 Z 轴转 90 度： glRotatef(90, 0, 0, 1)
            glRotate(90, 19, 19, 12)  # 根据实际模型调整这个旋转，使飞机机头朝向正 X 轴
            self._draw_imported_plane()

            glPopMatrix()

    def _draw_imported_plane(self):
        """渲染外部导入的 3D 飞机模型（直接调用预编译的显示列表）"""
        glCallList(self.plane_list)

# ─── 主窗口 ─────────────────────────────────────────────────────
app = QApplication(sys.argv)
win = QMainWindow()
win.setWindowTitle("MPU6050 real-time visualization")
win.resize(800, 600)
glview = GLView()
win.setCentralWidget(glview)
win.show()

def update():
    while not data_q.empty():
        process_sample(data_q.get())
    glview.update()

timer = QTimer()
timer.timeout.connect(update)
timer.start(16)  # ~60fps

sys.exit(app.exec())