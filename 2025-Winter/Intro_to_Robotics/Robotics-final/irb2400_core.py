# -*- coding: utf-8 -*-
"""
IRB2400 core module: kinematics + IK + dynamics (Newton–Euler) + default parameters.

This is a pure packaging refactor of the original single-file script:
- algorithms and numeric settings are preserved;
- only reorganized into a class-based API.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple, Dict, Optional

import numpy as np


# ---------------------------
# Small SO(3)/SE(3) utilities
# ---------------------------
def skew(w: np.ndarray) -> np.ndarray:
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy],
                     [wz, 0.0, -wx],
                     [-wy, wx, 0.0]], dtype=float)


def rotvec_from_R(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
    cos_th = (tr - 1.0) * 0.5
    cos_th = min(1.0, max(-1.0, cos_th))
    th = math.acos(cos_th)
    if th < 1e-12:
        return np.zeros(3, dtype=float)
    w_hat = (R - R.T) / (2.0 * math.sin(th))
    return th * np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]], dtype=float)


def R_from_euler_xyz(alpha: float, beta: float, gamma: float) -> np.ndarray:
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    cg, sg = math.cos(gamma), math.sin(gamma)
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, ca, -sa],
                   [0.0, sa, ca]], dtype=float)
    Ry = np.array([[cb, 0.0, sb],
                   [0.0, 1.0, 0.0],
                   [-sb, 0.0, cb]], dtype=float)
    Rz = np.array([[cg, -sg, 0.0],
                   [sg, cg, 0.0],
                   [0.0, 0.0, 1.0]], dtype=float)
    return Rx @ Ry @ Rz


def mdh_transform(a_im1: float, alpha_im1: float, d_i: float, theta_i: float) -> np.ndarray:
    ct, st = math.cos(theta_i), math.sin(theta_i)
    ca, sa = math.cos(alpha_im1), math.sin(alpha_im1)
    return np.array([
        [ct, -st, 0.0, a_im1],
        [st * ca, ct * ca, -sa, -sa * d_i],
        [st * sa, ct * sa, ca, ca * d_i],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=float)


def T_inverse(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


def wrap_to_pi(q: np.ndarray) -> np.ndarray:
    return (q + np.pi) % (2.0 * np.pi) - np.pi


def unwrap_to_near(q: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
    qn = q.copy()
    for i in range(q.size):
        while qn[i] - q_ref[i] > np.pi:
            qn[i] -= 2.0 * np.pi
        while qn[i] - q_ref[i] < -np.pi:
            qn[i] += 2.0 * np.pi
    return qn


# ---------------------------
# Parameters
# ---------------------------
@dataclass
class IRB2400Params:
    DH: np.ndarray
    R_T7: np.ndarray
    p_T7: np.ndarray
    T_SB: np.ndarray  # station->base

    # Dynamics (heuristic, per project statement)
    g: float = 9.81
    link_masses: np.ndarray = None
    link_lengths: np.ndarray = None
    link_com_local: np.ndarray = None
    inertias_local: np.ndarray = None


def build_default_params() -> IRB2400Params:
    DH = np.array([
        [0.0, 0.0, 0.615, 0.0],
        [0.1, -np.pi / 2.0, 0.0, -np.pi / 2.0],
        [0.705, 0.0, 0.0, 0.0],
        [0.135, -np.pi / 2.0, 0.755, 0.0],
        [0.0, np.pi / 2.0, 0.0, np.pi],
        [0.0, np.pi / 2.0, 0.0, 0.0],
        [0.0, 0.0, 0.085, 0.0],
    ], dtype=float)

    cy, sy = math.cos(np.pi / 2.0), math.sin(np.pi / 2.0)
    R_T7 = np.array([[cy, 0.0, sy],
                     [0.0, 1.0, 0.0],
                     [-sy, 0.0, cy]], dtype=float)
    p_T7 = np.zeros(3, dtype=float)
    T_SB = np.eye(4, dtype=float)

    params = IRB2400Params(DH=DH, R_T7=R_T7, p_T7=p_T7, T_SB=T_SB)

    # Dynamics defaults (same heuristic as original)
    params.link_masses = np.array([60.0, 40.0, 30.0, 20.0, 10.0, 5.0], dtype=float)
    params.link_lengths = np.array([DH[0, 2], DH[2, 0], DH[3, 0], DH[3, 2], 0.08, 0.05], dtype=float)
    params.link_com_local = np.array([[0.5 * L, 0.0, 0.0] for L in params.link_lengths], dtype=float)

    inertias = []
    for m, L in zip(params.link_masses, params.link_lengths):
        Ix = 1e-4 * m * (L**2 + 1e-12)
        Iy = m * (L**2) / 12.0
        Iz = Iy
        inertias.append(np.diag([Ix, Iy, Iz]))
    params.inertias_local = np.array(inertias, dtype=float)

    return params


# ---------------------------
# Core model class
# ---------------------------
class IRB2400:
    def __init__(self, params: Optional[IRB2400Params] = None):
        self.params = params if params is not None else build_default_params()

    @property
    def DH(self) -> np.ndarray:
        return self.params.DH

    def forward_kinematics_full(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        q = np.asarray(q, dtype=float).reshape(6,)
        DH = self.DH

        th = np.zeros(7, dtype=float)
        th[0] = q[0] + DH[0, 3]
        th[1] = q[1] + DH[1, 3]
        th[2] = (q[2] - q[1]) + DH[2, 3]  # special relation
        th[3] = q[3] + DH[3, 3]
        th[4] = q[4] + DH[4, 3]
        th[5] = q[5] + DH[5, 3]
        th[6] = 0.0 + DH[6, 3]

        T = self.params.T_SB.copy()
        Ts = np.zeros((8, 4, 4), dtype=float)
        Ps = np.zeros((8, 3), dtype=float)
        Ts[0] = T
        Ps[0] = T[:3, 3].copy()

        for i in range(7):
            a_im1, alpha_im1, d_i, _ = DH[i]
            A = mdh_transform(float(a_im1), float(alpha_im1), float(d_i), float(th[i]))
            T = T @ A
            Ts[i+1] = T
            Ps[i+1] = T[:3, 3].copy()

        T_tool = np.eye(4, dtype=float)
        T_tool[:3, :3] = self.params.R_T7
        T_tool[:3, 3] = self.params.p_T7
        T_S_T = T @ T_tool

        return Ts, Ps, T_S_T

    def numeric_jacobian_se3(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(6,)
        _, _, T0 = self.forward_kinematics_full(q)
        p0 = T0[:3, 3].copy()
        R0 = T0[:3, :3].copy()

        J = np.zeros((6, 6), dtype=float)
        for i in range(6):
            dq = np.zeros(6, dtype=float)
            dq[i] = eps
            _, _, Ti = self.forward_kinematics_full(q + dq)
            pi = Ti[:3, 3]
            Ri = Ti[:3, :3]
            dp = (pi - p0) / eps
            dR = Ri @ R0.T
            drot = rotvec_from_R(dR) / eps
            J[:3, i] = dp
            J[3:, i] = drot
        return J

    def ik_solve(
        self,
        q0: np.ndarray,
        p_des: np.ndarray,
        R_des: np.ndarray,
        max_iter: int = 300,
        lam: float = 1e-2,
        tol_p: float = 1e-5,
        tol_r: float = 1e-4
    ) -> Tuple[np.ndarray, bool, Dict[str, float]]:
        q = np.asarray(q0, dtype=float).reshape(6,).copy()
        p_des = np.asarray(p_des, dtype=float).reshape(3,)
        R_des = np.asarray(R_des, dtype=float).reshape(3, 3)

        info: Dict[str, float] = {"iter": 0}
        ok = False

        for it in range(max_iter):
            _, _, T = self.forward_kinematics_full(q)
            p = T[:3, 3]
            R = T[:3, :3]

            ep = p_des - p
            eR = R_des @ R.T
            er = rotvec_from_R(eR)
            e = np.hstack([ep, er])

            if np.linalg.norm(ep) < tol_p and np.linalg.norm(er) < tol_r:
                ok = True
                info["iter"] = it
                break

            J = self.numeric_jacobian_se3(q)
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + (lam ** 2) * np.eye(6), e)
            q = wrap_to_pi(q + dq)
            info["iter"] = it + 1

        return q, ok, info

    # ---------------------------
    # Dynamics: Newton–Euler torque
    # ---------------------------
    def newton_euler_tau(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(6)
        qd = np.asarray(qd, dtype=float).reshape(6)
        qdd = np.asarray(qdd, dtype=float).reshape(6)

        # Build transforms up to frame 6
        Ts, _, _ = self.forward_kinematics_full(q)

        R_list = []
        p_list = []
        for i in range(1, 7):
            T_im1 = Ts[i-1]
            T_i = Ts[i]
            T_im1_i = np.linalg.solve(T_im1, T_i)
            R_im1_i = T_im1_i[:3, :3]
            p_im1_i = T_im1_i[:3, 3]
            R_list.append(R_im1_i)
            p_list.append(p_im1_i)

        z = np.array([0.0, 0.0, 1.0], dtype=float)

        w = [np.zeros(3)]
        wd = [np.zeros(3)]
        gravity_0 = np.array([0.0, 0.0, -self.params.g], dtype=float)
        vd = [gravity_0.copy()]
        vcd = [None]

        for i in range(1, 7):
            R_im1_i = R_list[i-1]
            p_im1_i = p_list[i-1]
            w_im1 = w[i-1]
            wd_im1 = wd[i-1]
            vd_im1 = vd[i-1]

            w_i = R_im1_i.T @ w_im1 + z * qd[i-1]
            wd_i = (R_im1_i.T @ wd_im1
                    + z * qdd[i-1]
                    + np.cross(w_i, z * qd[i-1]))

            vd_i = R_im1_i.T @ (
                vd_im1
                + np.cross(wd_im1, p_im1_i)
                + np.cross(w_im1, np.cross(w_im1, p_im1_i))
            )

            rc_i = self.params.link_com_local[i-1]
            vcd_i = vd_i + np.cross(wd_i, rc_i) + np.cross(w_i, np.cross(w_i, rc_i))

            w.append(w_i); wd.append(wd_i); vd.append(vd_i); vcd.append(vcd_i)

        f_next = np.zeros(3)
        n_next = np.zeros(3)
        tau = np.zeros(6)

        for i in reversed(range(1, 7)):
            m = self.params.link_masses[i-1]
            I = self.params.inertias_local[i-1]
            rc = self.params.link_com_local[i-1]

            F_i = m * vcd[i]
            N_i = I @ wd[i] + np.cross(w[i], I @ w[i])

            if i == 6:
                R_i_ip1 = np.eye(3)
                p_i_ip1 = np.zeros(3)
            else:
                R_i_ip1 = R_list[i]
                p_i_ip1 = p_list[i]

            f_i = F_i + R_i_ip1 @ f_next
            n_i = (N_i
                   + R_i_ip1 @ n_next
                   + np.cross(rc, F_i)
                   + np.cross(p_i_ip1, R_i_ip1 @ f_next))

            tau[i-1] = n_i @ z
            f_next = f_i
            n_next = n_i

        return tau
