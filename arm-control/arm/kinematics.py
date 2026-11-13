#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDF 解析、正运动学、数值 IK。

单位约定（本工程内部）：
    * 长度 mm（URDF 里是 m，读入时 ×1000）
    * 角度 deg（对外）；内部计算用 rad
    * 姿态用 4×4 齐次矩阵，B 系为世界系

数据源：urdf/gluon_2l6_4l3.urdf（= 运动学参数.md §3 / Mac_agent待办 §16.1）。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

import numpy as np

MM = 1000.0
DEG = 180.0 / math.pi


# --------------------------------------------------------------------------- #
# 基础旋转
# --------------------------------------------------------------------------- #
def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    """URDF 固定轴 rpy：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def axis_rot(axis: np.ndarray, ang: float) -> np.ndarray:
    """Rodrigues：绕单位向量转 ang 的旋转矩阵。"""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    x, y, z = axis / n
    c, s = math.cos(ang), math.sin(ang)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ], dtype=float)


def T(R: np.ndarray | None = None, p: Iterable[float] | None = None) -> np.ndarray:
    out = np.eye(4)
    if R is not None:
        out[:3, :3] = R
    if p is not None:
        out[:3, 3] = np.asarray(p, dtype=float)
    return out


def rotvec(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 旋转向量（罗德里格斯，数值稳定版）。"""
    tr = float(np.trace(R))
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]], dtype=float) * 0.5
    if abs(math.pi - ang) < 1e-6:
        # 接近 180°：从对称部分取轴
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(axis))
        if axis[k] > 1e-9:
            axis = A[:, k] / axis[k]
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        return axis * ang
    return (ang / (2.0 * math.sin(ang))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=float)


def orient_from_axis(tool_axis_b, yaw_deg: float | None = None) -> np.ndarray:
    """由工具轴方向（法兰 z_f）造一个完整姿态；yaw 绕工具轴。

    yaw 的定义：先取参考 x0 = normalize(up × z)，y0 = z × x0；再绕 z 转 yaw。
    yaw=None 时只用到 z 列（IK 里只约束轴方向）。
    """
    z = np.asarray(tool_axis_b, dtype=float)
    z = z / max(np.linalg.norm(z), 1e-12)
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z, up))) > 0.999:
        up = np.array([1.0, 0.0, 0.0])
    x0 = np.cross(up, z)
    x0 = x0 / max(np.linalg.norm(x0), 1e-12)
    y0 = np.cross(z, x0)
    psi = 0.0 if yaw_deg is None else math.radians(yaw_deg)
    x = math.cos(psi) * x0 + math.sin(psi) * y0
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


# --------------------------------------------------------------------------- #
# URDF
# --------------------------------------------------------------------------- #
@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float = 0.0   # rad
    upper: float = 0.0   # rad
    velocity: float = 0.0


class RobotModel:
    """从 URDF 读一条串联链：base → J1..J6 → flange → tcp_*。"""

    def __init__(self, urdf_path: str | Path,
                 tcp_presets: dict | None = None):
        self.path = Path(urdf_path)
        self.tree = ET.parse(self.path)
        self.root = self.tree.getroot()
        self.joints: dict[str, Joint] = {}
        self.child_map: dict[str, str] = {}       # child link -> joint name
        self._parse()
        self.joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        self.chain = self._trace("base", "flange_link")
        self.tcp_presets = tcp_presets or {
            "probe": {"L_mm": 90.0, "yaw_deg": 0.0, "measured": False},
            "gripper_tip": {"L_mm": 161.65, "yaw_deg": -19.2, "measured": False},
            "gripper_center": {"L_mm": 139.15, "yaw_deg": -19.2, "measured": False},
            "flange": {"L_mm": 0.0, "yaw_deg": 0.0, "measured": True},
        }
        # 零位时各 link 原点（B 系，mm）：用于把简化几何转成 link 局部坐标
        f = self.fk_frames([0.0] * 6)
        self.zero_origins = {k: v[:3, 3].copy() for k, v in f.items()}

    # ---------------- 解析 ----------------
    def _parse(self) -> None:
        for el in self.root.findall("joint"):
            origin = np.eye(4)
            o = el.find("origin")
            if o is not None:
                xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
                rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
                origin = T(rpy_to_R(*rpy), np.array(xyz) * MM)
            ax = el.find("axis")
            axis = np.array([float(v) for v in (ax.get("xyz") if ax is not None else "0 0 1").split()])
            lim = el.find("limit")
            lower = float(lim.get("lower", "0")) if lim is not None else 0.0
            upper = float(lim.get("upper", "0")) if lim is not None else 0.0
            vel = float(lim.get("velocity", "0")) if lim is not None else 0.0
            j = Joint(el.get("name"), el.get("type"),
                      el.find("parent").get("link"),
                      el.find("child").get("link"),
                      origin, axis, lower, upper, vel)
            self.joints[j.name] = j
            self.child_map[j.child] = j.name

    def _trace(self, root_link: str, end_link: str) -> list[Joint]:
        out: list[Joint] = []
        link = end_link
        while link != root_link:
            jn = self.child_map.get(link)
            if jn is None:
                raise ValueError(f"URDF 链断开：{link}")
            j = self.joints[jn]
            out.append(j)
            link = j.parent
        out.reverse()
        return out

    # ---------------- FK ----------------
    def fk_frames(self, q_deg: Iterable[float]) -> dict[str, np.ndarray]:
        """返回 {link: T_B_link}，包含 flange_link / tcp_*。"""
        qs = list(q_deg)
        if len(qs) != 6:
            raise ValueError("需要 6 个关节角")
        frames: dict[str, np.ndarray] = {"base": np.eye(4)}
        Tcur = np.eye(4)
        qi = 0
        for j in self.chain:
            Tcur = Tcur @ j.origin
            if j.type in ("revolute", "continuous"):
                Tcur = Tcur @ T(axis_rot(j.axis, math.radians(qs[qi])))
                qi += 1
            frames[j.child] = Tcur.copy()
        for name in ("tcp_probe", "tcp_gripper_tip", "tcp_gripper_center"):
            jn = name
            if jn in self.joints:
                j = self.joints[jn]
                frames[name] = frames[j.parent] @ j.origin
        return frames

    def flange(self, q_deg) -> np.ndarray:
        return self.fk_frames(q_deg)["flange_link"]

    def tcp(self, q_deg, preset: str = "probe",
            yaw_deg: float | None = None) -> np.ndarray:
        f = self.flange(q_deg)
        cfg = self.tcp_presets.get(preset)
        if cfg is None:
            raise KeyError(f"未知 TCP 预设 {preset}")
        L = float(cfg["L_mm"])
        if yaw_deg is None:
            yaw_deg = cfg.get("yaw_deg")
        Tt = f.copy()
        if yaw_deg is not None:
            Tt[:3, :3] = f[:3, :3] @ rot_z(math.radians(yaw_deg))
        Tt[:3, 3] = f[:3, 3] + f[:3, :3] @ np.array([0.0, 0.0, L])
        return Tt

    # ---------------- IK ----------------
    def ik(self, target_mm: Iterable[float], tool_axis_b=(0.0, 0.0, -1.0),
           yaw_deg: float | None = 0.0, seed_deg: Iterable[float] | None = None,
           preset: str = "probe", limits: tuple[np.ndarray, np.ndarray] | None = None,
           max_iters: int = 120, structured_seeds: bool = True,
           restarts: int = 2, time_budget_s: float | None = None) -> "IKResult":
        """数值 IK：目标 = TCP 位置 + 工具轴方向 + 可选偏航。

        约束 6 维（位置 3 + 姿态 3）；yaw_deg=None 时只约束工具轴方向，
        绕工具轴的偏航自由（此时姿态误差只剩 2 维）。
        残差：位置 mm，姿态角 deg。
        """
        target = np.asarray(target_mm, dtype=float)
        axis = np.asarray(tool_axis_b, dtype=float)
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        seed = np.asarray(seed_deg if seed_deg is not None else [0.0] * 6, dtype=float)
        lo, hi = self._limit_arrays(limits)
        R_target = orient_from_axis(axis, yaw_deg) if yaw_deg is not None else None

        def axis_rotvec(a, b):
            """从 a 转到 b 的旋转向量（带符号；反平行时不能当成 0）。"""
            a = a / max(np.linalg.norm(a), 1e-12)
            b = b / max(np.linalg.norm(b), 1e-12)
            cross = np.cross(a, b)
            sin = float(np.linalg.norm(cross))
            cos = float(np.clip(np.dot(a, b), -1.0, 1.0))
            ang = math.atan2(sin, cos)
            if sin > 1e-9:
                return cross / sin * ang
            if cos > 0:
                return np.zeros(3)
            # 反平行：任取一个垂直轴，转 180°
            tmp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            perp = np.cross(a, tmp)
            perp = perp / max(np.linalg.norm(perp), 1e-12)
            return perp * math.pi

        def err(q: np.ndarray) -> np.ndarray:
            Tc = self.tcp(q, preset, yaw_deg)
            e_pos = target - Tc[:3, 3]
            if R_target is not None:
                e_rot = rotvec(Tc[:3, :3].T @ R_target)
            else:
                e_rot = axis_rotvec(Tc[:3, :3][:, 2], axis)
            return np.concatenate([e_pos, e_rot * 50.0])  # 50 mm/rad 权重

        def err_raw(q: np.ndarray) -> tuple[float, float]:
            Tc = self.tcp(q, preset, yaw_deg)
            pe = float(np.linalg.norm(target - Tc[:3, 3]))
            if R_target is not None:
                re = math.degrees(float(np.linalg.norm(rotvec(Tc[:3, :3].T @ R_target))))
            else:
                a = Tc[:3, :3][:, 2]
                a = a / max(np.linalg.norm(a), 1e-12)
                cos = float(np.clip(np.dot(a, axis), -1.0, 1.0))
                re = math.degrees(math.acos(cos))
            return pe, re

        seeds: list[np.ndarray] = [np.clip(seed, lo, hi)]
        if structured_seeds:
            seeds += self._structured_seeds(seed, target, axis, lo, hi)

        best: IKResult | None = None
        t_start = time.monotonic()
        for si, s in enumerate(seeds):
            if time_budget_s is not None and time.monotonic() - t_start > time_budget_s:
                break
            q, pe, re = self._dls(s, err, err_raw, lo, hi, max_iters)
            # 评分：位置误差为主，姿态误差折算成 mm，另外偏好 q3≥0、靠近种子
            score = pe + re * 0.5
            score += 0.01 * float(np.linalg.norm(q - seed))
            if q[2] < 0:
                score += 0.5 + 0.02 * abs(q[2])
            ok = pe <= 0.2 and re <= 0.5
            cand = IKResult(q.copy(), pe, re, score, s.copy(), ok)
            if best is None or (ok and not best.ok) or \
                    (ok == best.ok and score < best.score):
                best = cand
            # 已经很好就停：位置 <0.05 mm、轴 <0.1°，且至少看过 3 个种子
            if best.ok and best.pos_err_mm <= 0.05 and best.axis_err_deg <= 0.1 \
                    and si >= 2:
                break
        if best is None:
            raise RuntimeError("IK 没有产生结果")
        return best

    def _limit_arrays(self, limits):
        if limits is None:
            lo = np.array([-135.0, -90.0, -135.0, -135.0, -135.0, -165.0])
            hi = np.array([135.0, 90.0, 135.0, 135.0, 135.0, 165.0])
            return lo, hi
        lo = np.asarray(limits[0], dtype=float)
        hi = np.asarray(limits[1], dtype=float)
        return lo, hi

    def _structured_seeds(self, seed, target, axis, lo, hi) -> list[np.ndarray]:
        """给几个粗糙种子：按目标方位选 q1，扫 q2/q3，q5 取使工具轴朝下的值。"""
        out: list[np.ndarray] = []
        phi = math.degrees(math.atan2(target[1], target[0]))
        q1s = [seed[0], phi, phi + 180.0]
        q5 = -90.0 if axis[2] < -0.5 else 0.0
        for q1 in q1s:
            for q2 in (-40.0, -10.0, 20.0, 50.0, 80.0):
                for q3 in (-20.0, 10.0, 40.0, 70.0):
                    q = np.array([q1, q2, q3, 0.0, q5, seed[5]], dtype=float)
                    out.append(np.clip(q, lo, hi))
        # 去重
        uniq: list[np.ndarray] = []
        for q in out:
            if not any(np.allclose(q, u, atol=1e-6) for u in uniq):
                uniq.append(q)
        return uniq

    def _dls(self, q0, err, err_raw, lo, hi, max_iters):
        q = np.clip(np.asarray(q0, dtype=float).copy(), lo, hi)
        lam = 2.0
        best_q = q.copy()
        best_norm = float(np.linalg.norm(err(q)))
        for it in range(max_iters):
            e = err(q)
            n = float(np.linalg.norm(e))
            if n < best_norm:
                best_norm, best_q = n, q.copy()
            pe, re = err_raw(q)
            if pe <= 0.005 and re <= 0.01:
                return q, pe, re
            J = np.zeros((6, 6))
            h = 1e-4  # deg
            for k in range(6):
                qp = q.copy()
                qp[k] = min(hi[k], q[k] + h)
                if qp[k] == q[k]:
                    qp[k] = max(lo[k], q[k] - h)
                J[:, k] = (err(qp) - e) / (qp[k] - q[k])
            A = J @ J.T + (lam ** 2) * np.eye(6)
            try:
                dq = -J.T @ np.linalg.solve(A, e)
            except np.linalg.LinAlgError:
                break
            # 限步长
            md = float(np.max(np.abs(dq)))
            if md > 8.0:
                dq *= 8.0 / md
            accepted = False
            alpha = 1.0
            for _ in range(6):
                qn = np.clip(q + alpha * dq, lo, hi)
                if float(np.linalg.norm(err(qn))) < n:
                    q = qn
                    accepted = True
                    break
                alpha *= 0.5
            if not accepted:
                lam = min(lam * 3.0, 500.0)
            else:
                lam = max(lam * 0.8, 0.5)
        pe, re = err_raw(best_q)
        return best_q, pe, re


@dataclass
class IKResult:
    q_deg: np.ndarray
    pos_err_mm: float
    axis_err_deg: float
    score: float = 0.0
    seed: np.ndarray = field(default_factory=lambda: np.zeros(6))
    ok: bool = False

    def as_dict(self) -> dict:
        return {
            "q_deg": [round(float(v), 4) for v in self.q_deg],
            "pos_err_mm": round(self.pos_err_mm, 4),
            "axis_err_deg": round(self.axis_err_deg, 4),
            "ok": bool(self.ok),
        }
