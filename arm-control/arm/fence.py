#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""笛卡尔围栏（16.4 第 5 条）。

简化几何来自运动学参数.md §6：每颗电机壳体 / 铝管 / 探针当成圆柱（胶囊），
支架不画。规则：
  * 运动 link 的胶囊（探针除外）离 z=0 ≥ 25 mm；
  * TCP 平时 ≥ 20 mm，标记“下探”的一段允许到 5 mm；
  * 所有中心线端点和 TCP 到 J1 轴的水平距离 ≤ 490 mm（2026-11-09 从 480 放宽）；
  * 底座区（到 J1 轴水平距离 <90 且 z<200）TCP 和手腕不许进；
  * 工作区矩形（可选，参数化）。
底座 #32 固定在地面上，不参加离地 25 mm 和 480 mm 检查。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .kinematics import RobotModel


@dataclass
class Body:
    name: str
    link: str
    p0_local: np.ndarray
    p1_local: np.ndarray
    radius: float
    ground: bool = True      # 参加离地 25 mm
    reach: bool = True       # 参加 480 mm


@dataclass
class FenceResult:
    ok: bool
    reasons: list[str] = field(default_factory=list)
    min_ground_mm: float = 0.0
    tcp_z_mm: float = 0.0
    max_reach_mm: float = 0.0
    tcp: np.ndarray = field(default_factory=lambda: np.zeros(3))
    wrist: np.ndarray = field(default_factory=lambda: np.zeros(3))
    near_limit: list[tuple[str, float]] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "ok": bool(self.ok),
            "reasons": list(self.reasons),
            "min_ground_mm": round(self.min_ground_mm, 2),
            "tcp_z_mm": round(self.tcp_z_mm, 2),
            "max_reach_mm": round(self.max_reach_mm, 2),
            "tcp": [round(float(v), 2) for v in self.tcp],
            "wrist": [round(float(v), 2) for v in self.wrist],
            "near_limit": [{"joint": j, "margin_deg": round(m, 2)}
                           for j, m in self.near_limit],
        }


class Fence:
    def __init__(self, robot: RobotModel,
                 ground_mm: float = 25.0,
                 tcp_mm: float = 20.0,
                 descend_mm: float = 5.0,
                 reach_mm: float = 490.0,
                 base_r_mm: float = 90.0,
                 base_z_mm: float = 200.0,
                 workspace: dict | None = None):
        self.robot = robot
        self.ground_mm = ground_mm
        self.tcp_mm = tcp_mm
        self.descend_mm = descend_mm
        self.reach_mm = reach_mm
        self.base_r_mm = base_r_mm
        self.base_z_mm = base_z_mm
        self.workspace = workspace or {
            "x": [-200.0, 600.0], "y": [-400.0, 400.0], "enabled": False,
        }
        self.bodies = self._build_bodies()

    def _build_bodies(self) -> list[Body]:
        z = self.robot.zero_origins
        # (name, link, p0_B zero pose, p1_B, radius)
        raw = [
            ("J1_PR60", "link1", (0, 0, 84.5), (0, 0, 148.8), 40.0),
            ("J2_PR60", "link2", (0, 80.5, 120.5), (0, 144.8, 120.5), 40.0),
            ("上臂铝管", "link2", (0, 116.5, 187.5), (0, 116.5, 287.5), 17.5),
            ("J3_NE30", "link2", (0, 93.0, 324.0), (0, 159.8, 324.0), 26.0),
            ("前臂铝管", "link3", (0, 37.3, 360.5), (0, 37.3, 460.5), 17.5),
            ("J4_NE30", "link3", (0, -4.0, 497.0), (0, 60.8, 497.0), 26.0),
            ("J5_NE30", "link4", (0, 116.5, 455.7), (0, 116.5, 520.5), 26.0),
            ("J6_NE30", "link5", (0, 75.2, 576.2), (0, 140.0, 576.2), 26.0),
        ]
        out = []
        for name, link, p0, p1, r in raw:
            origin = z[link]
            out.append(Body(name, link,
                            np.asarray(p0, float) - origin,
                            np.asarray(p1, float) - origin, r))
        return out

    # ---------------- 检查 ----------------
    def check(self, q_deg, preset: str = "probe", yaw_deg: float = 0.0,
              descend: bool = False, limits: tuple | None = None,
              relax_below_z: float | None = None) -> FenceResult:
        frames = self.robot.fk_frames(q_deg)
        tcpT = self.robot.tcp(q_deg, preset, yaw_deg)
        tcp = tcpT[:3, 3]
        wrist = frames["link6"][:3, 3]
        reasons: list[str] = []
        min_ground = 1e9
        max_reach = 0.0

        for b in self.bodies:
            Tb = frames[b.link]
            p0 = Tb[:3, :3] @ b.p0_local + Tb[:3, 3]
            p1 = Tb[:3, :3] @ b.p1_local + Tb[:3, 3]
            if b.ground:
                g = min(p0[2], p1[2]) - b.radius
                min_ground = min(min_ground, g)
            if b.reach:
                for p in (p0, p1):
                    max_reach = max(max_reach, math.hypot(p[0], p[1]))

        # 探针是工具，不参加“运动件离地 25 mm”（TCP 另有 20/5 mm 规则）；
        # 探针锥面最低点只作为显示用的参考值，见 probe_min_z()。

        # TCP 离地
        tcp_z = float(tcp[2])
        # 16.4 第 11 条：起点在围栏外时，唯一放行的是“先竖直抬高”那一段：
        # TCP 低于 relax_below_z 时跳过离地检查（抬到 100 以上照常检查）。
        relaxed = relax_below_z is not None and tcp_z < relax_below_z
        tcp_limit = self.descend_mm if descend else self.tcp_mm
        if not relaxed and tcp_z < tcp_limit - 1e-6:
            reasons.append(
                f"TCP 离地 {tcp_z:.1f} mm < {'下探允许' if descend else '平时'} "
                f"{tcp_limit:.0f} mm")
        if not relaxed and min_ground < self.ground_mm - 1e-6:
            reasons.append(f"运动件离地 {min_ground:.1f} mm < {self.ground_mm:.0f} mm")

        # 480 水平距离：按运动链胶囊中心线 + 手腕。探针属于工具：
        # 姿态 B 探针尖到 J1 轴 512.3 mm，若把工具算进来，16.6 要求的姿态 B
        # 会被围栏拒掉；所以 480 只约束臂体，工具由 TCP 规则管。
        max_reach = max(max_reach, math.hypot(float(wrist[0]), float(wrist[1])))
        if max_reach > self.reach_mm + 1e-6:
            reasons.append(f"到 J1 轴水平距离 {max_reach:.1f} mm > {self.reach_mm:.0f} mm")

        # 底座区
        for label, p in (("TCP", tcp), ("手腕", wrist)):
            if math.hypot(float(p[0]), float(p[1])) < self.base_r_mm and p[2] < self.base_z_mm:
                reasons.append(f"{label} 进入底座区（r<{self.base_r_mm:.0f}, z<{self.base_z_mm:.0f}）")

        # 工作区矩形（可选）
        ws = self.workspace or {}
        if ws.get("enabled"):
            xr = ws.get("x", [-1e9, 1e9])
            yr = ws.get("y", [-1e9, 1e9])
            if not (xr[0] <= tcp[0] <= xr[1] and yr[0] <= tcp[1] <= yr[1]):
                reasons.append("TCP 超出工作区矩形")

        # 关节离限位 <10°
        near: list[tuple[str, float]] = []
        if limits is not None:
            lo, hi = np.asarray(limits[0], float), np.asarray(limits[1], float)
            for i, name in enumerate(self.robot.joint_names):
                m = min(float(q_deg[i]) - lo[i], hi[i] - float(q_deg[i]))
                if m < 10.0:
                    near.append((name, m))
        return FenceResult(not reasons, reasons, float(min_ground), tcp_z,
                           float(max_reach), tcp, wrist, near)

    def scene(self, q_deg, preset: str = "probe", yaw_deg: float = 0.0) -> dict:
        """给网页画的简化几何：胶囊 + 底座 + 探针 + TCP/法兰/手腕。"""
        frames = self.robot.fk_frames(q_deg)
        tcpT = self.robot.tcp(q_deg, preset, yaw_deg)
        tcp = tcpT[:3, 3]
        flange = frames["flange_link"][:3, 3]
        axis = tcpT[:3, 2]
        capsules = []
        for b in self.bodies:
            Tb = frames[b.link]
            p0 = Tb[:3, :3] @ b.p0_local + Tb[:3, 3]
            p1 = Tb[:3, :3] @ b.p1_local + Tb[:3, 3]
            capsules.append({"name": b.name, "link": b.link,
                             "p0": p0.round(3).tolist(),
                             "p1": p1.round(3).tolist(),
                             "r": b.radius})
        L = float(self.robot.tcp_presets.get(preset, {}).get("L_mm", 90.0))
        probe = {
            "p0": flange.round(3).tolist(),
            "p1": (flange + axis * L).round(3).tolist(),
            "r0": 20.0 if preset == "probe" else 6.0,
            "r1": 0.75 if preset == "probe" else 2.0,
            "tip": tcp.round(3).tolist(),
        }
        return {
            "capsules": capsules,
            "base": {"center": [0, 0, 41.75], "size": [122.6, 122.6, 83.5]},
            "probe": probe,
            "tcp": tcp.round(3).tolist(),
            "flange": flange.round(3).tolist(),
            "wrist": frames["link6"][:3, 3].round(3).tolist(),
            "tool_axis": axis.round(5).tolist(),
        }

    def probe_min_z(self, tcpT: np.ndarray, preset: str) -> float:
        """探针锥面最低点（近似，仅用于显示，不参与 25 mm 判定）。"""
        if preset != "probe":
            return float(tcpT[:3, 3][2])
        axis = tcpT[:3, 2]
        flange = tcpT[:3, 3] - axis * 90.0
        mn = 1e9
        for t in (0.0, 4.0, 9.5, 20.0, 40.0, 60.0, 80.0, 89.0, 90.0):
            r = 20.0 if t <= 9.5 else 12.0 + (0.75 - 12.0) * (t - 9.5) / (90.0 - 9.5)
            p = flange + axis * t
            mn = min(mn, float(p[2]) - r)
        return mn
