#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""18.2 静态重力力矩模型 + 路径预检。

数据源：Mac_agent待办.md §18.2（v8，2026-11-09）的质点表和核对值表。

约定：
  * 角度 deg，长度 mm；力矩 N·m，符号和 16.1 关节表正方向同向为正。
  * τ_i = -g · Σ m · ( (a_i × (p - o_i)) · ẑ )，a_i/o_i 是第 i 轴在 B 系的
    方向和轴上一点（由 FK 给出），p 是各质点在 B 系的位置。
    这是重力做的功对关节角的偏导，等价于 roboticstoolbox 的 gravload(q)。
  * 只有 J2…J5 会被检查；J1 轴竖直、夹爪质心在 J6 轴上，不产生重力力矩。

容量（N·m；取电机额定、支架、限流×Kt 三者最小）：
  J2 = min(12, 回读限流 × 1.91)；J3 = 6.5；J4/J5 = 2.3，
  18.4 标定以后 J4/J5 改成 min(2.3, Kt_eff × 回读限流)。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

G = 9.81

# link 系里的质点 (x, y, z, m)；零位时各 link 系和 B 系平行（18.2 质点表）
MASSES: dict[str, list[tuple[float, float, float, float]]] = {
    "link2": [(0.0, 0.0, 50.0, 0.12), (0.0, 0.0, 117.0, 0.017),
              (0.0, 0.0, 183.5, 0.10), (0.0, 0.0, 203.5, 0.30)],
    "link3": [(0.0, -79.2, 20.0, 0.08), (0.0, -79.2, 86.5, 0.017),
              (0.0, -79.2, 153.0, 0.08), (0.0, -79.2, 173.0, 0.23)],
    "link4": [(0.0, 79.2, 0.0, 0.23), (0.0, 79.2, 40.0, 0.10)],
    "link5": [(0.0, 0.0, 79.2, 0.23)],
}

# 末端配置（法兰系：z_f = 工具轴，离法兰面 mm）
END_EFFECTORS: dict[str, list[tuple[float, float, float, float]]] = {
    "flange": [],
    "probe": [],                       # 临时针，质量忽略（18.2 只给了 flange / gripper）
    "gripper": [(0.0, 0.0, 68.0, 0.45)],
}
PAYLOAD_CENTER_MM = 139.15             # 夹持中心（法兰系）

CAPACITY_J2_PLA = 8.0                  # #0/#16 支架 PLA 按 8 N·m（P08-9）
CAPACITY_J2_PETG = 12.0                # 换 PETG 100% 后按 12
CAPACITY_J2_KT = 1.91                  # PR60 标准 Kt
CAPACITY_J3 = 6.5                      # NE30 额定
CAPACITY_LITE = 2.3                    # Lite NE30 额定
CHECK_MARGIN = 0.8                     # |τ| > 0.8 × 容量 → 拒绝
HOLD_WARN_RATIO = 0.7                  # 保持看护：>70%
HOLD_WARN_S = 30.0

CHECKED_JOINTS = ("J2", "J3", "J4", "J5")


def _checked_indices() -> list[int]:
    return [1, 2, 3, 4]


def torques(robot, q_deg, end_effector: str = "flange",
            payload_kg: float = 0.0) -> np.ndarray:
    """返回 6 个关节的重力力矩（N·m），按 J1…J6 顺序。"""
    if end_effector not in END_EFFECTORS:
        raise KeyError(f"未知末端配置 {end_effector}")
    f = robot.fk_frames(q_deg)
    # 每个质点归属到 link；只有关节 i 的下游（link i+1 … flange）会动
    nodes: list[tuple[str, np.ndarray, float]] = []
    for link, ms in MASSES.items():
        T = f[link]
        for x, y, z, m in ms:
            nodes.append((link, T @ np.array([x, y, z, 1.0]), m))
    Tf = f["flange_link"]
    for x, y, z, m in END_EFFECTORS[end_effector]:
        nodes.append(("flange_link", Tf @ np.array([x, y, z, 1.0]), m))
    if payload_kg:
        nodes.append(("flange_link",
                      Tf @ np.array([0.0, 0.0, PAYLOAD_CENTER_MM, 1.0]),
                      float(payload_kg)))

    distal = ["link%d" % (i + 1) for i in range(6)] + ["flange_link"]
    out = np.zeros(6)
    for i in range(6):
        T = f[distal[i]]
        o = T[:3, 3]
        # 轴取自 URDF（J1/J5 = z，其余 = y），不写死。
        a = T[:3, :3] @ robot.joints[f"J{i + 1}"].axis
        a = a / max(float(np.linalg.norm(a)), 1e-12)
        s = 0.0
        for link, p, m in nodes:
            if link not in distal[i:]:
                continue
            r = p[:3] - o
            s += m * float(np.cross(a, r)[2])
        out[i] = -G * s / 1000.0  # mm → m
    return out


def capacities(limit_a: dict | None = None, kt: dict | None = None,
               j2_bracket: str = "PLA") -> dict:
    """各关节容量（N·m）。

    limit_a：回读限流（A）；kt：18.4 标定结果；j2_bracket：`PLA`（8 N·m）
    或 `PETG100`（12 N·m），见 18.2 容量表 / P08-9。
    """
    limit_a = limit_a or {}
    kt = kt or {}
    j2_a = float(limit_a.get("J2") or 4.0)
    bracket = (CAPACITY_J2_PETG if str(j2_bracket).upper() == "PETG100"
               else CAPACITY_J2_PLA)
    cap = {
        "J1": math.inf,
        "J2": min(bracket, j2_a * CAPACITY_J2_KT),
        "J3": CAPACITY_J3,
        "J4": CAPACITY_LITE,
        "J5": CAPACITY_LITE,
        "J6": math.inf,
    }
    for name in ("J4", "J5"):
        k = kt.get(name) or {}
        eff = k.get("kt_eff")
        ia = limit_a.get(name)
        if eff is not None and ia:
            cap[name] = min(CAPACITY_LITE, float(eff) * float(ia))
    return cap


def check_q(robot, q_deg, end_effector: str = "flange", payload_kg: float = 0.0,
            limit_a: dict | None = None, kt: dict | None = None,
            margin: float = CHECK_MARGIN, j2_bracket: str = "PLA") -> list[dict]:
    """单个姿态的预检；返回超限项（空 = 通过）。"""
    tau = torques(robot, q_deg, end_effector, payload_kg)
    cap = capacities(limit_a, kt, j2_bracket)
    bad = []
    for i in _checked_indices():
        name = f"J{i + 1}"
        c = cap[name]
        if not math.isfinite(c) or c <= 0:
            continue
        ratio = abs(float(tau[i])) / c
        if ratio > margin:
            bad.append({
                "joint": name,
                "tau": round(float(tau[i]), 3),
                "capacity": round(float(c), 3),
                "ratio": round(ratio, 3),
                "q": [round(float(v), 3) for v in q_deg],
            })
    return bad


def check_path(robot, points, end_effector: str = "flange", payload_kg: float = 0.0,
               limit_a: dict | None = None, kt: dict | None = None,
               margin: float = CHECK_MARGIN, j2_bracket: str = "PLA") -> list[dict]:
    """整条路径预检；points 是插补点列表（每个 6 个关节角）。"""
    bad = []
    for idx, q in enumerate(points):
        for item in check_q(robot, q, end_effector, payload_kg, limit_a, kt,
                            margin, j2_bracket):
            item["index"] = idx
            bad.append(item)
    return bad


def hold_ratios(robot, q_deg, end_effector: str = "flange", payload_kg: float = 0.0,
                limit_a: dict | None = None, kt: dict | None = None,
                j2_bracket: str = "PLA") -> dict:
    """保持看护：各关节 |τ| / 容量。"""
    tau = torques(robot, q_deg, end_effector, payload_kg)
    cap = capacities(limit_a, kt, j2_bracket)
    out = {}
    for i in _checked_indices():
        name = f"J{i + 1}"
        c = cap[name]
        out[name] = (round(abs(float(tau[i])) / c, 3)
                     if math.isfinite(c) and c > 0 else None)
    return out


# --------------------------------------------------------------------------- #
# Kt 标定结果（18.4）：config/kt.json
# --------------------------------------------------------------------------- #
def kt_path(root: Path | None = None) -> Path:
    base = Path(root) if root else Path(__file__).resolve().parent.parent
    return base / "config" / "kt.json"


def load_kt(root: Path | None = None) -> dict:
    p = kt_path(root)
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return {}


def save_kt(data: dict, root: Path | None = None) -> Path:
    p = kt_path(root)
    p.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                 encoding="utf-8")
    return p


# --------------------------------------------------------------------------- #
# 18.4 标定点
# --------------------------------------------------------------------------- #
# 每个点：name、q（deg）、挂重（kg，法兰上两颗 M3 螺钉挂）、要拟合的关节
KT_POINTS = [
    {"group": "J2", "label": "q2=0", "q": [0, 0, 0, 0, 0, 0], "load_kg": 0.0,
     "joints": ["J2"]},
    {"group": "J2", "label": "q2=30", "q": [0, 30, 0, 0, 0, 0], "load_kg": 0.0,
     "joints": ["J2"]},
    {"group": "J2", "label": "q2=60", "q": [0, 60, 0, 0, 0, 0], "load_kg": 0.0,
     "joints": ["J2"]},
    {"group": "J2", "label": "q2=85", "q": [0, 85, 0, 0, 0, 0], "load_kg": 0.0,
     "joints": ["J2"]},
    {"group": "J3J4", "label": "挂 0 g", "q": [0, 0, 90, 0, 0, 0], "load_kg": 0.0,
     "joints": ["J2", "J3", "J4"]},
    {"group": "J3J4", "label": "挂 250 g", "q": [0, 0, 90, 0, 0, 0],
     "load_kg": 0.25, "joints": ["J2", "J3", "J4"]},
    {"group": "J3J4", "label": "挂 500 g", "q": [0, 0, 90, 0, 0, 0],
     "load_kg": 0.50, "joints": ["J2", "J3", "J4"]},
    {"group": "J3J4", "label": "挂 1000 g", "q": [0, 0, 90, 0, 0, 0],
     "load_kg": 1.00, "joints": ["J2", "J3", "J4"]},
]

KT_J5_POINTS = [
    {"group": "J5", "label": f"q5={q}", "q": [0, 0, 90, 0, q, 0], "load_kg": 0.0,
     "joints": ["J5"]}
    for q in (0, 45, -45, 90, -90)
]


def fit_kt(points: list[dict]) -> dict:
    """points: [{joint, tau, current, full_scale_a, load_kg, label}] → 每关节拟合。

    τ = Kt_eff · I + b。R² 用 1 - SS_res/SS_tot；点数 <2 或 I 全相同则退化。
    """
    groups: dict[str, list[dict]] = {}
    for p in points:
        groups.setdefault(p["joint"], []).append(p)
    out = {}
    for joint, rows in groups.items():
        xs = np.array([float(r["current"]) for r in rows], float)
        ys = np.array([float(r["tau"]) for r in rows], float)
        rec = {"n": len(rows), "labels": [r.get("label", "") for r in rows],
               "current": [round(float(v), 4) for v in xs],
               "tau": [round(float(v), 4) for v in ys],
               "kt_date": None, "kt_eff": None, "offset": None, "r2": None,
               "kt_sign": None, "full_scale_a": None}
        if len(rows) >= 2 and float(np.ptp(xs)) > 1e-9:
            A = np.column_stack([xs, np.ones_like(xs)])
            (k, b), *_ = np.linalg.lstsq(A, ys, rcond=None)
            pred = k * xs + b
            ss_res = float(np.sum((ys - pred) ** 2))
            ss_tot = float(np.sum((ys - float(np.mean(ys))) ** 2))
            rec.update({"kt_eff": round(float(k), 6),
                        "offset": round(float(b), 6),
                        "r2": round(1.0 - ss_res / ss_tot, 4) if ss_tot > 1e-12 else 1.0,
                        # P08-17：拟出负值说明电流符号和 16.1 正方向相反，
                        # 记符号、不取绝对值
                        "kt_sign": -1 if k < 0 else 1})
        fs = [r.get("full_scale_a") for r in rows if r.get("full_scale_a")]
        if fs:
            rec["full_scale_a"] = float(fs[0])
        out[joint] = rec
    return out
