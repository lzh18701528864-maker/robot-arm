#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16.1 运动学自检：A/B/C/D + gripper_center。

用法：
    cd arm-control && python3 tools/kinematics_selfcheck.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from arm import config  # noqa: E402
from arm.kinematics import RobotModel  # noqa: E402

# 期望值（mm）：运动学参数.md §5 / Mac_agent待办 §16.1
CASES = [
    ("A", [0, 0, 0, 0, 0, 0],
     (0, 116.5, 576.2), (0, 144.0, 576.2), (0, 234.0, 576.2), (0, 1, 0)),
    ("B", [0, 90, 0, 0, 0, 0],
     (455.7, 116.5, 120.5), (455.7, 144.0, 120.5), (455.7, 234.0, 120.5), (0, 1, 0)),
    ("B′", [0, 85, 0, 0, 0, 0],
     (453.97, 116.5, 160.22), (453.97, 144.0, 160.22), (453.97, 234.0, 160.22), (0, 1, 0)),
    ("C", [0, 60, 30, 0, -90, 0],
     (428.44, 116.5, 222.25), (428.44, 116.5, 194.75), (428.44, 116.5, 104.75), (0, 0, -1)),
    ("D", [90, 0, 0, 0, 0, 0],
     (-116.5, 0, 576.2), (-144.0, 0, 576.2), (-234.0, 0, 576.2), (-1, 0, 0)),
]


def main() -> int:
    robot = RobotModel(config.URDF, config.tcp_presets())
    worst = 0.0
    all_ok = True
    print(f"URDF: {robot.path}")
    print(f"{'姿态':<4}{'点':<14}{'期望':<24}{'计算':<24}{'差(mm)':>8}")
    for name, q, wrist_e, flange_e, tcp_e, axis_e in CASES:
        f = robot.fk_frames(q)
        wrist = f["link6"][:3, 3]
        flange = robot.flange(q)[:3, 3]
        tcpT = robot.tcp(q, "probe")
        tcp = tcpT[:3, 3]
        axis = robot.flange(q)[:3, 2]
        for label, got, exp in (("J5∩J6", wrist, wrist_e),
                                ("法兰中心", flange, flange_e),
                                ("探针尖", tcp, tcp_e)):
            d = float(np.linalg.norm(got - np.asarray(exp, float)))
            worst = max(worst, d)
            all_ok &= d <= 0.05
            print(f"{name:<4}{label:<14}{fmt(exp):<24}{fmt(got):<24}{d:8.4f}")
        da = float(np.degrees(np.arccos(max(-1.0, min(1.0,
            float(np.dot(axis, np.asarray(axis_e, float))))))))
        all_ok &= da <= 0.05
        print(f"{name:<4}{'工具轴':<14}{str(axis_e):<24}{fmt(axis):<24}{da:8.4f}°")

    # gripper_center @ C
    q = [0, 60, 30, 0, -90, 0]
    gc = robot.tcp(q, "gripper_center")[:3, 3]
    exp = np.array([428.44, 116.5, 55.60])
    d = float(np.linalg.norm(gc - exp))
    worst = max(worst, d)
    all_ok &= d <= 0.05
    print(f"{'C':<4}{'gripper_center':<14}{fmt(exp):<24}{fmt(gc):<24}{d:8.4f}")
    gt = robot.tcp(q, "gripper_tip")[:3, 3]
    exp_t = np.array([428.44, 116.5, 33.10])
    dt = float(np.linalg.norm(gt - exp_t))
    print(f"{'C':<4}{'gripper_tip':<14}{fmt(exp_t):<24}{fmt(gt):<24}{dt:8.4f}")
    all_ok &= dt <= 0.05
    print()
    print(f"最大偏差 {worst:.6f} mm；判定 {'通过' if all_ok else '不通过'}")
    print(f"URDF自检 A/B/C/D/B′/gripper_center/gripper_tip = {'通过' if all_ok else '不通过'}"
          f"（最大 {worst:.4f} mm）")
    return 0 if all_ok else 1


def fmt(v) -> str:
    return "(" + ", ".join(f"{float(x):.3f}" for x in v) + ")"


if __name__ == "__main__":
    sys.exit(main())
