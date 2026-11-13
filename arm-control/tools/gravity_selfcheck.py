#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""18.2 自检：核对值表 8 行 ±0.05 N·m；预检拒绝/通过用例；路径预检。

跑法：cd arm-control && python3 tools/gravity_selfcheck.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from arm import gravity                                     # noqa: E402
from arm.kinematics import RobotModel                       # noqa: E402

URDF = ROOT / "urdf" / "gluon_2l6_4l3.urdf"

# Mac_agent待办.md §18.2 核对值表（N·m）
TABLE = [
    ((0, 0, 0, 0, 0, 0), "flange", [0.0, 0.0, 0.0, 0.0]),
    ((0, 30, 30, 0, 0, 0), "flange", [2.87, 1.48, 0.19, 0.0]),
    ((0, 30, 30, 0, 0, 0), "gripper", [4.29, 2.44, 0.49, 0.37]),
    ((0, 60, 30, 0, -90, 0), "flange", [4.12, 1.71, 0.22, 0.0]),
    ((0, 60, 30, 0, -90, 0), "gripper", [6.01, 2.82, 0.57, 0.00]),
    ((0, 85, 0, 0, 0, 0), "flange", [4.48, 1.70, 0.22, 0.0]),
    ((0, 85, 0, 0, 0, 0), "gripper", [6.48, 2.81, 0.57, 0.42]),
    ((-90, 85, 5, 90, 0, 0), "flange", [4.27, 1.49, 0.00, 0.0]),
]

LIMIT_A = {"J2": 4.0, "J3": 8.0}
TOL = 0.05


def main() -> int:
    robot = RobotModel(URDF)
    ok = True
    print("18.2 核对值（τ2 τ3 τ4 τ5，单位 N·m；允差 ±0.05）")
    worst = 0.0
    for q, ee, want in TABLE:
        got = gravity.torques(robot, q, ee)[1:5].copy()
        # 18.2：J5 只比绝对值（J5 轴随腕倾斜，符号约定不作要求）
        got[3] = abs(got[3])
        diffs = [abs(float(a) - b) for a, b in zip(got, want)]
        worst = max(worst, max(diffs))
        good = max(diffs) <= TOL
        ok = ok and good
        print(f"  {'OK ' if good else 'BAD'} {ee:8s} {str(tuple(q)):26s} "
              f"{' '.join(f'{v:6.3f}' for v in got)}   "
              f"(期望 {' '.join(f'{v:5.2f}' for v in want)}，最大差 {max(diffs):.3f})")
    print(f"  最大偏差 {worst:.3f} N·m → {'通过' if worst <= TOL else '不通过'}\n")

    # 容量
    cap = gravity.capacities(LIMIT_A)
    print("容量（J2 回读 4 A）："
          + "  ".join(f"{k} {v:.2f}" for k, v in cap.items() if math.isfinite(v)))

    # 预检用例：gripper + 0.5 kg 到 B′ 必须拒绝并指出 J2；flange 同目标通过
    q = [0.0, 85.0, 0.0, 0.0, 0.0, 0.0]
    bad_g = gravity.check_q(robot, q, "gripper", 0.5, LIMIT_A)
    bad_f = gravity.check_q(robot, q, "flange", 0.0, LIMIT_A)
    g_ok = any(b["joint"] == "J2" for b in bad_g)
    f_ok = not bad_f
    ok = ok and g_ok and f_ok
    print(f"\n预检 gripper+0.5kg→B′: {'拒绝' if bad_g else '通过'} "
          f"{[(b['joint'], b['tau'], b['ratio']) for b in bad_g]}")
    print(f"预检 flange→B′:       {'拒绝' if bad_f else '通过'} {bad_f}")

    # 路径预检：停机路径最后一段（预停机点 → P）
    pts = [[-90.0, 76.5 + (85.0 - 76.5) * t / 10, 13.5 + (5.0 - 13.5) * t / 10,
            90.0, 0.0, 0.0] for t in range(11)]
    bad_p = gravity.check_path(robot, pts, "flange", 0.0, LIMIT_A)
    ok = ok and not bad_p
    print(f"预检 18.3 停机末段: {'通过' if not bad_p else bad_p}")

    # 保持看护比例
    hr = gravity.hold_ratios(robot, [0, 85, 0, 0, 0, 0], "flange", 0.0, LIMIT_A)
    print(f"保持比例 B′ (flange): {hr}")

    print("\n结论：" + ("18.2 模型与预检通过" if ok else "有项目不通过"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
