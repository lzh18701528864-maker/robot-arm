#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16.2 临时探针（probe_L90）建模 + 自检。

坐标（探针自己的坐标系，和第十五节夹爪一致）：
    探针轴 = +Y，J6 法兰面 y=0，+Y 指向远离法兰；
    角度从 +X 量向 +Z（atan2(z,x)）。

运行（需要 OCP）：
    PYTHONPATH=../innfos-backup/_tools/pylibs python3 tools/probe_model.py

输出：
    models/probe_L90.step
    models/probe_L90.stl
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "models"

from OCP.BRepAdaptor import BRepAdaptor_Curve  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse  # noqa: E402
from OCP.BRepBndLib import BRepBndLib  # noqa: E402
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeFace,  # noqa: E402
                                BRepBuilderAPI_MakePolygon,
                                BRepBuilderAPI_Transform)
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet  # noqa: E402
from OCP.BRepMesh import BRepMesh_IncrementalMesh  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.BRepPrimAPI import (BRepPrimAPI_MakeBox,  # noqa: E402
                             BRepPrimAPI_MakeCone,
                             BRepPrimAPI_MakeCylinder,
                             BRepPrimAPI_MakePrism)
from OCP.Bnd import Bnd_Box  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain  # noqa: E402
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer  # noqa: E402
from OCP.StlAPI import StlAPI_Writer  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec  # noqa: E402

Y_AXIS = gp_Dir(0, 1, 0)


def cyl(d_mm: float, y0: float, y1: float):
    ax = gp_Ax2(gp_Pnt(0, y0, 0), Y_AXIS)
    return BRepPrimAPI_MakeCylinder(ax, d_mm / 2.0, y1 - y0).Shape()


def fuse(a, b):
    op = BRepAlgoAPI_Fuse(a, b)
    op.SetFuzzyValue(1e-6)
    op.Build()
    if not op.IsDone():
        raise RuntimeError("Fuse 失败")
    return op.Shape()


def cut(a, b):
    op = BRepAlgoAPI_Cut(a, b)
    op.SetFuzzyValue(1e-6)
    op.Build()
    if not op.IsDone():
        raise RuntimeError("Cut 失败")
    return op.Shape()


def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, b, False, False)
    return (np.array([b.GetXMin(), b.GetYMin(), b.GetZMin()]),
            np.array([b.GetXMax(), b.GetYMax(), b.GetZMax()]))


def volume_cm3(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    return abs(p.Mass()) / 1000.0


def fillet_tip(shape, radius=0.75):
    """把锥尖 y=90 的圆边倒圆 R0.75。"""
    mk = BRepFilletAPI_MakeFillet(shape)
    ex = TopExp_Explorer(shape, TopAbs_EDGE)
    found = 0
    while ex.More():
        e = TopoDS.Edge(ex.Current())
        c = BRepAdaptor_Curve(e)
        y1 = c.Value(c.FirstParameter()).Y()
        y2 = c.Value(c.LastParameter()).Y()
        if min(y1, y2) > 89.9 and max(y1, y2) < 90.1:
            mk.Add(radius, e)
            found += 1
        ex.Next()
    if not found:
        print("[WARN] 没找到锥尖圆边，跳过倒圆")
        return shape, 0
    mk.Build()
    if not mk.IsDone():
        print("[WARN] 锥尖倒圆失败，保留尖边")
        return shape, 0
    return mk.Shape(), found


def build_probe():
    # 1) 主体：定位凸台 + 细颈 + 头部 + 锥体
    s = cyl(15.8, -2.5, 0.0)
    s = fuse(s, cyl(32.0, 0.0, 5.5))
    s = fuse(s, cyl(40.0, 5.5, 9.5))
    cone = BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 9.5, 0), Y_AXIS),
                                12.0, 0.75, 80.5).Shape()
    s = fuse(s, cone)

    # 2) 锥尖 R0.75
    s, nfil = fillet_tip(s, 0.75)
    print(f"锥尖倒圆：{nfil} 条边")

    # 3) 6×Ø3.4 通孔（r=13，0°+k·60°，y 0…9.5）
    holes = None
    for k in range(6):
        a = math.radians(60.0 * k)
        x, z = 13.0 * math.cos(a), 13.0 * math.sin(a)
        ax = gp_Ax2(gp_Pnt(x, -0.6, z), Y_AXIS)
        h = BRepPrimAPI_MakeCylinder(ax, 3.4 / 2.0, 10.7).Shape()
        holes = h if holes is None else fuse(holes, h)
    s = cut(s, holes)

    # 4) 0°/120°/240° 三个沉孔 Ø6.2×3（y 6.5…9.5）
    cbs = None
    for k in (0, 2, 4):
        a = math.radians(60.0 * k)
        x, z = 13.0 * math.cos(a), 13.0 * math.sin(a)
        ax = gp_Ax2(gp_Pnt(x, 6.5, z), Y_AXIS)
        h = BRepPrimAPI_MakeCylinder(ax, 6.2 / 2.0, 3.4).Shape()
        cbs = h if cbs is None else fuse(cbs, h)
    s = cut(s, cbs)

    # 5) 0° 母线刻槽：1.0 宽（z）× 0.6 深，沿锥面从 y=9.5 到 y=89
    s = cut(s, groove_tool())

    # 6) 头部顶面 0° 的小三角凸起（0.8 高，沿 +Y）
    s = fuse(s, marker_bump())

    # 7) 统一同域
    try:
        u = ShapeUpgrade_UnifySameDomain(s, True, True, False)
        u.Build()
        s = u.Shape()
    except Exception as exc:  # noqa: BLE001
        print(f"[WARN] UnifySameDomain 跳过：{exc}")
    return s


def groove_tool():
    """槽刀：沿母线方向的长方体，内表面比锥面低 0.6，z 方向宽 1.0。"""
    r0, y0 = 12.0, 9.5
    r1, y1 = 0.75, 89.0
    p0 = np.array([r0, y0, 0.0])
    p1 = np.array([r1, y1, 0.0])
    u = (p1 - p0) / np.linalg.norm(p1 - p0)          # 母线方向
    n = np.array([u[1], -u[0], 0.0])                  # 外法线（x-y 平面内）
    n = n / np.linalg.norm(n)
    if n[0] < 0:
        n = -n
    w = np.array([0.0, 0.0, 1.0])
    mid = (p0 + p1) / 2.0
    L = float(np.linalg.norm(p1 - p0)) + 2.0
    # 局部盒：x=-L/2…L/2（母线），y=-0.6…+1.2（法线，内 0.6 外 1.2），z=±0.5
    box = BRepPrimAPI_MakeBox(gp_Pnt(-L / 2.0, -0.6, -0.5), L, 1.8, 1.0).Shape()
    trsf = gp_Trsf()
    trsf.SetValues(u[0], n[0], w[0], mid[0],
                   u[1], n[1], w[1], mid[1],
                   u[2], n[2], w[2], mid[2])
    return BRepBuilderAPI_Transform(box, trsf, True).Shape()


def marker_bump():
    """x-y 平面里的三角（r 17→19.5，高 0.8）沿 z 拉 1.6，放在 0°。"""
    pts = [(17.0, 9.5), (19.5, 9.5), (18.25, 10.3)]
    poly = BRepBuilderAPI_MakePolygon()
    for x, y in pts:
        poly.Add(gp_Pnt(x, y, -0.8))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, 1.6)).Shape()


def faces_with_normal(shape, target: np.ndarray, tol=1e-3, want_area=None):
    """列出法向与 target 平行的面（调试用）。"""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane
    out = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face(ex.Current())
        surf = BRepAdaptor_Surface(f)
        if surf.GetType() == GeomAbs_Plane:
            n = surf.Plane().Axis().Direction()
            v = np.array([n.X(), n.Y(), n.Z()])
            if abs(abs(float(np.dot(v, target))) - 1.0) < tol:
                p = GProp_GProps()
                BRepGProp.SurfaceProperties_s(f, p)
                out.append(abs(p.Mass()))
        ex.Next()
    return out


def main() -> int:
    MODELS.mkdir(parents=True, exist_ok=True)
    s = build_probe()
    ok_valid = BRepCheck_Analyzer(s).IsValid()
    n_solids = 0
    ex = TopExp_Explorer(s, TopAbs_SOLID)
    while ex.More():
        n_solids += 1
        ex.Next()

    lo, hi = bbox(s)
    vol = volume_cm3(s)
    print("\n== 探针 probe_L90 自检 ==")
    print(f"有效实体: {ok_valid}；实体数: {n_solids}")
    print(f"包围盒(AddOptimal): x {lo[0]:.3f}..{hi[0]:.3f}  "
          f"y {lo[1]:.3f}..{hi[1]:.3f}  z {lo[2]:.3f}..{hi[2]:.3f}")
    print(f"体积: {vol:.3f} cm3")

    checks = []
    checks.append(("包围盒 x ±20", abs(lo[0] + 20) <= 0.01 and abs(hi[0] - 20) <= 0.01))
    checks.append(("包围盒 z ±20", abs(lo[2] + 20) <= 0.01 and abs(hi[2] - 20) <= 0.01))
    checks.append(("包围盒 y -2.5..90", abs(lo[1] + 2.5) <= 0.01 and abs(hi[1] - 90) <= 0.01))
    checks.append(("体积 19.8..24.2 cm3", 19.8 <= vol <= 24.2))
    checks.append(("沉孔外缘 r=16.1<20", 13 + 3.1 < 20))
    checks.append(("通孔外缘 r=14.7<16", 13 + 1.7 < 16))
    checks.append(("单实体", n_solids == 1))
    checks.append(("形状有效", ok_valid))
    for name, cond in checks:
        print(f"  [{'OK' if cond else 'FAIL'}] {name}")

    step = MODELS / "probe_L90.step"
    stl = MODELS / "probe_L90.stl"
    w = STEPControl_Writer()
    w.Transfer(s, STEPControl_AsIs)
    st = w.Write(str(step))
    print(f"STEP: {step}  ({step.stat().st_size} B, status {st})")
    BRepMesh_IncrementalMesh(s, 0.05, False, 0.3, True)
    sw = StlAPI_Writer()
    sw.ASCIIMode = False
    ok_stl = sw.Write(s, str(stl))
    print(f"STL:  {stl}  ({stl.stat().st_size} B) ok={ok_stl}")

    all_ok = all(c for _, c in checks) and st == 1
    print(f"\n探针自检: {'通过' if all_ok else '不通过'}；"
          f"包围盒 x/z=±20, y=-2.5..90, 体积 {vol:.2f} cm3")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
