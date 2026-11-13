#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动层：同步点到点、路径、预演、中止、记录。

要求（16.4）：
  * 同步点到点：T = max |Δ|/v，≥0.5 s；各关节按 |Δ|/T 配梯形速度；
  * 路径：相邻点插 10 个中间点，每点 FK + 围栏，任何一点不过就整条拒绝；
  * 预演只算不发；
  * 中止 = 保持（目标改成当前位置），不是失能；
  * 全部失能只在“已回零并扶住”或“通信完全断了”时做；
  * 18.2：每条路径的起点、终点、10 个插补点都过重力力矩预检；
    被拒绝时自动重试一次（先把 q4–q6 走到目标，再走 q1–q3）；
  * 18.3：停机路径（抬高 → 收拢 → 预停机 → P）用同一套检查。
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from . import gravity
from .fence import Fence
from .kinematics import RobotModel
from .recorder import Recorder


class MotionError(RuntimeError):
    pass


class TorqueError(MotionError):
    """18.2 重力力矩预检不通过（可以尝试“远端先收”重试）。"""


class StartFenceError(MotionError):
    """16.4 第 11 条：起点在围栏外；只允许零扭矩拖动或先抬高再回零。"""


class Motion:
    def __init__(self, bus, robot: RobotModel, fence: Fence,
                 limits: tuple[list[float], list[float]],
                 recorder: Recorder | None = None,
                 poll_hz: float = 25.0):
        self.bus = bus
        self.robot = robot
        self.fence = fence
        self.limits = (np.asarray(limits[0], float), np.asarray(limits[1], float))
        self.recorder = recorder
        self.poll_hz = max(10.0, float(poll_hz))
        self.order = list(bus.order)
        self.preset = "probe"
        self.yaw_deg = 0.0
        # 18.2：末端配置 / 载荷 / 回读限流 / Kt 标定结果，由执行器刷新
        self.end_effector = "flange"
        self.payload_kg = 0.0
        self.limit_a: dict[str, float] = {}
        self.kt: dict = {}
        self.j2_bracket = "PLA"          # P08-9：PLA 8 N·m / PETG100 12 N·m
        self._stop = threading.Event()
        self._stop_reason = "用户按停"
        self._lock = threading.RLock()
        self.status = {
            "busy": False, "phase": "idle", "reason": "", "label": "",
            "progress": 0.0, "current_q": None, "target_q": None,
            "segments": [], "started": None, "finished": None,
        }

    # ---------------- 基础 ----------------
    def read_q(self) -> list[float]:
        states = self.bus.read_all(full=False)
        q = []
        for name in self.order:
            v = states[name].out_deg
            if v is None:
                raise MotionError(f"{name} 无位置读数（失能或掉线）")
            q.append(float(v))
        return q

    def read_states(self):
        return self.bus.read_all(full=False)

    def q_dict(self, q) -> dict[str, float]:
        return {n: float(v) for n, v in zip(self.order, q)}

    def _near_limit(self, q) -> list[dict]:
        out = []
        for i, n in enumerate(self.order):
            m = min(q[i] - self.limits[0][i], self.limits[1][i] - q[i])
            if m < 10.0:
                out.append({"joint": n, "margin_deg": round(float(m), 2)})
        return out

    def check_torque(self, q) -> list[dict]:
        """18.2 预检：单个姿态，返回超限项。"""
        return gravity.check_q(self.robot, q, self.end_effector,
                               self.payload_kg, self.limit_a, self.kt,
                               j2_bracket=self.j2_bracket)

    def capacities(self) -> dict:
        return gravity.capacities(self.limit_a, self.kt, self.j2_bracket)

    def start_problem(self, q) -> str | None:
        """16.4 第 11 条：把当前实际姿态也过一次围栏。"""
        fr = self.fence.check(q, self.preset, self.yaw_deg, False, self.limits)
        return None if fr.ok else "；".join(fr.reasons)

    def _check_q(self, q, descend: bool = False,
                 relax_below_z: float | None = None) -> dict:
        for i, n in enumerate(self.order):
            if not (self.limits[0][i] - 1e-9 <= q[i] <= self.limits[1][i] + 1e-9):
                raise MotionError(
                    f"{n} 目标 {q[i]:.2f}° 超出安全限位 "
                    f"{self.limits[0][i]:.0f}..{self.limits[1][i]:.0f}°")
        fr = self.fence.check(q, self.preset, self.yaw_deg, descend, self.limits,
                              relax_below_z)
        if not fr.ok:
            raise MotionError("围栏不通过：" + "；".join(fr.reasons))
        bad = self.check_torque(q)
        if bad:
            b = bad[0]
            raise TorqueError(
                f"力矩预检不通过：{b['joint']} {b['tau']:+.2f} N·m "
                f"> 0.8×{b['capacity']:.2f}（{b['ratio'] * 100:.0f}%）"
                f"，姿态 {tuple(round(float(v), 1) for v in q)}"
                + ("；末端 " + self.end_effector
                   + (f" + {self.payload_kg:g} kg" if self.payload_kg else "")))
        return fr.as_dict()

    def _interp(self, q0, q1, n=10):
        return [np.asarray(q0, float) * (1 - t) + np.asarray(q1, float) * t
                for t in np.linspace(0, 1, n + 1)]

    def _plan_joint(self, q_from, q_target, label: str, descend: bool = False,
                    relax_below_z: float | None = None) -> dict:
        q0 = np.asarray(q_from, float)
        q1 = np.asarray(q_target, float)
        # 目标先查
        fr = self._check_q(q1, descend, relax_below_z)
        # 中间 10 点；起点另外由 _plan_move 的 start_problem 检查
        for p in self._interp(q0, q1, 10)[1:]:
            self._check_q(p, descend, relax_below_z)
        d = q1 - q0
        vmax = np.array([self.bus.cfg[n].motion_speed_deg_s for n in self.order])
        T = max(0.5, float(np.max(np.abs(d) / vmax))) if np.max(np.abs(d)) > 0 else 0.5
        vel = {}
        acc = {}
        for i, n in enumerate(self.order):
            v = abs(float(d[i])) / T
            v = max(0.25, min(vmax[i], v))
            vel[n] = v
            acc[n] = v / max(self.bus.cfg[n].motion_accel_time_s, 0.05)
        return {
            "label": label,
            "q_from": [round(float(v), 4) for v in q0],
            "q_target": [round(float(v), 4) for v in q1],
            "descend": bool(descend),
            "T": round(T, 3),
            "vel_deg_s": {k: round(v, 3) for k, v in vel.items()},
            "acc_deg_s2": {k: round(v, 3) for k, v in acc.items()},
            "fence": fr,
            "near_limit": self._near_limit(q1),
        }

    # ---------------- 规划 ----------------
    def _plan_move(self, q0, q1, label: str, descend: bool = False,
                   allow_outside_start: bool = False,
                   relax_below_z: float | None = None) -> list[dict]:
        """18.2 第 2 条：被力矩预检拒绝的路径自动重试一次（远端先收）。

        中间点 = q1 的 q4–q6 + q0 的 q1–q3：先把腕走到目标，再走大臂。
        16.4 第 11 条：起点也要过围栏；只有“抬高”段 allow_outside_start=True。
        """
        if not allow_outside_start:
            problem = self.start_problem(q0)
            if problem:
                raise StartFenceError(
                    "起点在围栏外（" + problem + "）：只允许零扭矩拖动，"
                    "或先“竖直抬高”再回零；普通目标一律拒绝")
        try:
            return [self._plan_joint(q0, q1, label, descend, relax_below_z)]
        except TorqueError as exc:
            # P08-10：先“远端先收”，还不过再反过来“伸臂后伸腕”
            last = exc
            for first in ("wrist", "arm"):
                try:
                    return self._split_move(q0, q1, label, descend,
                                            relax_below_z, first, str(last))
                except TorqueError as e2:
                    last = e2
            raise last

    def _split_move(self, q0, q1, label, descend, relax_below_z,
                    first: str, note: str) -> list[dict]:
        q1 = list(q1)
        if first == "wrist":
            mid = list(q0)
            mid[3:] = q1[3:]
            tags = ("（远端先收 q4–q6）", "（再走 q1–q3）")
        else:
            mid = list(q1)
            mid[3:] = list(q0)[3:]
            tags = ("（先走 q1–q3）", "（再走 q4–q6）")
        if max(abs(mid[i] - q0[i]) for i in range(6)) < 0.5:
            raise TorqueError(note)
        segs = [self._plan_joint(q0, mid, label + tags[0], descend, relax_below_z),
                self._plan_joint(mid, q1, label + tags[1], descend, relax_below_z)]
        for s in segs:
            s["torque_retry"] = True
            s["torque_retry_order"] = first
            s["torque_note"] = note
        return segs

    def plan_joint_move(self, q_target, q_from=None, descend: bool = False,
                        label: str = "move_joints") -> list[dict]:
        q0 = self.read_q() if q_from is None else list(q_from)
        return self._plan_move(q0, q_target, label, descend)

    def plan_tcp_move(self, target_mm, tool_axis=(0, 0, -1), yaw_deg=None,
                      q_from=None, preset: str | None = None,
                      descend: bool = False, label: str = "move_tcp",
                      free_yaw: bool = False, fast: bool = False,
                      allow_outside_start: bool = False,
                      relax_below_z: float | None = None) -> list[dict]:
        old_preset, old_yaw = self.preset, self.yaw_deg
        if preset:
            self.preset = preset
        if yaw_deg is not None:
            self.yaw_deg = yaw_deg
        try:
            q0 = self.read_q() if q_from is None else list(q_from)
            use_yaw = None if free_yaw else self.yaw_deg
            kw = {"max_iters": 60, "time_budget_s": 0.6} if fast else {}
            res = self.robot.ik(target_mm, tool_axis, use_yaw, q0,
                                self.preset, (self.limits[0], self.limits[1]), **kw)
            if not res.ok:
                tx, ty, tz = (round(float(v), 1) for v in target_mm)
                raise MotionError(
                    f"IK 无解（目标 {tx},{ty},{tz}；偏航{'自由' if free_yaw else '锁 0°'}）："
                    f"残差 {res.pos_err_mm:.2f} mm / 轴 {res.axis_err_deg:.2f}°；"
                    "往可达区拖，或勾“偏航自由”再试")
            segs = self._plan_move(q0, res.q_deg, label, descend,
                                   allow_outside_start, relax_below_z)
            segs[-1]["ik"] = res.as_dict()
            return segs
        finally:
            self.preset, self.yaw_deg = old_preset, old_yaw

    def plan_click(self, x: float, y: float, z_target: float = 20.0,
                   yaw_deg: float = 0.0, q_from=None,
                   fast: bool = False) -> list[dict]:
        """16.5 的三段路径：抬高 → 平移到目标上方 z=100 → 下探到 z=20。"""
        q0 = self.read_q() if q_from is None else list(q_from)
        old_yaw = self.yaw_deg
        self.yaw_deg = yaw_deg
        try:
            tcp0 = self.robot.tcp(q0, self.preset, yaw_deg)[:3, 3]
            segs: list[dict] = []
            cur = q0
            if tcp0[2] < 100.0 - 0.5:
                lift = self.plan_tcp_move(
                    [tcp0[0], tcp0[1], 100.0], (0, 0, -1), yaw_deg,
                    cur, label="抬到 z=100", fast=fast)
                segs.extend(lift)
                cur = lift[-1]["q_target"]
            mid = self.plan_tcp_move(
                [x, y, 100.0], (0, 0, -1), yaw_deg, cur,
                label="平移到目标上方", fast=fast)
            segs.extend(mid)
            fin = self.plan_tcp_move(
                [x, y, z_target], (0, 0, -1), yaw_deg, mid[-1]["q_target"],
                descend=True, label="下探", fast=fast)
            segs.extend(fin)
            return segs
        finally:
            self.yaw_deg = old_yaw

    def plan_home(self, q_from=None) -> list[dict]:
        """16.5 第 4 条：抬高 → 一步同步插补回 q=0。

        2026-11-09（P05/18.2）去掉“先收 J5 到 0”和“收臂到 (0,30,30)”两段：
        J5 转到 0 对 J4 没有卸载作用（J4 的 0.57 N·m 不变），反而给 J5 加载。
        大关节的过载由 18.2 预检 + 远端先收重试处理。
        """
        q0 = self.read_q() if q_from is None else list(q_from)
        segs: list[dict] = self._plan_lift(q0)
        cur = segs[-1]["q_target"] if segs else list(q0)
        segs.extend(self._plan_move(cur, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "回零位"))
        return segs

    def plan_through(self, waypoints, descend_flags=None, labels=None) -> list[dict]:
        """路径：相邻点之间再插 10 个中间点，全查 FK + 围栏 + 力矩，任一点不过整条拒绝。"""
        q0 = self.read_q()
        pts = [list(q0)] + [list(w) for w in waypoints]
        flags = descend_flags or [False] * (len(pts) - 1)
        segs: list[dict] = []
        for i in range(1, len(pts)):
            segs.extend(self._plan_move(
                pts[i - 1], pts[i],
                (labels[i - 1] if labels else f"path {i}"),
                bool(flags[i - 1])))
        return segs

    # ---------------- 18.3 停机 ----------------
    def default_park(self) -> dict:
        side = getattr(self, "park_side", -1)
        q1 = 90.0 * side          # park_side=-1 → 臂沿 −y，J1 = −90°；+1 → +90°
        return {
            "park_side": side,
            "poses": {
                "tuck": [q1, 30.0, 30.0, 90.0, 0.0, 0.0],
                "pre": [q1, 76.5, 13.5, 90.0, 0.0, 0.0],
                "park": [q1, 85.0, 5.0, 90.0, 0.0, 0.0],
            },
        }

    def _plan_lift(self, q0) -> list[dict]:
        """抬高到 TCP z ≥ 150（回零和停机共用）。起点围栏不过时仍允许抬高。"""
        try:
            tcp = self.robot.tcp(q0, self.preset, self.yaw_deg)[:3, 3]
            if tcp[2] < 150.0 - 0.5:
                return self.plan_tcp_move([tcp[0], tcp[1], 150.0], (0, 0, -1),
                                          self.yaw_deg, q0, label="抬高",
                                          allow_outside_start=True,
                                          relax_below_z=100.0)
        except MotionError:
            pass
        return []

    def plan_park(self, q_from=None, park: dict | None = None,
                  include_down: bool = True) -> dict:
        """18.3 停机：抬高 → 收拢+折腕+转 J1 → 预停机点 → P。

        返回四段列表；执行器在 pre / park 到达后各停 1 s 读 I₀、I₁。
        """
        q0 = self.read_q() if q_from is None else list(q_from)
        pk = park or self.default_park()
        self.park_side = int(pk.get("park_side", getattr(self, "park_side", -1)))
        poses = pk.get("poses") or {}
        lift = self._plan_lift(q0)
        cur = lift[-1]["q_target"] if lift else list(q0)
        tuck = self._plan_move(cur, poses["tuck"], "收拢+折腕+转 J1")
        pre = self._plan_move(poses["tuck"], poses["pre"], "J2/J3 到预停机点")
        down = (self._plan_move(poses["pre"], poses["park"], "下降到 P")
                if include_down else [])
        return {"lift": lift, "tuck": tuck, "pre": pre, "park": down,
                "poses": poses}

    def plan_unpark(self, q_from=None, park: dict | None = None) -> list[dict]:
        """18.3 从停机启动：J2/J3 同步抬回预停机点（离开搁架）。"""
        q0 = self.read_q() if q_from is None else list(q_from)
        pk = park or self.default_park()
        self.park_side = int(pk.get("park_side", getattr(self, "park_side", -1)))
        return self._plan_move(q0, (pk.get("poses") or {})["pre"], "抬回预停机点")

    # ---------------- 执行 ----------------
    def begin_command(self):
        """20.3：接受新命令时清停止标志并立即置 busy。

        这样从 /api/execute 返回“已启动”那一刻起，按“停”一定有效，
        不会掉进“线程还没跑到 execute”的空档。
        """
        with self._lock:
            self._stop.clear()
            self._stop_reason = "用户按停"
            self.status.update({
                "busy": True, "phase": "starting", "reason": "",
                "started": time.time(), "finished": None, "progress": 0.0,
            })

    def stop(self, reason: str = "用户按停"):
        # 20.3/20.7：停止标志无条件置位；不忙时也不能把 aborted/error 洗成 idle
        self._stop.set()
        self._stop_reason = reason
        with self._lock:
            if self.status.get("busy"):
                self.status["phase"] = "stopping"
                self.status["reason"] = reason
            elif self.status.get("phase") not in ("aborted", "error"):
                self.status["phase"] = "idle"
                self.status["reason"] = ""

    def _abort(self, reason: str):
        """中止 = 原地保持；只有六颗全部读不到（通信完全断）才全部失能。"""
        res = None
        try:
            res = self.bus.hold_all()
        except Exception as exc:  # noqa: BLE001
            reason += f"（保持时通信失败：{exc}）"
        unreadable = list((res or {}).get("unreadable") or [])
        unconfirmed = list((res or {}).get("unconfirmed") or [])
        if len(unreadable) >= len(self.order):
            reason += "；六颗都读不到位置（通信完全断），按 16.4 全部失能"
            try:
                self.bus.disable_all(True, "通信中断")
            except Exception:  # noqa: BLE001
                pass
        elif unconfirmed:
            reason += "；未确认保持：" + "，".join(
                f"{x.get('name')}({x.get('reason')})" for x in unconfirmed)
        with self._lock:
            self.status["phase"] = "aborted"
            self.status["reason"] = reason
        if self.recorder:
            self.recorder.command("abort", reason)
        raise MotionError(reason)

    def _run_segment(self, seg: dict):
        q_target = np.asarray(seg["q_target"], float)
        bad = self.bus.verify_ready()
        if bad:
            self._abort("这些关节没有真正使能/不在位置模式，拒绝发目标："
                        + "，".join(f"{n}({m})" for n, m in bad))
        self.bus.set_profiles_q(seg["vel_deg_s"], seg["acc_deg_s2"])
        with self._lock:
            self.status.update({"phase": "moving", "label": seg["label"],
                                "target_q": seg["q_target"], "progress": 0.0})
        self.bus.set_targets_q(self.q_dict(q_target))
        if self.recorder:
            for n, q in zip(self.order, q_target):
                self.recorder.command("command_move", seg["label"], n, float(q))
        t0 = time.monotonic()
        # 计划 T 只按 20°/s 匀速估，实际还有加速段/摩擦/轮询；给足余量
        timeout = max(5.0, seg["T"] * 1.5 + 3.0)
        last_ok = {n: t0 for n in self.order}
        consec = 0
        err_ref = {n: None for n in self.order}
        err_ref_t = {n: t0 for n in self.order}
        while True:
            if self._stop.is_set():
                self._abort(self._stop_reason)
            states = self.bus.read_all(full=False)
            if self.recorder:
                self.recorder.poll(states)
            now = time.monotonic()
            q = []
            errs: dict[str, float | None] = {}
            for i, n in enumerate(self.order):
                st = states[n]
                if st.out_deg is None:
                    q.append(None)
                    errs[n] = None
                    if now - last_ok[n] > 0.5:
                        self._abort(f"{n} 0.5 s 没应答")
                    continue
                last_ok[n] = now
                q.append(st.out_deg)
                errs[n] = abs(st.out_deg - q_target[i])
                lo_s, hi_s = self.bus.cfg[n].safe_limits_deg
                if st.out_deg < lo_s - 1.0 or st.out_deg > hi_s + 1.0:
                    self._abort(
                        f"{n} 实际位置 {st.out_deg:.1f}° 超出安全限位 "
                        f"{lo_s:.0f}..{hi_s:.0f}°（撞限位）")
                if st.error:
                    self._abort(f"{n} 报警 0x{st.error:04X} {st.error_text}")
            if None not in q:
                maxerr = max(errs.values())
                # 0.8° 以内算到位；J3 这种有摩擦/背隙的关节常停在 0.3~0.5°
                if maxerr < 0.8:
                    consec += 1
                else:
                    consec = 0
                if consec >= 3:
                    with self._lock:
                        self.status["phase"] = "arrived"
                        self.status["progress"] = 1.0
                        self.status["current_q"] = [round(v, 4) for v in q]
                    if self.recorder:
                        for i, n in enumerate(self.order):
                            err = float(q[i] - q_target[i])
                            self.recorder.command(
                                "arrived_error",
                                f"{seg['label']} 到位误差 {err:+.3f}°",
                                n, float(q_target[i]), pos_deg=float(q[i]),
                                error=states[n].error)
                    return
            # 16.4 第 7 条（P08-4 口径）：逐关节判，>5° 且连续 0.3 s 内
            # 误差下降不到 1° → 中止。数值不动；一颗小关节卡住不会等到
            # 大关节走完/超时才发现。
            if now - t0 > 0.35:
                for n in self.order:
                    e = errs.get(n)
                    if e is None:
                        continue
                    if e > 5.0:
                        ref = err_ref.get(n)
                        if ref is None or e <= ref - 1.0:
                            err_ref[n] = e
                            err_ref_t[n] = now
                        elif now - err_ref_t.get(n, t0) > 0.3:
                            self._abort(
                                f"{n} 实际位置和目标差 {e:.1f}° >5°，"
                                "且 0.3 s 内下降不到 1°")
                    else:
                        err_ref[n] = None
                        err_ref_t[n] = now
            if now - t0 > timeout:
                self._abort(f"到达超时（{timeout:.1f} s）")
            if None not in q:
                with self._lock:
                    self.status["current_q"] = [round(v, 4) for v in q]
                    denom = max(seg["T"], 0.2)
                    self.status["progress"] = round(min(0.99, (now - t0) / denom), 3)
            time.sleep(max(0.0, 1.0 / self.poll_hz - (time.monotonic() - now)))

    def execute(self, segments: list[dict], preview: bool = False,
                pause_between: float = 0.2, clear_stop: bool = True) -> dict:
        """执行（或预演）一组已规划好的段。预演绝不写电机。

        clear_stop=False 用于“命令已在接受时 begin_command() 过”，
        避免把刚按下的停又清掉。
        """
        if clear_stop:
            self._stop.clear()
            self._stop_reason = "用户按停"
        elif self._stop.is_set():
            self._abort(self._stop_reason or "启动前已按停")
        for seg in segments:
            self._check_q(seg["q_target"], seg.get("descend", False))
        with self._lock:
            self.status.update({
                "busy": True, "phase": "preview" if preview else "running",
                "reason": "", "segments": segments, "started": time.time(),
                "finished": None, "progress": 0.0,
            })
        if self.recorder:
            self.recorder.command("plan" if preview else "execute_start",
                                  f"{len(segments)} 段" + ("（预演，不发写命令）" if preview else ""))
        if preview:
            with self._lock:
                self.status.update({"busy": False, "phase": "preview_done"})
            return {"preview": True, "segments": segments}
        try:
            for i, seg in enumerate(segments):
                with self._lock:
                    self.status["index"] = i
                self._run_segment(seg)
                if i < len(segments) - 1:
                    time.sleep(max(0.0, pause_between))
            with self._lock:
                self.status.update({"busy": False, "phase": "done",
                                    "finished": time.time(), "progress": 1.0})
            if self.recorder:
                self.recorder.command("execute_done", "全部段到达")
            return {"ok": True, "segments": segments}
        except MotionError as exc:
            with self._lock:
                self.status.update({"busy": False, "phase": "aborted",
                                    "reason": str(exc), "finished": time.time()})
            raise
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.status.update({"busy": False, "phase": "error",
                                    "reason": str(exc), "finished": time.time()})
            try:
                res = self.bus.hold_all()
                un = (res or {}).get("unconfirmed") or []
                if un:
                    with self._lock:
                        self.status["reason"] += "；未确认保持：" + "，".join(
                            f"{x.get('name')}({x.get('reason')})" for x in un)
            except Exception:  # noqa: BLE001
                pass
            raise
