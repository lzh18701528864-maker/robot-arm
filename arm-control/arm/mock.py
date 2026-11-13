#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无硬件模拟：和 JointBus 同接口，按梯形速度把关节推向目标。

用于网页/运动层离线测试（--mock），不连接 ECB。
"""
from __future__ import annotations

import math
import threading
import time

from .joints import JointState, load_joint_configs


class VirtualJointBus:
    """离线预演用的假总线：六轴固定在 q=0，只读不算写。

    电机没连/没使能时，网页显示零位姿态、IK/围栏照常预演；执行会被执行器拒绝。
    """

    is_mock = False
    is_virtual = True

    def __init__(self, cfg=None):
        self.cfg = cfg or load_joint_configs()
        self.order = list(self.cfg)
        self.startup_done = False
        self.q = {n: 0.0 for n in self.order}

    def read_all(self, full: bool = False):
        out = {}
        for n in self.order:
            c = self.cfg[n]
            out[n] = JointState(name=n, id=c.id, enabled=False,
                                pos_turns=0.0, out_deg=0.0, target_deg=0.0,
                                error=0, error_text="",
                                current_a=None, voltage_v=None, temp_c=None,
                                mode=None, current_limit_a=None,
                                speed_limit_rpm=None,
                                pos_limit_lo_r=c.eeprom_limits_deg[0] / 10.0,
                                pos_limit_hi_r=c.eeprom_limits_deg[1] / 10.0)
        return out

    def set_targets_q(self, targets):
        return {n: True for n in targets}

    def set_profiles_q(self, vel, acc=None):
        return None

    def hold_all(self):
        return dict(self.q)

    def stats(self):
        return {"virtual": True}


class MockJointBus:
    is_mock = True

    def __init__(self, initial_deg=None):
        self.cfg = load_joint_configs()
        self.order = list(self.cfg)
        self.q = {n: float((initial_deg or {}).get(n, 0.0)) for n in self.order}
        self.target = dict(self.q)
        self.v = {n: 0.0 for n in self.order}
        self.vmax = {n: self.cfg[n].motion_speed_deg_s for n in self.order}
        self.amax = {n: self.cfg[n].motion_speed_deg_s / self.cfg[n].motion_accel_time_s
                     for n in self.order}
        self.enabled = True
        self.connected = True
        self.startup_done = True
        self.discovered = {self.cfg[n].id: int(self.cfg[n].sn4, 16) for n in self.order}
        self.startup_report = {"ok": True, "mock": True}
        self._lock = threading.RLock()
        self._t = time.monotonic()
        self._errors = {n: 0 for n in self.order}
        self.writes = 0
        self.off = set()          # 18.3：被单独失能的关节

    # ---------------- 生命周期 ----------------
    def preflight(self):
        self.connected = True
        self.discovered = {self.cfg[n].id: int(self.cfg[n].sn4, 16) for n in self.order}
        return {"ok": True, "mock": True, "handshake": True,
                "scan": self.discovered, "mismatch": []}

    def enable_all(self, confirmed_supported: bool = False):
        if not confirmed_supported:
            from .joints import StartupRefused
            raise StartupRefused("没有确认臂已支撑，拒绝使能")
        self.enabled = True
        self.startup_done = True
        enabled = [{"name": n, "out_deg": round(self.q[n], 3),
                    "pos_turns": round(self.q[n] / 10.0, 5),
                    "current_limit_a": 3.0, "speed_limit_rpm": 360.0,
                    "pos_limits_deg": list(self.cfg[n].eeprom_limits_deg),
                    "voltage_v": 24.0, "profile_deg_s": 20.0} for n in reversed(self.order)]
        return {"ok": True, "mock": True, "enabled": enabled,
                "zero_torque": [{"name": n, "mode": 1, "current_a": 0.0}
                                for n in reversed(self.order)],
                "chain": {"max_v": 24.0, "voltages": {n: 24.0 for n in self.order}},
                "warnings": []}

    def disable_all(self, confirmed: bool = False, reason: str = ""):
        if not confirmed:
            from .joints import StartupRefused
            raise StartupRefused("全部失能要二次确认")
        self.enabled = False
        self.startup_done = False
        return True

    def close(self):
        pass

    # ---------------- 仿真 ----------------
    def _tick(self):
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self._t))
        self._t = now
        if not self.enabled:
            return
        for n in self.order:
            err = self.target[n] - self.q[n]
            dist = abs(err)
            # 允许的最高速度同时受减速距离限制
            v_allow = math.sqrt(max(0.0, 2.0 * self.amax[n] * dist))
            desired = min(self.vmax[n], v_allow) * (1.0 if err >= 0 else -1.0)
            dv = desired - self.v[n]
            step = self.amax[n] * dt
            self.v[n] += max(-step, min(step, dv))
            move = self.v[n] * dt
            if abs(move) >= dist:
                self.q[n] = self.target[n]
                self.v[n] = 0.0
            else:
                self.q[n] += move

    # ---------------- 读 ----------------
    def read_all(self, full: bool = False):
        with self._lock:
            self._tick()
            out = {}
            for n in self.order:
                c = self.cfg[n]
                st = JointState(name=n, id=c.id,
                                enabled=(self.enabled and n not in self.off),
                                pos_turns=c.turns(self.q[n]), out_deg=self.q[n],
                                target_deg=self.target[n], error=self._errors[n],
                                error_text="正常" if not self._errors[n] else "模拟报警",
                                current_a=0.0, voltage_v=24.0, temp_c=30.0,
                                mode=6, current_limit_a=c.required_current_limit_a,
                                speed_limit_rpm=360.0,
                                pos_limit_lo_r=c.eeprom_limits_deg[0] / 10.0,
                                pos_limit_hi_r=c.eeprom_limits_deg[1] / 10.0)
                out[n] = st
            return out

    # ---------------- 写 ----------------
    def set_targets_q(self, targets):
        with self._lock:
            self.writes += 1
            self._tick()
            for n, q in targets.items():
                lo, hi = self.cfg[n].soft_limits_deg
                self.target[n] = max(lo, min(hi, float(q)))
            return {n: True for n in targets}

    def set_profiles_q(self, vel, acc=None):
        self.writes += 1
        acc = acc or {}
        for n in self.order:
            if n in vel:
                self.vmax[n] = max(0.2, abs(float(vel[n])))
                a = abs(float(acc.get(n, vel[n] / max(self.cfg[n].motion_accel_time_s, 0.05))))
                self.amax[n] = max(0.2, a)

    def hold_all(self):
        with self._lock:
            self.writes += 1
            self._tick()
            for n in self.order:
                self.target[n] = self.q[n]
                self.v[n] = 0.0
            return dict(self.q)

    # ---------------- 18.3 单颗失能 / 重新使能 ----------------
    def read_positions_turns(self):
        with self._lock:
            self._tick()
            return {n: self.cfg[n].turns(self.q[n]) for n in self.order}

    def disable_one(self, name):
        with self._lock:
            self.off.add(name)
            self.target[name] = self.q[name]
            self.v[name] = 0.0
            return True

    def enable_one_hold(self, name, pos_turns):
        with self._lock:
            self.off.discard(name)
            self.q[name] = self.cfg[name].out_deg(pos_turns)
            self.target[name] = self.q[name]
            self.v[name] = 0.0
            return True

    def current_limit_a(self, name):
        return 3.0

    def current_full_scale_a(self, name):
        return 33.0

    def current_raw_counts(self, name):
        return (0, 0)

    def verify_ready(self, names=None):
        if not self.enabled:
            return [(n, "mock 未使能") for n in (names or self.order)]
        return []

    def stats(self):
        return {"tx": 0, "rx": 0, "mock": True}
