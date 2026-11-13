#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无硬件模拟：和 JointBus 同接口，按梯形速度把关节推向目标。

用于网页/运动层离线测试（--mock），不连接 ECB。

20.0：MockJointBus 带故障注入（仅 mock）：
    silent        某一颗完全不应答
    drop_prob     指定指令回包按概率丢（R_POSITION/R_POWER/R_MODE/...）
    delay_rounds  指定指令的回包延迟一轮，下一轮恢复
    error_bits    某一颗报警位非零
    limits        限流回读值（A）
    force_mode    模式回读值（默认 6）
"""
from __future__ import annotations

import math
import random
import threading
import time

from .joints import JointState, load_joint_configs
from .sca import (MODE_PROFILE_POSITION, R_CURRENT_LIMIT, R_ERROR, R_MODE,
                  R_POSITION, R_POWER)


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
        return {"ok": True, "held": dict(self.q),
                "unconfirmed": [], "unreadable": []}

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
        # 20.0 故障注入（仅 mock）
        self.silent: set[str] = set()
        self.drop_prob: dict[int, float] = {}
        self.delay_rounds: dict[tuple[str, int], int] = {}
        self.error_bits: dict[str, int] = {}
        self.force_mode: dict[str, int] = {}
        self.current_override: dict[str, float] = {}
        self.current_limit_readback: dict[str, float | None] = {
            n: 3.0 for n in self.order}
        self.mock_fault_log: list[str] = []

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
        for n in self.order:
            self.current_limit_readback.setdefault(n, 3.0)
        enabled = [{"name": n, "out_deg": round(self.q[n], 3),
                    "pos_turns": round(self.q[n] / 10.0, 5),
                    "current_limit_a": self.current_limit_readback.get(n),
                    "speed_limit_rpm": 360.0,
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

    # ---------------- 20.0 故障注入 ----------------
    def _lost(self, name: str, cmd: int) -> bool:
        """按概率丢包；delay_rounds 让这一轮丢、下一轮恢复。"""
        if name in self.silent:
            return True
        key = (name, cmd)
        if self.delay_rounds.get(key, 0) > 0:
            self.delay_rounds[key] -= 1
            self.mock_fault_log.append(f"delay {name} cmd=0x{cmd:02X}")
            return True
        p = float(self.drop_prob.get(cmd, 0.0))
        if p > 0 and random.random() < p:
            self.mock_fault_log.append(f"drop {name} cmd=0x{cmd:02X}")
            return True
        return False

    def set_faults(self, silent=None, drop=None, delay=None, error_bits=None,
                   limits=None, mode=None, currents=None):
        """drop: {"position"/"power"/"mode"/"error"/"current": 概率}；
        delay: {"J4": 1} 或 1（所有关节位置延迟轮数）；
        error_bits: {关节: 位}；limits: {关节: A}；mode: {关节: 模式}。"""
        if silent is not None:
            self.silent = {str(x).upper() for x in silent}
        if drop is not None:
            names = {"position": R_POSITION, "power": R_POWER,
                     "mode": R_MODE, "error": R_ERROR, "current": R_CURRENT_LIMIT}
            self.drop_prob = {names[k]: float(v) for k, v in drop.items() if k in names}
        if delay is not None:
            if isinstance(delay, dict):
                self.delay_rounds = {(str(k).upper(), R_POSITION): int(v)
                                     for k, v in delay.items()}
            else:
                self.delay_rounds = {(n, R_POSITION): int(delay) for n in self.order}
        if error_bits is not None:
            self.error_bits = {str(k).upper(): int(v) for k, v in error_bits.items()}
        if limits is not None:
            for k, v in limits.items():
                self.current_limit_readback[str(k).upper()] = None if v is None else float(v)
        if mode is not None:
            self.force_mode = {str(k).upper(): int(v) for k, v in mode.items()}
        if currents is not None:
            self.current_override = {str(k).upper(): float(v) for k, v in currents.items()}

    def clear_faults(self):
        self.silent = set()
        self.drop_prob = {}
        self.delay_rounds = {}
        self.error_bits = {}
        self.force_mode = {}
        self.current_override = {}
        self.mock_fault_log = []

    def mock_faults(self):
        return {"silent": sorted(self.silent),
                "drop_prob": {f"0x{k:02X}": v for k, v in self.drop_prob.items()},
                "delay_rounds": {f"{k[0]}:0x{k[1]:02X}": v
                                 for k, v in self.delay_rounds.items()},
                "error_bits": dict(self.error_bits),
                "force_mode": dict(self.force_mode),
                "limits": dict(self.current_limit_readback)}

    # ---------------- 读 ----------------
    def read_all(self, full: bool = False):
        with self._lock:
            self._tick()
            out = {}
            for n in self.order:
                c = self.cfg[n]
                err = self.error_bits.get(n, self._errors.get(n, 0))
                pos_lost = self._lost(n, R_POSITION)
                power_lost = self._lost(n, R_POWER)
                mode_lost = self._lost(n, R_MODE)
                error_lost = self._lost(n, R_ERROR)
                enabled = (self.enabled and n not in self.off)
                out[n] = JointState(
                    name=n, id=c.id,
                    enabled=None if power_lost else enabled,
                    pos_turns=None if pos_lost else c.turns(self.q[n]),
                    out_deg=None if pos_lost else self.q[n],
                    target_deg=self.target[n],
                    error=None if error_lost else err,
                    error_text="" if error_lost else ("报警" if err else "正常"),
                    current_a=(None if full and self._lost(n, R_CURRENT_LIMIT)
                               else self.current_override.get(n, 0.0)),
                    voltage_v=24.0, temp_c=30.0,
                    mode=None if mode_lost else self.force_mode.get(
                        n, MODE_PROFILE_POSITION),
                    current_limit_a=self.current_limit_readback.get(n),
                    speed_limit_rpm=360.0,
                    pos_limit_lo_r=c.eeprom_limits_deg[0] / 10.0,
                    pos_limit_hi_r=c.eeprom_limits_deg[1] / 10.0)
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
        """20.1：和真机 JointBus 同返回结构；silent 关节算“读不到”。"""
        with self._lock:
            self.writes += 1
            self._tick()
            held = {}
            unconfirmed = []
            unreadable = []
            for n in self.order:
                if n in self.silent:
                    unreadable.append(n)
                    unconfirmed.append({"name": n, "reason": "读不到位置"})
                    continue
                self.target[n] = self.q[n]
                self.v[n] = 0.0
                held[n] = self.q[n]
                if not self.enabled:
                    unconfirmed.append({"name": n, "reason": "读到了但 power=0"})
            for name, why in self.verify_ready():
                if not any(x["name"] == name for x in unconfirmed):
                    unconfirmed.append({"name": name, "reason": why})
            return {"ok": not unconfirmed, "held": held,
                    "unconfirmed": unconfirmed, "unreadable": unreadable}

    # ---------------- 18.3 单颗失能 / 重新使能 ----------------
    def read_positions_turns(self):
        with self._lock:
            self._tick()
            out = {}
            for n in self.order:
                if n in self.silent:
                    continue
                out[n] = self.cfg[n].turns(self.q[n])
            return out

    def disable_one(self, name):
        with self._lock:
            self.off.add(name)
            self.target[name] = self.q[name]
            self.v[name] = 0.0
            return True

    def enable_one_hold(self, name, pos_turns):
        with self._lock:
            if name in self.silent:
                return False
            self.off.discard(name)
            self.q[name] = self.cfg[name].out_deg(pos_turns)
            self.target[name] = self.q[name]
            self.v[name] = 0.0
            return True

    def current_limit_a(self, name):
        return self.current_limit_readback.get(name)

    def current_full_scale_a(self, name):
        return 33.0

    def current_raw_counts(self, name):
        return (0, 0)

    def verify_ready(self, names=None):
        names = list(names or self.order)
        if not self.enabled:
            return [(n, "读到了但 power=0") for n in names]
        bad = []
        for n in names:
            if n in self.silent:
                bad.append((n, "读不到电源(0x2B)和模式(0x55)"))
                continue
            p = None if self._lost(n, R_POWER) else 1
            m = None if self._lost(n, R_MODE) else self.force_mode.get(
                n, MODE_PROFILE_POSITION)
            # 模拟“单独再问一次”
            if p is None:
                p = None if self._lost(n, R_POWER) else 1
            if m is None:
                m = None if self._lost(n, R_MODE) else self.force_mode.get(
                    n, MODE_PROFILE_POSITION)
            if p is None and m is None:
                bad.append((n, "读不到电源(0x2B)和模式(0x55)"))
            elif p is None:
                bad.append((n, "读不到电源(0x2B)"))
            elif m is None:
                bad.append((n, "读不到模式(0x55)"))
            elif p != 1 or m != MODE_PROFILE_POSITION:
                bad.append((n, f"读到了但 power={p},mode={m}"))
        return bad

    def stats(self):
        return {"tx": 0, "rx": 0, "mock": True, "faults": self.mock_faults()}
