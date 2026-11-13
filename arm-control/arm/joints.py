#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节层：配置文件、ID↔SN 核对、使能、参数读写、单位换算。

对外角度一律是“URDF 正方向”的输出端角度：
    out_deg = sign × 电机位置(电机侧圈数) × 10°/圈 + offset
    target_turns = sign × (q_target − offset) / 10
"""
from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field

from . import config as cfgmod
from .sca import (DEG_PER_MOTOR_TURN, GEAR, IQ8, IQ10, IQ24,
                  MODE_CURRENT, MODE_PROFILE_POSITION,
                  R_CURRENT, R_CURRENT_LIMIT, R_CURRENT_MAX, R_ERROR,
                  R_MODE, R_POSITION, R_POS_LIMIT_H, R_POS_LIMIT_L, R_POWER,
                  R_TEMP_MOTOR, R_VEL_LIMIT, R_VOLTAGE,
                  ScaMotor, VELOCITY_FULL_RPM, describe_error)
from .transport import EcbTransport


@dataclass
class JointConfig:
    name: str
    id: int
    sn4: str
    model: str
    sign: int
    offset_deg: float
    eeprom_limits_deg: tuple[float, float]
    soft_limits_deg: tuple[float, float]
    safe_limits_deg: tuple[float, float]
    eeprom_speed_limit_out_deg_s: float
    motion_speed_deg_s: float
    motion_accel_time_s: float
    required_current_limit_a: float | None
    current_check_max_a: float | None
    j2_bracket: str = "PLA"
    zero_date: str = ""
    sign_checked: bool = False
    note: str = ""

    @property
    def gear(self) -> float:
        return GEAR

    def out_deg(self, turns: float) -> float:
        return self.sign * turns * DEG_PER_MOTOR_TURN + self.offset_deg

    def turns(self, out_deg: float) -> float:
        return self.sign * (out_deg - self.offset_deg) / DEG_PER_MOTOR_TURN


@dataclass
class JointState:
    name: str
    id: int
    enabled: bool | None = None
    out_deg: float | None = None
    pos_turns: float | None = None
    target_deg: float | None = None
    error: int | None = None
    error_text: str = ""
    current_a: float | None = None
    voltage_v: float | None = None
    temp_c: float | None = None
    mode: int | None = None
    current_limit_a: float | None = None
    speed_limit_rpm: float | None = None
    pos_limit_lo_r: float | None = None
    pos_limit_hi_r: float | None = None

    def as_dict(self) -> dict:
        def r(v, n=3):
            return None if v is None else round(float(v), n)
        return {
            "name": self.name, "id": self.id, "enabled": self.enabled,
            "out_deg": r(self.out_deg), "pos_turns": r(self.pos_turns, 5),
            "target_deg": r(self.target_deg), "error": self.error,
            "error_text": self.error_text, "current_a": r(self.current_a),
            "voltage_v": r(self.voltage_v, 2), "temp_c": r(self.temp_c, 1),
            "mode": self.mode, "current_limit_a": r(self.current_limit_a, 2),
            "speed_limit_rpm": r(self.speed_limit_rpm, 1),
            "pos_limit_deg": (None if self.pos_limit_hi_r is None else
                              [r(self.pos_limit_lo_r * DEG_PER_MOTOR_TURN, 1),
                               r(self.pos_limit_hi_r * DEG_PER_MOTOR_TURN, 1)]),
        }


def load_joint_configs(path=None) -> dict[str, JointConfig]:
    raw = cfgmod.load_joints()
    out: dict[str, JointConfig] = {}
    for name in raw.get("joint_order", ["J1", "J2", "J3", "J4", "J5", "J6"]):
        j = raw["joints"][name]
        out[name] = JointConfig(
            name=name, id=int(j["id"]), sn4=str(j["sn4"]).upper(),
            model=j.get("model", "?"), sign=int(j.get("sign", 1)),
            offset_deg=float(j.get("offset_deg", 0.0)),
            eeprom_limits_deg=tuple(j["eeprom_limits_deg"]),
            soft_limits_deg=tuple(j["soft_limits_deg"]),
            safe_limits_deg=tuple(j.get("safe_limits_deg", j["soft_limits_deg"])),
            eeprom_speed_limit_out_deg_s=float(j["eeprom_speed_limit_out_deg_s"]),
            motion_speed_deg_s=float(j["motion_speed_deg_s"]),
            motion_accel_time_s=float(j["motion_accel_time_s"]),
            required_current_limit_a=j.get("required_current_limit_a"),
            current_check_max_a=j.get("current_check_max_a"),
            j2_bracket=str(j.get("j2_bracket", "PLA")),
            zero_date=j.get("zero_date", ""),
            sign_checked=bool(j.get("sign_checked", False)),
            note=j.get("note", ""),
        )
    return out


class StartupRefused(RuntimeError):
    pass


class JointBus:
    """真机（ECB）关节总线。"""

    is_mock = False

    def __init__(self, ip: str = "192.168.1.30", port: int = 2000,
                 joint_configs: dict[str, JointConfig] | None = None,
                 debug: bool = False):
        self.cfg = joint_configs or load_joint_configs()
        self.order = list(self.cfg)
        raw_cfg = cfgmod.load_joints()
        self.expected_count = int(raw_cfg.get("expected_count", len(self.order)))
        self.gripper_installed = bool(raw_cfg.get("gripper_installed", False))
        self.link = EcbTransport(ip, port, debug=debug)
        self.motors = {n: ScaMotor(self.link, self.cfg[n].id) for n in self.order}
        self.discovered: dict[int, int] = {}
        self.connected = False
        self.startup_done = False
        self.startup_report: dict = {}
        self._lock = threading.RLock()
        self._enabled_cache: dict[str, bool | None] = {n: None for n in self.order}
        self._targets: dict[str, float] = {}
        self._last_state: dict[str, JointState] = {}
        self._error_tick = 0
        # 20.5：启动/写限位/校准时真正回读的限流（A），不拿配置值冒充
        self.current_limit_readback: dict[str, float | None] = {}

    def reload_config(self, cfg: dict | None = None):
        """重新读 config/joints.json（sign/offset/限位），不重建电机连接。"""
        self.cfg = cfg or load_joint_configs()
        self.order = list(self.cfg)
        self._targets = {}

    # ---------------- 连接 / 启动 ----------------
    def preflight(self) -> dict:
        """握手 + 枚举 + ID↔SN 核对；不使能、不动。"""
        report = {"handshake": False, "scan": {}, "mismatch": [], "ok": False}
        if not self.link.hello(timeout=0.9):
            report["error"] = f"ECB {self.link.ip}:{self.link.port} 没有握手应答"
            self.startup_report = report
            return report
        report["handshake"] = True
        disc = self.link.scan(2.0)
        report["scan"] = {int(k): v for k, v in disc.items()}
        self.discovered = disc
        problems = []
        if len(disc) != self.expected_count:
            problems.append(f"枚举到 {len(disc)} 颗，配置要求 {self.expected_count} 颗"
                            + ("（带夹爪）" if self.gripper_installed else "（不带夹爪）"))
        for name in self.order:
            want = self.cfg[name]
            sn = disc.get(want.id)
            if sn is None:
                problems.append(f"{name}: ID {want.id} 没找到")
                continue
            sn4 = f"{sn:08X}"[-4:]
            if sn4 != want.sn4:
                problems.append(f"{name}: ID {want.id} SN {sn4} ≠ 配置 {want.sn4}")
        report["mismatch"] = problems
        report["ok"] = not problems
        report["ports"] = self.link.stats
        self.connected = report["ok"]
        self.startup_report = report
        return report

    def enable_all(self, confirmed_supported: bool = False) -> dict:
        """16.4 第 1 条启动序列（v2）：

        握手 → 枚举/计数/ID↔SN → 用户确认支撑 → J6→J1 一次一颗
        **零扭矩使能（电流模式，给 0 A）** → 回读限流/速度限制/位置限位/母线电压
        → 成链电压检查 → 全部切 S 曲线保持（目标=当前位置）→ 写梯形参数并回读。

        启动流程**不改写**电机参数；要改走 16.3。
        """
        if not confirmed_supported:
            raise StartupRefused("没有确认臂已支撑，拒绝使能")
        if not self.connected:
            r = self.preflight()
            if not r["ok"]:
                raise StartupRefused("预检未通过：" + "；".join(r.get("mismatch", [])))

        t_start = time.monotonic()
        report = {"enabled": [], "voltages": {}, "zero_torque": [],
                  "warnings": [], "chain": {}}
        positions: dict[str, float] = {}
        hold_ref: dict[str, float] = {}

        # ---- 1) J6→J1：零扭矩使能 + 读核 ----
        for name in reversed(self.order):
            m = self.motors[name]
            c = self.cfg[name]
            # 尽量在使能前就把模式设成电流、电流 0（写命令无回包，失败也继续）
            try:
                m.set_mode(MODE_CURRENT)
                m.set_current_a(0.0)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.04)
            if not m.enable():
                self._disable_all_quiet()
                raise StartupRefused(f"{name} 使能无应答；已全部失能")
            time.sleep(0.05)
            # 使能后立刻零扭矩，避免“使能那一下出力”
            m.set_mode(MODE_CURRENT)
            m.set_current_a(0.0)
            time.sleep(0.12)
            m.set_current_a(0.0)
            mode = m.mode()
            cur = m.current_a()
            pos = m.pos_turns()
            if pos is None:
                m.disable()
                self._disable_all_quiet()
                raise StartupRefused(f"{name} 零扭矩使能后读不到位置；已全部失能")
            tel = m.telemetry(full=True)
            issues = self._validate_enabled(name, tel)
            out_now = c.out_deg(pos)
            lo_s, hi_s = c.safe_limits_deg
            if out_now < lo_s - 0.5 or out_now > hi_s + 0.5:
                issues.append(
                    f"当前位置 {out_now:.1f}° 超出安全限位 {lo_s:.0f}..{hi_s:.0f}°；"
                    "先断电手动移回安全范围内再使能")
            if mode is not None and mode != MODE_CURRENT:
                issues.append(f"模式={mode}，不是电流模式（零扭矩）")
            if cur is not None and abs(cur) > 0.25:
                issues.append(f"零扭矩时电流 {cur:.2f} A > 0.25 A")
            if issues:
                m.disable()
                self._disable_all_quiet()
                raise StartupRefused(
                    f"{name} 启动核对不合格，已把这颗失能并停下："
                    + "；".join(issues))
            positions[name] = pos
            v = m.voltage_v()
            report["voltages"][name] = None if v is None else round(v, 2)
            report["zero_torque"].append({
                "name": name, "mode": mode,
                "current_a": None if cur is None else round(cur, 3),
            })
            report["enabled"].append({
                "name": name, "pos_turns": round(pos, 5),
                "out_deg": round(c.out_deg(pos), 3),
                "current_limit_a": None if tel.current_limit_a is None else round(tel.current_limit_a, 2),
                "speed_limit_rpm": None if tel.speed_limit_rpm is None else round(tel.speed_limit_rpm, 1),
                "pos_limits_deg": None if tel.pos_limit_hi_r is None else [
                    round(tel.pos_limit_lo_r * DEG_PER_MOTOR_TURN, 1),
                    round(tel.pos_limit_hi_r * DEG_PER_MOTOR_TURN, 1)],
                "voltage_v": None if v is None else round(v, 2),
            })
            self.current_limit_readback[name] = tel.current_limit_a
            if c.required_current_limit_a is not None and tel.current_limit_a is not None:
                if abs(tel.current_limit_a - c.required_current_limit_a) > 0.15:
                    report["warnings"].append(
                        f"{name} 限流 {tel.current_limit_a:.2f} A ≠ 要求 "
                        f"{c.required_current_limit_a:.2f} A（启动不改写，记录）")

        # ---- 2) 成链检查：母线电压 ----
        vs = {n: v for n, v in report["voltages"].items() if v is not None}
        if len(vs) < len(self.order):
            self._disable_all_quiet()
            raise StartupRefused("成链检查：有电机读不到母线电压；已全部失能")
        # B4AE（J7）读数比实际低约 1.5 V；现在带夹爪时把 J7 加回去
        for n, v in list(vs.items()):
            if n.upper() == "J7":
                vs[n] = v + 1.5
        vmax = max(vs.values())
        low = {n: v for n, v in vs.items() if vmax - v > 1.0}
        report["chain"] = {"max_v": round(vmax, 2),
                           "voltages": {n: round(v, 2) for n, v in vs.items()}}
        if low:
            self._disable_all_quiet()
            raise StartupRefused(
                "成链检查失败：母线电压比最高低 1.0 V 以上："
                + "；".join(f"{n} {v:.2f} V" for n, v in low.items())
                + f"（最高 {vmax:.2f} V）；已全部失能")

        # ---- 3) 切 S 曲线保持，写梯形参数并回读 ----
        for name in self.order:
            m = self.motors[name]
            c = self.cfg[name]
            pos = m.pos_turns()
            if pos is None:
                self._disable_all_quiet()
                raise StartupRefused(f"{name} 进入保持前读不到位置；已全部失能")
            hold_ref[name] = c.out_deg(pos)
            m.set_target_turns(pos)       # 目标=当前位置，不跳变
            time.sleep(0.02)
            m.set_target_turns(pos)       # 再发一次，降低丢包导致切模式阶跃
            time.sleep(0.02)
            m.set_mode(MODE_PROFILE_POSITION)
            time.sleep(0.08)
            mode_now = m.mode()
            if mode_now != MODE_PROFILE_POSITION:
                m.set_mode(MODE_PROFILE_POSITION)
                time.sleep(0.12)
                mode_now = m.mode()
            if mode_now != MODE_PROFILE_POSITION:
                self._disable_all_quiet()
                raise StartupRefused(
                    f"{name} 没有进入 S 曲线位置模式（当前模式 {mode_now}）；已全部失能")
            self._write_profile(name, c.motion_speed_deg_s)
            time.sleep(0.05)
            prof = m.profile()
            if prof is None:
                self._disable_all_quiet()
                raise StartupRefused(f"{name} 梯形参数读不回来；已全部失能")
            vel_deg_s = prof[0] * DEG_PER_MOTOR_TURN / 60.0
            acc_deg_s2 = prof[1] * DEG_PER_MOTOR_TURN / 60.0
            dec_deg_s2 = prof[2] * DEG_PER_MOTOR_TURN / 60.0
            want_acc = c.motion_speed_deg_s / max(c.motion_accel_time_s, 0.05)
            if abs(vel_deg_s - c.motion_speed_deg_s) > 1.0:
                self._disable_all_quiet()
                raise StartupRefused(
                    f"{name} 梯形最大速度回读 {vel_deg_s:.1f}°/s ≠ "
                    f"{c.motion_speed_deg_s:.1f}°/s；已全部失能")
            if abs(acc_deg_s2 - want_acc) > max(2.0, want_acc * 0.1):
                self._disable_all_quiet()
                raise StartupRefused(
                    f"{name} 梯形加速度回读 {acc_deg_s2:.1f}°/s² ≠ "
                    f"{want_acc:.1f}°/s²；已全部失能")
            self._targets[name] = c.out_deg(pos)
            for item in report["enabled"]:
                if item["name"] == name:
                    item["profile_deg_s"] = round(vel_deg_s, 2)
                    item["profile_acc_deg_s2"] = round(acc_deg_s2, 2)
                    item["profile_dec_deg_s2"] = round(dec_deg_s2, 2)
                    item["mode"] = m.mode()

        # ---- 4) 保持目标确认：如果目标写丢了，不能让它停在旧目标 ----
        pos2 = self.link.read_many(
            [(self.motors[n].mid, R_POSITION, 4) for n in self.order],
            timeout=0.07, retries=3)
        moved = []
        for name in self.order:
            d = pos2.get((self.motors[name].mid, R_POSITION))
            if d is None:
                moved.append(f"{name} 读不到保持位置")
                continue
            p2 = struct.unpack(">i", d)[0] / IQ24
            dq = abs(self.cfg[name].out_deg(p2) - hold_ref.get(name, self.cfg[name].out_deg(p2)))
            if dq > 2.0:
                moved.append(f"{name} 保持位置偏了 {dq:.2f}°")
        if moved:
            self._disable_all_quiet()
            raise StartupRefused(
                "S 曲线保持目标没有确认（这是启动自检，不是电机报警，清报警不能消），"
                "已全部失能：" + "；".join(moved)
                + "。请确认臂已支撑、J2/J3 没被重力带走，再重试")
        self.startup_done = True
        report["ok"] = True
        report["elapsed_s"] = round(time.monotonic() - t_start, 2)
        report["msg"] = "零扭矩使能/读核/成链检查通过；已切 S 曲线保持"
        self.startup_report = report
        return report

    def _validate_enabled(self, name: str, tel) -> list[str]:
        c = self.cfg[name]
        bad = []
        lo, hi = c.eeprom_limits_deg
        if tel.pos_limit_lo_r is None or tel.pos_limit_hi_r is None:
            bad.append("读不到位置限位")
        else:
            lo_d = tel.pos_limit_lo_r * DEG_PER_MOTOR_TURN
            hi_d = tel.pos_limit_hi_r * DEG_PER_MOTOR_TURN
            if lo_d < lo - 0.5 or hi_d > hi + 0.5:
                bad.append(f"位置限位 {lo_d:.1f}..{hi_d:.1f}° 比表 {lo}..{hi}° 宽")
        if tel.speed_limit_rpm is not None:
            v = tel.speed_limit_rpm * DEG_PER_MOTOR_TURN / 60.0
            if v > 70.0:
                bad.append(f"速度限制 {v:.1f}°/s > 70°/s")
        else:
            bad.append("读不到速度限制值")
        # P08-2/P08-6：J2 阈值跟 #0/#16 支架材料走（PLA 4.5 A / PETG100 6.5 A）
        limit = c.current_check_max_a
        if name == "J2":
            limit = 6.5 if c.j2_bracket.upper() == "PETG100" else 4.5
        if limit is not None:
            if tel.current_limit_a is None:
                bad.append("读不到限流值")
            elif tel.current_limit_a > limit:
                bad.append(f"限流 {tel.current_limit_a:.2f} A > {limit:.1f} A（支架 {c.j2_bracket}）")
        if tel.error:
            bad.append(f"报警 0x{tel.error:04X} {describe_error(tel.error)}")
        return bad

    def _write_profile(self, name: str, vel_deg_s: float):
        c = self.cfg[name]
        acc = vel_deg_s / max(c.motion_accel_time_s, 0.05)
        self.motors[name].set_profile_out(vel_deg_s, acc, acc)

    def _disable_all_quiet(self):
        for m in self.motors.values():
            try:
                m.disable()
            except Exception:  # noqa: BLE001
                pass

    def disable_all(self, confirmed: bool = False, reason: str = "") -> bool:
        if not confirmed:
            raise StartupRefused("全部失能要二次确认（臂在零位且已扶住）")
        ok = True
        for name in reversed(self.order):
            ok &= self.motors[name].disable()
            self._enabled_cache[name] = False
        self.startup_done = False
        return ok

    # ---------------- 读 ----------------
    def read_all(self, full: bool = False, fast: bool = True) -> dict[str, JointState]:
        out: dict[str, JointState] = {}
        to = 0.07 if fast else 0.15
        rt = 2 if fast else 3
        names = list(self.order)
        with self._lock:
            ids = [(self.motors[n].mid, n) for n in names]
            def batch(cmd, n, size=2):
                got = {}
                for i in range(0, len(ids), size):
                    chunk = ids[i:i + size]
                    got.update(self.link.read_many(
                        [(mid, cmd, n) for mid, _ in chunk], timeout=to, retries=rt))
                return got
            # 位置 0x06 六颗并发很稳；其它指令 6 颗一起发会丢，按 2 颗一组
            pos = batch(R_POSITION, 4, size=6)
            err = batch(R_ERROR, 2, size=2)
            full_maps: dict[int, dict] = {}
            if full:
                # 状态页只需要电压/温度/模式/电流；限流/速度/限位在启动报告里已有
                specs = [(R_VOLTAGE, 2), (R_TEMP_MOTOR, 2), (R_MODE, 1),
                         (R_CURRENT_MAX, 2), (R_CURRENT, 4), (R_CURRENT_LIMIT, 4)]
                for cmd, n in specs:
                    full_maps[cmd] = batch(cmd, n, size=2)
            for mid, name in ids:
                m = self.motors[name]
                c = self.cfg[name]
                st = JointState(name=name, id=c.id)
                d = pos.get((mid, R_POSITION))
                if d is not None:
                    p = struct.unpack(">i", d)[0] / IQ24
                    st.enabled = True
                    st.pos_turns = p
                    st.out_deg = c.out_deg(p)
                    self._enabled_cache[name] = True
                else:
                    p = m.pos_turns(timeout=to, retries=rt)
                    if p is None:
                        pw = m.power(timeout=to, retries=rt)
                        st.enabled = None if pw is None else (pw == 1)
                        self._enabled_cache[name] = st.enabled
                    else:
                        st.enabled = True
                        st.pos_turns = p
                        st.out_deg = c.out_deg(p)
                        self._enabled_cache[name] = True
                st.target_deg = self._targets.get(name)
                st.current_limit_a = self.current_limit_readback.get(name)

                de = err.get((mid, R_ERROR))
                if de is not None:
                    st.error = struct.unpack(">H", de)[0]
                elif st.enabled:
                    st.error = m.error(timeout=to, retries=rt)
                st.error_text = describe_error(st.error or 0)

                if full and st.enabled:
                    def u(cmd, fmt):
                        dd = full_maps.get(cmd, {}).get((mid, cmd))
                        return struct.unpack(fmt, dd)[0] if dd is not None else None
                    cm = u(R_CURRENT_MAX, ">h")
                    if cm is not None:
                        cm = cm / IQ8
                    v = u(R_VOLTAGE, ">H")
                    st.voltage_v = None if v is None else v / IQ10
                    t = u(R_TEMP_MOTOR, ">h")
                    st.temp_c = None if t is None else t / 256.0
                    md = u(R_MODE, ">B")
                    st.mode = md
                    cur = u(R_CURRENT, ">i")
                    st.current_a = None if cur is None or cm is None else cur / IQ24 * cm
                    cl = u(R_CURRENT_LIMIT, ">i")
                    if cl is not None and cm is not None:
                        st.current_limit_a = cl / IQ24 * cm
                        self.current_limit_readback[name] = st.current_limit_a
                    else:
                        st.current_limit_a = self.current_limit_readback.get(name)
                out[name] = st
                self._last_state[name] = st
        return out

    # ---------------- 写 ----------------
    def set_targets_q(self, targets: dict[str, float]) -> dict[str, bool]:
        res = {}
        with self._lock:
            for name, q in targets.items():
                c = self.cfg[name]
                res[name] = self.motors[name].set_target_turns(c.turns(q))
                self._targets[name] = q
        return res

    def set_profiles_q(self, vel: dict[str, float], acc: dict[str, float] | None = None):
        acc = acc or {}
        for name in self.order:
            if name in vel:
                a = acc.get(name, vel[name] / max(self.cfg[name].motion_accel_time_s, 0.05))
                self.motors[name].set_profile_out(vel[name], a, a)

    def verify_ready(self, names=None) -> list[tuple[str, str]]:
        """检查每颗：电源必须为 1，模式必须为 S 曲线位置 6。

        20.2：批量读丢了的，按颗单独再问一次（单条、带重试）；仍读不到
        才算坏，并把“读不到”和“读到了但值不对”分开报。
        """
        names = list(names or self.order)
        ids = [(self.motors[n].mid, n) for n in names]
        power, mode = {}, {}
        for i in range(0, len(ids), 2):
            chunk = ids[i:i + 2]
            try:
                power.update(self.link.read_many(
                    [(mid, R_POWER, 1) for mid, _ in chunk], timeout=0.06, retries=2))
                mode.update(self.link.read_many(
                    [(mid, R_MODE, 1) for mid, _ in chunk], timeout=0.06, retries=2))
            except Exception:  # noqa: BLE001
                # 通信异常按“读不到”处理，交给单颗重试/上层中止
                pass

        def single(mid: int, cmd: int) -> int | None:
            # 20.2：单条再问，最多 12 次短超时。30% 丢包时 0.3^13 ≈ 1.6e-7，
            # 200 次起步 × 12 个字段的理论误判上限约 4e-4 次。
            for _ in range(12):
                try:
                    d = self.link.read(mid, cmd, 1, timeout=0.06, retries=1)
                except Exception:  # noqa: BLE001
                    d = None
                if d:
                    return d[0]
            return None

        bad: list[tuple[str, str]] = []
        for mid, name in ids:
            p = (power.get((mid, R_POWER)) or b"")[:1]
            m = (mode.get((mid, R_MODE)) or b"")[:1]
            p = p[0] if p else None
            m = m[0] if m else None
            if p is None:
                p = single(mid, R_POWER)
            if m is None:
                m = single(mid, R_MODE)
            if p is None and m is None:
                bad.append((name, "读不到电源(0x2B)和模式(0x55)"))
            elif p is None:
                bad.append((name, "读不到电源(0x2B)"))
            elif m is None:
                bad.append((name, "读不到模式(0x55)"))
            elif p != 1 and m != MODE_PROFILE_POSITION:
                bad.append((name, f"读到了但 power={p},mode={m}"))
            elif p != 1:
                bad.append((name, f"读到了但 power={p}"))
            elif m != MODE_PROFILE_POSITION:
                bad.append((name, f"读到了但 mode={m}"))
        return bad

    # ---------------- 18.3 停机：单颗失能 / 重新使能到保持 ----------------
    def read_positions_turns(self) -> dict[str, float]:
        out: dict[str, float] = {}
        for n in self.order:
            p = self.motors[n].pos_turns()
            if p is not None:
                out[n] = p
        return out

    def disable_one(self, name: str) -> bool:
        return bool(self.motors[name].disable())

    def enable_one_hold(self, name: str, pos_turns: float) -> bool:
        """把一颗重新使能到“目标 = 该位置”的 S 曲线保持（停机流程报警时用）。"""
        m = self.motors[name]
        c = self.cfg[name]
        try:
            m.set_mode(MODE_CURRENT)
            m.set_current_a(0.0)
        except Exception:  # noqa: BLE001
            pass
        if not m.enable():
            return False
        time.sleep(0.1)
        # 20.9.1：先写目标=当前位置，再切位置模式，再写一次目标
        m.set_target_turns(pos_turns)
        self._write_profile(name, c.motion_speed_deg_s)
        m.set_mode(MODE_PROFILE_POSITION)
        m.set_target_turns(pos_turns)
        self._targets[name] = c.out_deg(pos_turns)
        return True

    def current_limit_a(self, name: str) -> float | None:
        try:
            return self.motors[name].current_limit_a()
        except Exception:  # noqa: BLE001
            return None

    def current_raw_counts(self, name: str):
        """18.4：返回 (0x04 原始计数, 0x59 原始计数)。"""
        try:
            return self.motors[name].current_raw_counts()
        except Exception:  # noqa: BLE001
            return (None, None)

    def current_full_scale_a(self, name: str) -> float | None:
        """0x53 原始满量程（A）。18.4 要求记这个，不是换算后的限流。"""
        try:
            return self.motors[name].current_max_a()
        except Exception:  # noqa: BLE001
            return None

    def hold_all(self) -> dict:
        """逐颗尽力把目标改写成当前位置，并切回 S 曲线位置模式。

        20.1：不因个别关节读不到/没确认就整体抛异常。返回：
            {"ok": bool, "held": {name: deg},
             "unconfirmed": [{"name": n, "reason": r}],
             "unreadable": [name, ...]}
        """
        held: dict[str, float] = {}
        unconfirmed: list[dict] = []
        unreadable: list[str] = []
        seen: set[str] = set()

        def add(name: str, reason: str):
            if name not in seen:
                seen.add(name)
                unconfirmed.append({"name": name, "reason": reason})

        with self._lock:
            for name in self.order:
                m = self.motors[name]
                try:
                    pos = m.pos_turns(timeout=0.15, retries=2)
                except Exception:  # noqa: BLE001
                    pos = None
                if pos is None:
                    unreadable.append(name)
                    add(name, "读不到位置")
                    continue
                try:
                    m.set_target_turns(pos)
                    m.set_mode(MODE_PROFILE_POSITION)
                    m.set_target_turns(pos)
                except Exception as exc:  # noqa: BLE001
                    add(name, f"写目标/切模式失败：{exc}")
                    continue
                held[name] = self.cfg[name].out_deg(pos)
                self._targets[name] = held[name]
        if held:
            for n, why in self.verify_ready(list(held)):
                add(n, why)
        return {"ok": not unconfirmed, "held": held,
                "unconfirmed": unconfirmed, "unreadable": unreadable}

    def stats(self) -> dict:
        return self.link.stats

    def close(self):
        self.link.close()
