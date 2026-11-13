#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16.3 网页校准会话：设零 + 方向核对。

设计成执行器里的一个“会话”，网页通过 /api/calib/* 驱动：
    prepare      其余关节全部失能 → 目标关节零扭矩使能 → 可手动摆
    set_zero     清 Homing → 当前位置设零 → 回读 |q|<0.05° → 写限位/限流/限速
                 → 存 EEPROM → 全部回读比对（不符就恢复原值、不保存）
    jog          电机正方向 +5°（输出端），保持住给用户看
    confirm_sign 用户选 ±1，写配置；回 raw 0 并失能
    return_zero  不写 sign，先回 raw 0（可重点动）
    abort        失能目标关节，结束会话
"""
from __future__ import annotations

import datetime as _dt
import json
import threading
import time
from pathlib import Path

from . import config as cfgmod
from .joints import load_joint_configs
from .sca import (DEG_PER_MOTOR_TURN, MODE_CURRENT, MODE_HOMING,
                  MODE_PROFILE_POSITION, ScaMotor, describe_error, out_deg_to_turns)
from .transport import EcbTransport

REFERENCE = {
    "J6": ("探针刻槽 / K 孔在 12 点钟方向（+z，竖直向上）。这是最里面的一颗，"
           "先做 J6。"),
    "J5": ("J6 电机轴平行 J4 电机轴，J6 法兰朝 +y（和 J4 法兰同向）。"
           "从上往下看 J4、J6 两个 Ø52 圆筒外形平行；目测 ±1°。"),
    "J4": ("J5 电机圆筒竖直。手机测角 App 贴 J5 电机 Ø52 圆筒侧面，"
           "在 x–z 平面内读竖直，±0.3°。"),
    "J3": ("前臂铝管竖直。手机测角 App 贴前臂铝管，在 x–z 平面内读竖直，"
           "±0.3°。"),
    "J2": ("上臂铝管竖直。手机测角 App 贴铝管，在 x–z 平面内读竖直，"
           "±0.3°。"),
    "J1": ("J2 电机轴平行 y；面向 +x 时上臂在左手边（+y 侧）。"
           "用 #32 的边当参照，目测 ±1° 即可，剩余误差以后标定吸收。"),
}
EXPECTED = {
    "J1": "从上往下看，整臂逆时针转",
    "J2": "上臂向 +x（工作区方向）倾倒",
    "J3": "前臂相对上臂向 +x 方向弯",
    "J4": "J5 电机向 +x 方向倾",
    "J5": "从上往下看，J6 电机连探针逆时针转",
    "J6": "站在 +y 侧正对法兰看，K 孔从 12 点往你的左手边（+x）转",
}


def busy_op(fn):
    """标记校准会话正在执行一条长命令；这期间心跳超时先不中止。"""
    def wrapper(self, *args, **kwargs):
        self.busy = True
        try:
            return fn(self, *args, **kwargs)
        finally:
            self.busy = False
    return wrapper


def patch_joint(name: str, **kw):
    """原子地改 config/joints.json（单关节字段）。"""
    path = cfgmod.CONFIG_DIR / "joints.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["joints"][name].update(kw)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(raw, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


class CalibSession:
    def __init__(self, ex):
        self.ex = ex
        self.lock = threading.RLock()
        self.transport: EcbTransport | None = None
        self.motor: ScaMotor | None = None
        self.joint: str | None = None
        self.stage = "idle"          # idle/ready/zeroed/jogged/done/error
        self.msg = ""
        self.pos_turns: float | None = None
        self.out_deg: float | None = None
        self.power: int | None = None
        self.error: int | None = None
        self.battery_v: float | None = None
        self.mode: int | None = None
        self.current_a: float | None = None
        self.params: dict | None = None
        self.sign_pending: int | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._jog_from: float | None = None
        self.busy = False

    # ---------------- 状态 ----------------
    @property
    def active(self) -> bool:
        return self.joint is not None and self.transport is not None

    def snapshot(self) -> dict:
        with self.lock:
            cfg = load_joint_configs()
            done = {n: bool(c.zero_date and c.sign_checked) for n, c in cfg.items()}
            return {
                "active": self.active,
                "busy": bool(self.busy),
                "joint": self.joint,
                "stage": self.stage,
                "msg": self.msg,
                "pos_turns": None if self.pos_turns is None else round(self.pos_turns, 6),
                "out_deg": None if self.out_deg is None else round(self.out_deg, 3),
                "power": self.power,
                "error": self.error,
                "error_text": describe_error(self.error or 0),
                "battery_v": None if self.battery_v is None else round(self.battery_v, 2),
                "mode": self.mode,
                "current_a": None if self.current_a is None else round(self.current_a, 3),
                "params": self.params,
                "reference": REFERENCE.get(self.joint or "", ""),
                "expected": EXPECTED.get(self.joint or "", ""),
                "done": done,
                "all_done": all(done.values()),
                "order": ["J6", "J5", "J4", "J3", "J2", "J1"],
                "zeroed": {n: bool(c.zero_date) for n, c in cfg.items()},
                "sign_checked": {n: bool(c.sign_checked) for n, c in cfg.items()},
                "signs": {n: c.sign for n, c in cfg.items()},
            }

    def _set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    # ---------------- 连接/准备 ----------------
    @busy_op
    def prepare(self, joint: str, confirmed_supported: bool) -> dict:
        if not confirmed_supported:
            raise RuntimeError("没有确认臂已支撑，拒绝使能")
        if self.ex.mode == "mock":
            raise RuntimeError("模拟模式不能做真机校准；请用真机执行器")
        if self.ex.motion and self.ex.motion.status.get("busy"):
            raise RuntimeError("执行器正在运动，先停")
        if self.ex.bus and getattr(self.ex.bus, "startup_done", False):
            raise RuntimeError("执行器的关节还处于使能状态，先全部失能再校准")
        joint = joint.upper()
        if joint not in load_joint_configs():
            raise RuntimeError(f"未知关节 {joint}")
        self.abort(quiet=True)

        t = EcbTransport(self.ex.ecb_ip, self.ex.ecb_port)
        if not t.hello():
            t.close()
            raise RuntimeError(f"ECB {self.ex.ecb_ip}:{self.ex.ecb_port} 没有握手应答")
        disc = t.scan(2.0)
        cfg = load_joint_configs()
        if not disc:
            t.close()
            raise RuntimeError("ECB 没发现执行器")
        # 全部 ID↔SN 核对，任何不符拒绝
        bad = []
        for n, c in cfg.items():
            sn = disc.get(c.id)
            if sn is None:
                bad.append(f"{n} ID {c.id} 没找到")
            elif f"{sn:08X}"[-4:] != c.sn4:
                bad.append(f"{n} ID {c.id} SN {f'{sn:08X}'[-4:]} ≠ {c.sn4}")
        if bad:
            t.close()
            raise RuntimeError("ID↔SN 不一致：" + "；".join(bad))
        target = cfg[joint]
        target_mid = target.id

        # 其余关节全部失能
        still_on = []
        for n, c in cfg.items():
            if n == joint:
                continue
            om = ScaMotor(t, c.id)
            try:
                om.disable()
                if om.power() == 1:
                    still_on.append(n)
            except Exception:  # noqa: BLE001
                still_on.append(n)
        if still_on:
            t.close()
            raise RuntimeError("这些关节没有确认失能：" + ", ".join(still_on)
                               + "；先手动/用网页全部失能再校准")
        m = ScaMotor(t, target_mid)
        # 零扭矩使能：先尽量把模式设成电流、电流 0，再使能；使能后立刻再给 0 A
        try:
            m.set_mode(MODE_CURRENT)
            m.set_current_a(0.0)
        except Exception:  # noqa: BLE001
            pass
        if not m.enable():
            t.close()
            raise RuntimeError(f"{joint} 使能无应答")
        time.sleep(0.05)
        m.set_mode(MODE_CURRENT)
        m.set_current_a(0.0)
        time.sleep(0.15)
        m.clear_error()
        pos = m.pos_turns()
        if pos is None:
            m.disable()
            t.close()
            raise RuntimeError(f"{joint} 零扭矩使能后读不到位置")
        mode = m.mode()
        cur = m.current_a()
        if mode is not None and mode != MODE_CURRENT:
            # 再试一次切电流模式
            m.set_mode(MODE_CURRENT)
            m.set_current_a(0.0)
            time.sleep(0.1)
            mode = m.mode()
        if mode is not None and mode != MODE_CURRENT:
            m.disable()
            t.close()
            raise RuntimeError(f"{joint} 切不到电流模式（当前 {mode}），不能保证零扭矩")
        if cur is not None and abs(cur) > 0.3:
            m.disable()
            t.close()
            raise RuntimeError(f"{joint} 零扭矩时电流 {cur:.2f} A > 0.3 A，拒绝继续")
        time.sleep(0.1)
        pos = m.pos_turns()
        batt = m.battery_v()
        if batt is not None and batt < 2.5:
            m.disable()
            t.close()
            raise RuntimeError(f"电池 {batt:.2f} V < 2.5 V，先通电充电，不做")
        self.transport = t
        self.motor = m
        self.joint = joint
        self.stage = "ready"
        self.msg = (f"{joint} 已零扭矩使能（电流模式 0 A），输出端可用手转动。"
                    "把关节摆到参考姿态后点“设为零位”。")
        self.pos_turns = pos
        self.out_deg = None if pos is None else pos * DEG_PER_MOTOR_TURN
        self.power = 1
        self.error = 0
        self.battery_v = batt
        self.mode = mode
        self.current_a = cur
        self.params = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll, daemon=True)
        self._thread.start()
        self.ex._calib_active = True
        return self.snapshot()

    def _poll(self):
        while not self._stop.is_set():
            m = self.motor
            if not m:
                break
            try:
                p = m.pos_turns(timeout=0.2, retries=1)
                pw = m.power(timeout=0.2, retries=1)
                err = m.error(timeout=0.2, retries=1) if pw == 1 else None
                md = m.mode(timeout=0.2, retries=1) if pw == 1 else None
                cu = m.current_a() if pw == 1 else None
                with self.lock:
                    self.power = pw
                    self.pos_turns = p
                    self.out_deg = None if p is None else p * DEG_PER_MOTOR_TURN
                    self.error = err
                    self.mode = md
                    self.current_a = cu
                if pw == 0:
                    self._set(msg="关节已失能（可能是掉线/保护），校准中止", stage="error")
                    break
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.15)

    # ---------------- 设零 ----------------
    @busy_op
    def set_zero(self, joint: str) -> dict:
        self._require(joint, ("ready", "zeroed", "jogged"))
        m = self.motor
        cfg = load_joint_configs()[self.joint]
        assert m is not None
        p = m.pos_turns()
        if p is None:
            raise RuntimeError("读不到位置")
        old = {"limits": m.pos_limits_turns(),
               "current": m.current_limit_a(),
               "speed": m.speed_limit_rpm()}
        # 0x88/0x87 这套 Homing 命令只在 homing 模式（0x08）下可靠生效；
        # 电流模式 0 A 下实测 0x87 不改变读数。切 homing 再设零，输出端仍是自由的。
        m.set_mode(MODE_HOMING)
        time.sleep(0.25)
        if not m.clear_homing():
            raise RuntimeError("清除 Homing 数据发送失败；不继续")
        time.sleep(0.1)
        ok_zero = m.set_home_turns(0.0)
        time.sleep(0.1)
        p = m.pos_turns()
        if p is None:
            raise RuntimeError("设零后读不到位置")
        out = p * DEG_PER_MOTOR_TURN
        if not ok_zero or abs(out) >= 0.05:
            m.disable()
            self._set(stage="error",
                      msg=f"设零后回读 {out:.3f}°，未到 0（要求 |q|<0.05°），不写 EEPROM")
            raise RuntimeError(self.msg)

        lim = cfg.eeprom_limits_deg
        hi_t, lo_t = out_deg_to_turns(lim[1]), out_deg_to_turns(lim[0])
        m.set_pos_limits_turns(lo_t, hi_t)
        if cfg.required_current_limit_a is not None:
            m.set_current_limit_a(cfg.required_current_limit_a)
        m.set_speed_limit_out_deg_s(cfg.eeprom_speed_limit_out_deg_s)
        m.save()
        time.sleep(0.4)
        lim2 = m.pos_limits_turns()
        cur2 = m.current_limit_a()
        spd2 = m.speed_limit_rpm()
        pos2 = m.pos_turns()
        problems = []
        if lim2 is None or abs(lim2[0] - lo_t) > 0.01 or abs(lim2[1] - hi_t) > 0.01:
            problems.append(f"限位回读 {lim2}")
        if cfg.required_current_limit_a is not None and (
                cur2 is None or abs(cur2 - cfg.required_current_limit_a) > 0.05):
            problems.append(f"限流回读 {cur2}")
        if spd2 is None or abs(spd2 - 360.0) > 1.0:
            problems.append(f"速度限制回读 {spd2}")
        if pos2 is None or abs(pos2 * DEG_PER_MOTOR_TURN) >= 0.05:
            problems.append(f"位置回读 {pos2}")
        if problems:
            # 恢复原值，不保留新参数
            try:
                if old["limits"]:
                    m.set_pos_limits_turns(*old["limits"])
                if old["current"] is not None and cfg.required_current_limit_a is not None:
                    m.set_current_limit_a(old["current"])
                if old["speed"] is not None:
                    m.set_speed_limit_out_deg_s(old["speed"] * DEG_PER_MOTOR_TURN / 60.0)
                m.save()
            except Exception:  # noqa: BLE001
                pass
            m.disable()
            self._set(stage="error", msg="回读不符，已恢复原值：" + "；".join(problems))
            raise RuntimeError(self.msg)

        today = _dt.date.today().isoformat()
        patch_joint(self.joint, zero_date=today)
        self.ex.refresh_config_version()
        self.params = {
            "limits_deg": [lim[0], lim[1]],
            "current_limit_a": None if cur2 is None else round(cur2, 3),
            "speed_limit_rpm": None if spd2 is None else round(spd2, 1),
            "zero_date": today,
        }
        # 进入位置模式，准备方向核对
        m.set_mode(MODE_PROFILE_POSITION)
        m.set_profile_out(5.0, 5.0)
        pos_now = m.pos_turns() or 0.0
        m.set_target_turns(pos_now)
        self._set(stage="zeroed",
                  msg=f"{self.joint} 零位已存（{today}）。下一步：按电机正方向点动 +5°。")
        return self.snapshot()

    # ---------------- 方向核对 ----------------
    @busy_op
    def jog(self, delta: float = 5.0) -> dict:
        self._require(self.joint, ("zeroed", "jogged"))
        m = self.motor
        assert m is not None
        m.set_mode(MODE_PROFILE_POSITION)
        m.set_profile_out(5.0, 5.0)
        p0 = m.pos_turns()
        if p0 is None:
            raise RuntimeError("读不到位置")
        m.set_target_turns(p0)
        time.sleep(0.2)
        p0 = m.pos_turns() or p0
        target = p0 + delta / DEG_PER_MOTOR_TURN
        m.set_target_turns(target)
        t0 = time.monotonic()
        last = p0
        while time.monotonic() - t0 < 5.0:
            time.sleep(0.1)
            p = m.pos_turns(timeout=0.2, retries=1)
            if p is None:
                continue
            last = p
            if abs(p - target) < 0.02:
                break
        e = m.error() or 0
        self._jog_from = p0
        self._set(stage="jogged", pos_turns=last,
                  out_deg=last * DEG_PER_MOTOR_TURN, error=e,
                  msg=f"已按电机正方向走 {delta:+.1f}°（实际 {(last-p0)*DEG_PER_MOTOR_TURN:+.2f}°）。"
                      f"请判断方向；错误 0x{e:04X}")
        if e:
            raise RuntimeError(f"点动后报警 0x{e:04X} {describe_error(e)}；先停下处理")
        return self.snapshot()

    @busy_op
    def confirm_sign(self, sign: int) -> dict:
        self._require(self.joint, ("jogged",))
        if sign not in (-1, 1):
            raise RuntimeError("sign 只能是 +1 或 -1")
        patch_joint(self.joint, sign=int(sign), sign_checked=True)
        self.ex.refresh_config_version()
        m = self.motor
        assert m is not None
        # 回 raw 0，失能
        try:
            m.set_mode(MODE_PROFILE_POSITION)
            m.set_profile_out(5.0, 5.0)
            m.set_target_turns(0.0)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 5.0:
                time.sleep(0.1)
                p = m.pos_turns(timeout=0.2, retries=1)
                if p is not None and abs(p) < 0.02:
                    break
        finally:
            m.disable()
        self._set(stage="done", sign_pending=int(sign),
                  msg=f"{self.joint} 完成：sign = {sign:+d}，已回零位并失能。")
        self._stop.set()
        self._close_transport()
        self.ex._calib_active = False
        return self.snapshot()

    @busy_op
    def return_zero(self) -> dict:
        self._require(self.joint, ("jogged", "zeroed"))
        m = self.motor
        assert m is not None
        m.set_mode(MODE_PROFILE_POSITION)
        m.set_profile_out(5.0, 5.0)
        m.set_target_turns(0.0)
        t0 = time.monotonic()
        p = None
        while time.monotonic() - t0 < 5.0:
            time.sleep(0.1)
            p = m.pos_turns(timeout=0.2, retries=1)
            if p is not None and abs(p) < 0.02:
                break
        self._set(stage="zeroed", pos_turns=p,
                  out_deg=None if p is None else p * DEG_PER_MOTOR_TURN,
                  msg="已回零位。可以再次点动 +5° 或选方向。")
        return self.snapshot()

    def abort(self, quiet: bool = False, reason: str = "用户中止校准") -> dict:
        self._stop.set()
        m = self.motor
        if m is not None:
            try:
                m.disable()
            except Exception:  # noqa: BLE001
                pass
        self._close_transport()
        self.ex._calib_active = False
        if not quiet:
            self._set(stage="idle", msg=reason)
        return self.snapshot()

    def _close_transport(self):
        t = self.transport
        self.transport = None
        self.motor = None
        if t is not None:
            try:
                t.close()
            except Exception:  # noqa: BLE001
                pass

    def _require(self, joint, stages):
        if not self.active or not self.joint:
            raise RuntimeError("没有活动的校准会话；先点“零扭矩使能”")
        if joint and joint.upper() != self.joint:
            raise RuntimeError(f"当前校准的是 {self.joint}")
        if self.stage not in stages:
            raise RuntimeError(f"当前步骤 {self.stage}，不允许这个操作")
        if self.power != 1:
            raise RuntimeError("关节没有使能")
