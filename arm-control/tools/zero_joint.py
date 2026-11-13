#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16.3 单关节设零 / 方向核对（按第十一节的 EEPROM 流程）。

这是一个“人机配合”的状态机，不是一脚到底的脚本：
    prepare   使能该关节 + 切 homing 模式（输出端可手动摆），打印实时位置
    read      只读：状态、位置、电压、电池、错误
    set-zero  清 Homing → 当前位置设零 → 回读 |q|<0.05° → 写限位/限流/限速
              → 存 EEPROM → 全部回读比对（任何不符就恢复原值、不保存）
    jog       按“电机正方向”走 +5°（输出端），给用户看方向；不自动回位
    return    回到 jog 前记录的位置
    set-sign  把方向核对结果写进 config/joints.json
    disable   失能该关节
    params    读当前限位/限流/限速/模式

用法（例）：
    python3 tools/zero_joint.py read --joint J6
    python3 tools/zero_joint.py prepare --joint J6
    python3 tools/zero_joint.py set-zero --joint J6
    python3 tools/zero_joint.py jog --joint J6 --delta 5
    python3 tools/zero_joint.py set-sign --joint J6 --sign -1
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from arm import config as cfgmod  # noqa: E402
from arm.joints import load_joint_configs  # noqa: E402
from arm.sca import (DEG_PER_MOTOR_TURN, MODE_HOMING, MODE_PROFILE_POSITION,
                     ScaMotor, describe_error, out_deg_to_turns)  # noqa: E402
from arm.transport import EcbTransport  # noqa: E402

STATE_FILE = ROOT / "runs" / "zero_state.json"


def load_state() -> dict:
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return {}


def save_state(st: dict):
    STATE_FILE.parent.mkdir(exist_ok=True)
    STATE_FILE.write_text(json.dumps(st, ensure_ascii=False, indent=2), encoding="utf-8")


def patch_joint(name: str, **kw):
    path = cfgmod.CONFIG_DIR / "joints.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["joints"][name].update(kw)
    path.write_text(json.dumps(raw, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def connect_and_check(args):
    cfg = load_joint_configs()[args.joint]
    link = EcbTransport(args.ip, args.ecb_port)
    if not link.hello():
        raise SystemExit(f"ECB {args.ip}:{args.ecb_port} 没有握手应答")
    disc = link.scan(2.0)
    sn = disc.get(cfg.id)
    if sn is None:
        raise SystemExit(f"ECB 没发现 {args.joint}（ID {cfg.id}）")
    sn4 = f"{sn:08X}"[-4:]
    if sn4 != cfg.sn4:
        raise SystemExit(f"{args.joint} ID {cfg.id} 的 SN 是 {sn4}，配置是 {cfg.sn4}；拒绝操作")
    return link, cfg, ScaMotor(link, cfg.id)


def cmd_read(args):
    link, cfg, m = connect_and_check(args)
    p = m.power()
    print(f"{cfg.name} ID {cfg.id} SN {cfg.sn4} 模式={m.mode()} 使能={p} "
          f"位置_turns={m.pos_turns()} 输出_deg={None if m.pos_turns() is None else round(m.pos_turns()*DEG_PER_MOTOR_TURN, 3)}")
    if p == 1:
        print(f"  错误=0x{(m.error() or 0):04X} {describe_error(m.error() or 0)}")
        print(f"  电压={m.voltage_v()} V  电池(0xB7)={m.battery_v()} V  温度={m.temp_c()} C")
        lim = m.pos_limits_turns()
        print(f"  限位={lim} turns  限流={m.current_limit_a()} A  "
              f"速度限制={m.speed_limit_rpm()} RPM  梯形={m.profile()}")
    else:
        print("  （失能状态这颗固件不回答位置/参数；先 prepare）")
    link.close()
    return 0


def cmd_prepare(args):
    link, cfg, m = connect_and_check(args)
    p = m.power()
    if p != 1:
        if not m.enable():
            raise SystemExit("使能无应答")
        time.sleep(0.15)
        m.clear_error()
    pos = m.pos_turns()
    if pos is None:
        raise SystemExit("使能后读不到位置")
    m.set_target_turns(pos)
    if not m.set_mode(MODE_HOMING):
        print("[WARN] 切 homing 模式未确认")
    time.sleep(0.2)
    pos = m.pos_turns()
    print(f"{cfg.name} 已进入 homing 模式（输出端应可用手转动）。")
    print(f"当前位置：{pos:.5f} turns = {pos*DEG_PER_MOTOR_TURN:.3f}° 输出")
    print(f"错误：0x{(m.error() or 0):04X} {describe_error(m.error() or 0)}")
    print(f"电池(0xB7)：{m.battery_v()} V；母线：{m.voltage_v()} V")
    st = load_state()
    st[cfg.name] = {"prepare_pos_turns": pos, "prepare_time": _dt.datetime.now().isoformat(timespec="seconds")}
    save_state(st)
    print("把该关节摆到参考姿态，然后运行：")
    print(f"  python3 tools/zero_joint.py set-zero --joint {cfg.name}")
    link.close()
    return 0


def cmd_set_zero(args):
    link, cfg, m = connect_and_check(args)
    if m.power() != 1:
        raise SystemExit("关节没使能；先 prepare")
    mode = m.mode()
    if mode != MODE_HOMING:
        print(f"[WARN] 当前模式 {mode}，不是 homing(8)；仍按当前姿态设零")
    pos_before = m.pos_turns()
    if pos_before is None:
        raise SystemExit("读不到位置")
    out_before = pos_before * DEG_PER_MOTOR_TURN
    print(f"{cfg.name} 设零前：{pos_before:.5f} turns = {out_before:.3f}° 输出")

    old = {
        "limits": m.pos_limits_turns(),
        "current": m.current_limit_a(),
        "speed": m.speed_limit_rpm(),
    }
    print(f"原参数：限位={old['limits']} 限流={old['current']} A 速度限制={old['speed']} RPM")

    # 1) clear homing → 2) 当前位置设零
    if not m.clear_homing():
        raise SystemExit("清除 Homing 数据无应答；不继续")
    time.sleep(0.1)
    if not m.set_home_turns(0.0):
        raise SystemExit("设置 Homing 位置无应答；不继续")
    time.sleep(0.25)
    pos_after = m.pos_turns()
    if pos_after is None:
        raise SystemExit("设零后读不到位置")
    out_after = pos_after * DEG_PER_MOTOR_TURN
    print(f"设零后回读：{pos_after:.5f} turns = {out_after:.3f}° 输出")
    if abs(out_after) >= 0.05:
        print(f"[FAIL] 回读 {out_after:.3f}° 未到 0（要求 |q|<0.05°）；不写 EEPROM")
        m.disable()
        link.close()
        return 1

    # 3) 写限位 / 限流 / 限速
    lim = cfg.eeprom_limits_deg
    hi_t, lo_t = out_deg_to_turns(lim[1]), out_deg_to_turns(lim[0])
    ok = m.set_pos_limits_turns(lo_t, hi_t)
    print(f"写位置限位 {lim[0]}..{lim[1]}°（{lo_t:.3f}..{hi_t:.3f} turns）：{ok}")
    if cfg.required_current_limit_a is not None:
        okc = m.set_current_limit_a(cfg.required_current_limit_a)
        print(f"写限流 {cfg.required_current_limit_a} A：{okc}")
    else:
        print("限流：保持出厂值，不写")
    oks = m.set_speed_limit_out_deg_s(cfg.eeprom_speed_limit_out_deg_s)
    print(f"写速度限制 {cfg.eeprom_speed_limit_out_deg_s}°/s：{oks}")

    # 4) 存 EEPROM
    if not m.save():
        print("[FAIL] 存储参数（0x0D）无应答；尝试恢复原值")
        restore(m, old, cfg)
        m.disable()
        link.close()
        return 1
    time.sleep(0.3)

    # 5) 全部回读比对
    lim2 = m.pos_limits_turns()
    cur2 = m.current_limit_a()
    spd2 = m.speed_limit_rpm()
    pos2 = m.pos_turns()
    problems = []
    if lim2 is None or abs(lim2[0] - lo_t) > 0.01 or abs(lim2[1] - hi_t) > 0.01:
        problems.append(f"限位回读 {lim2} ≠ {lo_t:.3f}..{hi_t:.3f}")
    if cfg.required_current_limit_a is not None:
        if cur2 is None or abs(cur2 - cfg.required_current_limit_a) > 0.05:
            problems.append(f"限流回读 {cur2} ≠ {cfg.required_current_limit_a}")
    if spd2 is None or abs(spd2 - 360.0) > 1.0:
        problems.append(f"速度限制回读 {spd2} ≠ 360 RPM（60°/s）")
    if pos2 is None or abs(pos2 * DEG_PER_MOTOR_TURN) >= 0.05:
        problems.append(f"位置回读 {None if pos2 is None else pos2*DEG_PER_MOTOR_TURN:.3f}°")

    print(f"回读：限位={lim2} 限流={cur2} A 速度限制={spd2} RPM 位置={pos2}")
    if problems:
        print("[FAIL] 回读不符：" + "；".join(problems))
        print("恢复原值，不保留新参数")
        restore(m, old, cfg)
        m.disable()
        link.close()
        return 1

    today = _dt.date.today().isoformat()
    patch_joint(cfg.name, zero_date=today)
    print(f"[PASS] {cfg.name} 设零完成，限位/限流/限速已存并回读一致（{today}）")
    m.disable()
    print("已失能。下一步：方向核对 jog，然后 set-sign。")
    link.close()
    return 0


def restore(m, old, cfg):
    try:
        if old["limits"]:
            m.set_pos_limits_turns(*old["limits"])
        if old["current"] is not None and cfg.required_current_limit_a is not None:
            m.set_current_limit_a(old["current"])
        if old["speed"] is not None:
            m.set_speed_limit_out_deg_s(old["speed"] * DEG_PER_MOTOR_TURN / 60.0)
        m.save()
    except Exception as exc:  # noqa: BLE001
        print(f"[WARN] 恢复原值失败：{exc}")


def cmd_jog(args):
    link, cfg, m = connect_and_check(args)
    if m.power() != 1:
        if not m.enable():
            raise SystemExit("使能无应答")
        time.sleep(0.2)
    m.clear_error()
    p0 = m.pos_turns()
    if p0 is None:
        raise SystemExit("读不到位置")
    m.set_mode(MODE_PROFILE_POSITION)
    m.set_target_turns(p0)
    time.sleep(0.1)
    m.set_profile_out(max(0.5, min(5.0, abs(args.delta) / 2.0)),
                      max(0.2, abs(args.delta) / 2.0))
    p0 = m.pos_turns()
    # 电机的正方向：raw 增加（不乘 sign）
    target = p0 + args.delta / DEG_PER_MOTOR_TURN
    m.set_target_turns(target)
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < 6.0:
        time.sleep(0.2)
        last = m.pos_turns()
        if last is not None and abs(last - target) < 0.02:
            break
    e = m.error() or 0
    print(f"{cfg.name} 电机正方向走 {args.delta:+.1f}° 输出："
          f"{p0:.4f} → {last if last is None else round(last, 4)} turns，"
          f"实际 {(last-p0)*DEG_PER_MOTOR_TURN:+.2f}°，错误 0x{e:04X}")
    print("请用户看：这个方向对应 16.1 关节表里的正方向吗？")
    st = load_state()
    st.setdefault(cfg.name, {})["jog_from_turns"] = p0
    st[cfg.name]["jog_to_turns"] = last
    save_state(st)
    link.close()
    return 0


def cmd_return(args):
    link, cfg, m = connect_and_check(args)
    st = load_state().get(cfg.name, {})
    target = st.get("jog_from_turns")
    if target is None:
        raise SystemExit("没有 jog 前的位置记录")
    m.set_mode(MODE_PROFILE_POSITION)
    m.set_profile_out(3.0, 3.0)
    m.set_target_turns(target)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 6.0:
        time.sleep(0.2)
        p = m.pos_turns()
        if p is not None and abs(p - target) < 0.02:
            break
    print(f"{cfg.name} 回位到 {target:.4f} turns：{p}")
    m.disable()
    link.close()
    return 0


def cmd_set_sign(args):
    if args.sign not in (-1, 1):
        raise SystemExit("sign 只能是 +1 或 -1")
    patch_joint(args.joint, sign=int(args.sign), sign_checked=True)
    print(f"{args.joint} sign = {args.sign} 已写入 config/joints.json（sign_checked=true）")


def cmd_disable(args):
    link, cfg, m = connect_and_check(args)
    print("失能:", m.disable())
    link.close()
    return 0


def cmd_params(args):
    link, cfg, m = connect_and_check(args)
    if m.power() != 1:
        if not m.enable():
            raise SystemExit("使能无应答")
        time.sleep(0.15)
    print(f"{cfg.name} 模式={m.mode()} 限位={m.pos_limits_turns()} "
          f"限流={m.current_limit_a()} A 速度限制={m.speed_limit_rpm()} RPM "
          f"梯形={m.profile()} 位置={m.pos_turns()} turns")
    m.disable()
    link.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="16.3 单关节设零/方向核对")
    ap.add_argument("action", choices=["read", "prepare", "set-zero", "jog", "return",
                                       "set-sign", "disable", "params"])
    ap.add_argument("--joint", default="J6")
    ap.add_argument("--ip", default="192.168.1.30")
    ap.add_argument("--ecb-port", type=int, default=2000)
    ap.add_argument("--delta", type=float, default=5.0)
    ap.add_argument("--sign", type=int, default=1)
    args = ap.parse_args(argv)
    args.joint = args.joint.upper()
    if args.joint not in load_joint_configs():
        raise SystemExit(f"未知关节 {args.joint}")
    fn = {"read": cmd_read, "prepare": cmd_prepare, "set-zero": cmd_set_zero,
          "jog": cmd_jog, "return": cmd_return, "set-sign": cmd_set_sign,
          "disable": cmd_disable, "params": cmd_params}[args.action]
    return fn(args)


if __name__ == "__main__":
    sys.exit(main())
