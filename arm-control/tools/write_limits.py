#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只重写一颗关节的 EEPROM 限位/限流/速度限制（不动零位）。

P03 把 J2 限位从 ±110° 改成 ±95°（软件 ±90°），用这个工具更新电机里的值。

    python3 tools/write_limits.py --joint J2 --yes
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from arm.joints import load_joint_configs  # noqa: E402
from arm.sca import (DEG_PER_MOTOR_TURN, MODE_CURRENT, ScaMotor,
                     out_deg_to_turns)  # noqa: E402
from arm.transport import EcbTransport  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="J2")
    ap.add_argument("--ip", default="192.168.1.30")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args(argv)
    if not args.yes:
        print("这会写 EEPROM；确认臂已支撑后加 --yes")
        return 2
    name = args.joint.upper()
    cfg = load_joint_configs()[name]
    t = EcbTransport(args.ip, args.port)
    if not t.hello():
        raise SystemExit("ECB 没有握手应答")
    disc = t.scan(2.0)
    sn = disc.get(cfg.id)
    if sn is None or f"{sn:08X}"[-4:] != cfg.sn4:
        raise SystemExit(f"{name} ID/SN 不符：{sn}")
    m = ScaMotor(t, cfg.id)
    was_on = m.power() == 1
    if not was_on:
        try:
            m.set_mode(MODE_CURRENT)
            m.set_current_a(0.0)
        except Exception:  # noqa: BLE001
            pass
        if not m.enable():
            raise SystemExit("使能无应答")
        m.set_mode(MODE_CURRENT)
        m.set_current_a(0.0)
        time.sleep(0.2)
    # 已经在位置模式保持时不要切电流模式，避免关节失力掉下来；只读写参数
    pos = m.pos_turns()
    print(f"{name} 当前位置 {pos} turns = {None if pos is None else round(pos*DEG_PER_MOTOR_TURN,3)}°")
    print("原参数：限位", m.pos_limits_turns(), "限流", m.current_limit_a(),
          "速度限制", m.speed_limit_rpm())
    lo, hi = cfg.eeprom_limits_deg
    lo_t, hi_t = out_deg_to_turns(lo), out_deg_to_turns(hi)
    m.set_pos_limits_turns(lo_t, hi_t)
    if cfg.required_current_limit_a is not None:
        m.set_current_limit_a(cfg.required_current_limit_a)
    m.set_speed_limit_out_deg_s(cfg.eeprom_speed_limit_out_deg_s)
    m.save()
    time.sleep(0.4)
    lim = m.pos_limits_turns()
    cur = m.current_limit_a()
    spd = m.speed_limit_rpm()
    pos = m.pos_turns()
    print("回读：限位", lim, "限流", cur, "速度限制", spd, "位置", pos)
    ok = (lim is not None and abs(lim[0] - lo_t) < 0.01 and abs(lim[1] - hi_t) < 0.01
          and (spd is not None and abs(spd - 360.0) < 1.0))
    if cfg.required_current_limit_a is not None:
        ok = ok and cur is not None and abs(cur - cfg.required_current_limit_a) < 0.05
    if was_on:
        # 恢复位置保持，不要失能
        try:
            m.set_target_turns(m.pos_turns() or 0.0)
            m.set_mode(6)
            m.set_profile_out(cfg.motion_speed_deg_s,
                              cfg.motion_speed_deg_s / max(cfg.motion_accel_time_s, 0.05))
            m.set_target_turns(m.pos_turns() or 0.0)
        except Exception:  # noqa: BLE001
            pass
    else:
        m.disable()
    t.close()
    print("[PASS]" if ok else "[FAIL]")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
