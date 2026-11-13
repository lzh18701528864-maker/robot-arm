#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 17.1 夹爪定零 / 读位置 / 方向 / h 核对（B4AE，ID 7）。

- 失能读不到位置时，用零扭矩使能（电流模式 0 A）读；
- 读零要在贴软垫之前；第十一节在 B4AE 上的 Homing/设零试验要先做完；
- 装好丝杠/滑块以后，这个工具**只做零扭矩读**，不发位置/速度/电流指令。

用法：
    python3 tools/gripper_zero.py read
    python3 tools/gripper_zero.py zero          # 当前 h=0，存为零点
    python3 tools/gripper_zero.py sign --sign -1
    python3 tools/gripper_zero.py status        # 按零点算 h
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
from arm.sca import (DEG_PER_MOTOR_TURN, MODE_CURRENT, ScaMotor, describe_error)  # noqa: E402
from arm.transport import EcbTransport  # noqa: E402

CFG = ROOT / "config" / "gripper.json"


def load_cfg():
    return json.loads(CFG.read_text(encoding="utf-8"))


def save_cfg(d):
    CFG.write_text(json.dumps(d, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def connect():
    c = load_cfg()
    t = EcbTransport()
    if not t.hello():
        raise SystemExit("ECB 没有握手应答")
    disc = t.scan(2.0)
    sn = disc.get(c["id"])
    if sn is None:
        t.close()
        raise SystemExit(f"ECB 没发现 ID {c['id']}（B4AE）")
    if f"{sn:08X}"[-4:] != c["sn4"]:
        t.close()
        raise SystemExit(f"ID {c['id']} 的 SN 是 {f'{sn:08X}'[-4:]}，不是 {c['sn4']}")
    return t, ScaMotor(t, c["id"]), c


def read_pos(m):
    p = m.pos_turns()
    if p is not None:
        return p, "失能读到"
    # 零扭矩使能读
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
    p = m.pos_turns()
    return p, "零扭矩使能读到"


def h_mm(turns, c):
    z = c.get("zero_turns")
    if z is None:
        return None
    out_deg = (turns - z) * DEG_PER_MOTOR_TURN
    return out_deg / float(c["deg_per_mm"])


def cmd_read(args):
    t, m, c = connect()
    p, how = read_pos(m)
    print(f"{how}：{p} turns = {None if p is None else round(p*DEG_PER_MOTOR_TURN,3)}° 输出")
    print(f"错误 0x{(m.error() or 0):04X} {describe_error(m.error() or 0)}")
    print(f"零点 {c.get('zero_turns')}；按零点 h = {None if p is None else h_mm(p,c)} mm")
    m.disable()
    t.close()
    return 0


def cmd_zero(args):
    t, m, c = connect()
    p, how = read_pos(m)
    if p is None:
        m.disable(); t.close()
        raise SystemExit("读不到位置")
    c["zero_turns"] = round(p, 6)
    c["zero_date"] = _dt.date.today().isoformat()
    save_cfg(c)
    print(f"{how}：零点存为 {p:.6f} turns（h=0，{c['zero_date']}）")
    print("注意：贴软垫之前读；以后再做 Homing/清零/改位置偏置，零点作废要重读。")
    m.disable()
    t.close()
    return 0


def cmd_sign(args):
    if args.sign not in (-1, 1):
        raise SystemExit("sign 只能是 +1 或 -1")
    c = load_cfg()
    c["sign"] = int(args.sign)
    c["sign_checked"] = True
    save_cfg(c)
    print(f"夹爪 sign = {args.sign} 已写入 config/gripper.json")
    return 0


def cmd_status(args):
    t, m, c = connect()
    p, how = read_pos(m)
    h = None if p is None else h_mm(p, c)
    print(f"{how}：{p} turns，h = {None if h is None else round(h,2)} mm")
    if h is not None:
        lo, hi = c["h_soft_limits_mm"]
        print(f"软限位 h {lo}…{hi}；机械限位块 {c['h_mech_limit_mm']}；"
              f"{'在范围内' if lo <= h <= hi else '超出软限位'}")
    m.disable()
    t.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["read", "zero", "sign", "status"])
    ap.add_argument("--sign", type=int, default=1)
    args = ap.parse_args(argv)
    return {"read": cmd_read, "zero": cmd_zero, "sign": cmd_sign,
            "status": cmd_status}[args.action](args)


if __name__ == "__main__":
    sys.exit(main())
