#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16.3 辅助：保持一颗关节在 homing 模式，持续把位置写进 runs/zero_live.json。

用户在旁边用手摆关节时，这个进程保持 UDP/使能状态并记录实时读数。
停止：Ctrl-C，或创建 runs/zero_stop 文件。

    python3 tools/zero_hold.py --joint J6 --duration 300
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from arm.joints import load_joint_configs  # noqa: E402
from arm.sca import DEG_PER_MOTOR_TURN, MODE_HOMING, ScaMotor, describe_error  # noqa: E402
from arm.transport import EcbTransport  # noqa: E402

LIVE = ROOT / "runs" / "zero_live.json"
STOP = ROOT / "runs" / "zero_stop"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="J6")
    ap.add_argument("--duration", type=float, default=300)
    ap.add_argument("--ip", default="192.168.1.30")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args(argv)
    name = args.joint.upper()
    cfg = load_joint_configs()[name]

    t = EcbTransport(args.ip, args.port)
    if not t.hello():
        raise SystemExit("ECB 没有握手应答")
    disc = t.scan(2.0)
    sn = disc.get(cfg.id)
    if sn is None or f"{sn:08X}"[-4:] != cfg.sn4:
        raise SystemExit(f"{name} ID {cfg.id} SN 不匹配：{sn}")
    m = ScaMotor(t, cfg.id)
    if not m.enable():
        raise SystemExit("使能失败")
    time.sleep(0.2)
    p = m.pos_turns()
    if p is None:
        raise SystemExit("读不到位置")
    m.set_target_turns(p)
    if not m.set_mode(MODE_HOMING):
        print("[WARN] 切 homing 模式未确认")
    if STOP.exists():
        STOP.unlink()
    print(f"{name} homing 模式保持中；实时位置写入 {LIVE}", flush=True)
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < args.duration:
        if STOP.exists():
            print("收到停止文件", flush=True)
            break
        p = m.pos_turns()
        power = m.power()
        err = m.error() if power == 1 else None
        d = {
            "joint": name, "time": time.strftime("%H:%M:%S"),
            "power": power,
            "turns": None if p is None else round(p, 6),
            "out_deg": None if p is None else round(p * DEG_PER_MOTOR_TURN, 3),
            "error": err,
            "error_text": describe_error(err or 0),
            "holding_s": round(time.monotonic() - t0, 1),
        }
        LIVE.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")
        if p is not None and (last is None or abs(p - last) > 0.0005):
            print(f"{d['time']}  {d['out_deg']:+.3f}°  ({d['turns']:+.5f} R)  电源={power}", flush=True)
            last = p
        time.sleep(0.2)
    m.disable()
    t.close()
    print("hold 结束，已失能", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
