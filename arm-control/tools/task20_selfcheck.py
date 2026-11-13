#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二十节 mock 自检（不连真机、不改 config）。

覆盖 20.0–20.9；20.10 真机两项不在本脚本内。
运行：cd arm-control && python3 tools/task20_selfcheck.py
"""
from __future__ import annotations

import random
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arm.calib import CalibSession
from arm.config import URDF, tcp_presets
from arm.executor import Executor
from arm.fence import Fence
from arm.joints import JointBus, load_joint_configs
from arm.kinematics import RobotModel
from arm.mock import MockJointBus
from arm.motion import Motion, MotionError
from arm.sca import DEG_PER_MOTOR_TURN, MODE_PROFILE_POSITION, R_MODE, R_POWER
from arm.transport import EcbTransport, Frame
from arm import config as cfgmod

# 自检的停机完成会写 park_state.json；用 mock 的结果不能留到真机
cfgmod.save_park_state = lambda _data: None

FAILS: list[str] = []


def check(name: str, cond: bool, detail: str = ""):
    print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  — {detail}" if detail else ""))
    if not cond:
        FAILS.append(name)


def mk_exec() -> Executor:
    e = Executor()
    e.connect("mock")
    return e


def close_exec(e: Executor):
    e._poll_stop.set()
    e._watch_stop.set()
    try:
        e.disconnect()
    except Exception:  # noqa: BLE001
        pass


def park_prereq():
    return {"gripper_empty": True, "workspace_clear": True, "shelf_ready": True}


# ---------------- 20.0 mock 故障注入 ----------------
b = MockJointBus()
b.set_faults(silent=["J4"], drop={"position": 0.3}, delay={"J5": 1},
             error_bits={"J6": 0x0004}, limits={"J2": 3.0})
d = b.read_all(full=True)
check("20.0 故障开关生效", "J4" in b.silent and b.drop_prob and b.error_bits.get("J6") == 4,
      str(b.mock_faults()))
check("20.0 silent 关节读不到位置", d["J4"].out_deg is None)
check("20.0 报警位可注入", b.read_all()["J6"].error == 4)
b.clear_faults()

# ---------------- 20.1 中止保持 ----------------
e = mk_exec()
e.jog("J2", 30, preview=False)
time.sleep(0.3)
e.bus.set_faults(silent=["J4"])
t0 = time.time()
while time.time() - t0 < 8 and e.motion.status.get("phase") not in ("aborted", "error"):
    time.sleep(0.03)
check("20.1 单颗失联：中止并保持", e.motion.status.get("phase") == "aborted"
      and e.bus.enabled is True, e.motion.status.get("reason", "")[:90])
check("20.1 单颗失联：其余仍使能", e.bus.enabled and not e.bus.off)
close_exec(e)

e = mk_exec()
e.jog("J2", 30, preview=False)
time.sleep(0.25)
e.bus.set_faults(silent=list(e.bus.order))
t0 = time.time()
while time.time() - t0 < 8 and e.motion.status.get("phase") not in ("aborted", "error"):
    time.sleep(0.03)
check("20.1 六颗全失联：才全部失能", e.motion.status.get("phase") == "aborted"
      and e.bus.enabled is False, e.motion.status.get("reason", "")[:110])
close_exec(e)

# ---------------- 20.2 read_many 补发 / verify_ready ----------------
t = EcbTransport("127.0.0.1", 9)
sent, rounds = [], [0]


def fake_send(raw):
    sent.append((raw[1], raw[2]))


def fake_collect(_sec):
    rounds[0] += 1
    if rounds[0] == 1:
        return [Frame(a, 0x2B, b"\x01", b"") for a in (1, 2)]
    if rounds[0] == 2:
        return [Frame(a, 0x2B, b"\x01", b"") for a in (3, 4)]
    return []


t._send = fake_send
t._collect = fake_collect
r = t.read_many([(a, 0x2B, 1) for a in (1, 2, 3, 4)], timeout=0.03, retries=2)
check("20.2 read_many 部分回包后仍补发收齐", all((a, 0x2B) in r for a in (1, 2, 3, 4)),
      f"{len(r)}/4，{rounds[0]} 轮")
t.close()

names = ["J1", "J2", "J3", "J4", "J5", "J6"]
motors = {n: types.SimpleNamespace(mid=i + 1) for i, n in enumerate(names)}


class FakeLink:
    def __init__(self, p=0.0, bad=None):
        self.p, self.bad = p, bad or {}

    def read_many(self, items, timeout=0.05, retries=2):
        out = {}
        for addr, cmd, _n in items:
            if addr in self.bad.get("never", set()) or random.random() < self.p:
                continue
            val = 1 if cmd == R_POWER else 6
            val = self.bad.get("wrong", {}).get((addr, cmd), val)
            out[(addr, cmd)] = bytes([val])
        return out

    def read(self, addr, cmd, _n, **_kw):
        if addr in self.bad.get("never", set()) or random.random() < self.p:
            return None
        val = 1 if cmd == R_POWER else 6
        val = self.bad.get("wrong", {}).get((addr, cmd), val)
        return bytes([val])


obj = types.SimpleNamespace(order=names, motors=motors, link=FakeLink())
check("20.2 verify_ready 正常", JointBus.verify_ready(obj) == [])
obj.link = FakeLink(bad={"never": {4}, "wrong": {(5, R_MODE): 1, (6, R_POWER): 0}})
bad = JointBus.verify_ready(obj)
check("20.2 verify_ready 区分读不到/值不对",
      any("读不到" in x[1] for x in bad) and any("读到了但" in x[1] for x in bad),
      str(bad))
random.seed(1)
obj.link = FakeLink(p=0.3)
false_bad = sum(1 for _ in range(200) if JointBus.verify_ready(obj))
check("20.2 30% 丢包 200 次起步无误判", false_bad == 0,
      f"误判 {false_bad}/200；理论上限 200×12×0.3^13≈4e-4")

# ---------------- 20.3 停机可停 + 互斥 ----------------
e = mk_exec()
e.bus.set_faults(currents={"J2": 2.0, "J3": 1.0})
e.park(preview=False, prereq=park_prereq())
t0 = time.time()
while time.time() - t0 < 15:
    if e.park_step.get("phase") == "descending" and (e.park_step.get("step") or 0) >= 2:
        break
    time.sleep(0.02)
check("20.3 停机下降到第 2 步", e.park_step.get("phase") == "descending", str(e.park_step))
rejected = []
for fn, args in (("jog", ("J2", 5)), ("home", ()), ("park", (False,)),
                 ("teach_park", ()), ("set_end_effector", ("gripper",)),
                 ("write_limits_all", ())):
    try:
        if fn == "park":
            e.park(False, prereq=park_prereq())
        else:
            getattr(e, fn)(*args)
    except Exception as exc:  # noqa: BLE001
        rejected.append(fn)
check("20.3 停机期间其它运动被拒", len(rejected) == 6, str(rejected))
e.stop_all("自检按停")
t1 = time.time()
while time.time() - t1 < 6 and e.park_active:
    time.sleep(0.05)
check("20.3 停机中按停：停在原地、不失能",
      e.park_step.get("phase") == "aborted" and e.bus.enabled is True
      and not e.bus.off, e.msg[:90])
close_exec(e)

# 20.3 完成后 startup_done 置假（20.9.3）
e = mk_exec()
calls = {"n": 0}


def fake_sample(names_, n=None, stop_event=None):
    calls["n"] += 1
    return {"J2": 2.0 if calls["n"] == 1 else 0.5, "J3": 1.0}


e._sample_currents = fake_sample
e.park(False, prereq=park_prereq())
t0 = time.time()
while time.time() - t0 < 30 and e.park_active:
    time.sleep(0.1)
check("20.9.3 停机完成 startup_done=False", e.park_step.get("phase") == "parked"
      and e.bus.startup_done is False, e.park_step.get("phase", ""))
close_exec(e)

# ---------------- 20.4 心跳 ----------------
e = mk_exec()
e.start_watchdog()
e.acquire("a", e.config_version)
e.jog("J2", 30, preview=False)
t0 = time.time()
while time.time() - t0 < 1.2:
    e.touch("a")
    time.sleep(0.05)
check("20.4 touch/SSE 不续心跳，运动被中止",
      e.motion.status.get("phase") == "aborted"
      and "心跳" in (e.motion.status.get("reason") or ""),
      (e.motion.status.get("reason") or "")[:80])
e._watch_stop.set()
close_exec(e)

# ---------------- 20.5 限流回读 ----------------
e = mk_exec()
check("20.5 J2 3.0A → 5.73 N·m", abs(e.capacities()["J2"] - 5.73) < 0.01,
      str(e.capacities()["J2"]))
e.bus.current_limit_readback["J4"] = 1.0
e.kt = {"J4": {"kt_eff": 0.5}}
check("20.5 J4 用回读限流×kt_eff", abs(e.capacities()["J4"] - 0.5) < 1e-9,
      str(e.capacities()["J4"]))
e.bus.current_limit_readback["J2"] = None
check("20.5 J2 未回读按 3.0A 保守", e.current_limits()["J2"] == 3.0
      and e.limit_used()["J2"]["source"].startswith("保守"),
      str(e.limit_used()["J2"]))
close_exec(e)

# ---------------- 20.6 配置热重载（方案 B） ----------------
e = mk_exec()
old = e.config_version
cfgmod.config_version = lambda: old + "-changed"
e.refresh_config_version()
check("20.6 连接中配置变 → stale", e.config_stale is True, e.config_stale_msg)
try:
    e._ready_for_motion()
    check("20.6 运动被拒", False)
except MotionError:
    check("20.6 运动被拒", True)
close_exec(e)
cfgmod.config_version = lambda: old

# ---------------- 20.7 停不洗 aborted ----------------
e = mk_exec()
e.jog("J2", 30, preview=False)
e.stop_all("10ms")
t0 = time.time()
while time.time() - t0 < 5 and e.motion.status.get("phase") not in ("aborted", "error"):
    time.sleep(0.02)
e.stop_all("再次停")
check("20.7 再次按停不洗 aborted", e.motion.status.get("phase") == "aborted")
try:
    e.jog("J2", 5)
    check("20.7 中止后仍要求恢复", False)
except MotionError:
    check("20.7 中止后仍要求恢复", True)
close_exec(e)

# ---------------- 20.8 预演实际下降 ----------------
e = mk_exec()
p = e.park(preview=True, prereq=park_prereq())
q_expected = e._park_q_at_z(130.5, 90.0)
check("20.8 预演画实际下降终点", [round(v, 3) for v in p["preview"]["q"]] ==
      [round(v, 3) for v in q_expected], str(p["preview"]["q"]))
check("20.8 网页说明教的点只决定左右侧",
      "只决定左右侧" in p["descent"]["note"] and
      "只决定左右侧" in e.state()["park"]["descent"]["note"])
path = p["preview"]["park"]["descent_path"]
check("20.8 预演路径是实际下降轨迹", len(path) > 2 and path[0][2] > path[-1][2],
      f"{len(path)} 点，z {path[0][2]:.1f}→{path[-1][2]:.1f}")
close_exec(e)

# ---------------- 20.9.1 切模式顺序 ----------------
class FakeMotor:
    def __init__(self):
        self.calls, self.pos = [], 0.5

    def set_mode(self, m):
        self.calls.append(("mode", m))

    def set_profile_out(self, *a):
        self.calls.append(("profile",))

    def pos_turns(self, **kw):
        return self.pos

    def set_target_turns(self, t):
        self.calls.append(("target", round(t, 4)))
        self.pos = t

    def enable(self):
        self.calls.append(("enable",))
        return True

    def error(self):
        return 0


fm = FakeMotor()
jb = JointBus.__new__(JointBus)
jb.cfg, jb.order, jb._targets, jb.motors = load_joint_configs(), names, {}, {"J2": fm}
JointBus.enable_one_hold(jb, "J2", 0.5)
mode_idx = [i for i, x in enumerate(fm.calls) if x == ("mode", MODE_PROFILE_POSITION)][0]
target_before = [i for i, x in enumerate(fm.calls) if x[0] == "target" and i < mode_idx]
check("20.9.1 enable_one_hold 先写目标再切位置模式", bool(target_before),
      str(fm.calls))
c = CalibSession(types.SimpleNamespace())
c.joint, c.transport, c.stage, c.power = "J2", object(), "zeroed", 1
c.motor = FakeMotor()
c.jog(5.0)
mode_idx = [i for i, x in enumerate(c.motor.calls)
            if x == ("mode", MODE_PROFILE_POSITION)][0]
check("20.9.1 calib.jog 先写目标再切位置模式",
      any(x[0] == "target" for x in c.motor.calls[:mode_idx]), str(c.motor.calls))

# ---------------- 20.9.2 逐关节跟踪 ----------------
class Stuck(MockJointBus):
    def read_all(self, full=False):
        d = super().read_all(full)
        d["J4"].out_deg = 10.0
        return d


bus = Stuck()
robot = RobotModel(URDF, tcp_presets())
jc = load_joint_configs()
lim = ([jc[n].safe_limits_deg[0] for n in bus.order],
       [jc[n].safe_limits_deg[1] for n in bus.order])
m = Motion(bus, robot, Fence(robot), lim)
m.preset = "flange"
segs = m._plan_move([0] * 6, [0, 30, 0, 0, 0, 0], "test")
t0 = time.time()
try:
    m.execute(segs, preview=False)
    check("20.9.2 小关节卡住逐关节发现", False)
except MotionError as exc:
    check("20.9.2 小关节卡住逐关节发现",
          "J4" in str(exc) and time.time() - t0 < 1.5, str(exc)[:90])

# ---------------- 20.9.4 换末端清计划 ----------------
e = mk_exec()
e.pending = [{"x": 1}]
e.preview = {"q": [0] * 6}
e.set_end_effector("gripper")
check("20.9.4 换末端清空待执行计划/预演", e.pending is None and e.preview is None)
close_exec(e)

# ---------------- 20.9.5 控制权 token ----------------
e = mk_exec()
a = e.acquire("a", e.config_version)
try:
    e.acquire("a", e.config_version)
    no_token_ok = True
except Exception:  # noqa: BLE001
    no_token_ok = False
check("20.9.5 同 client_id 无 token 不能再拿控制权", not no_token_ok)
check("20.9.5 带原 token 可以重复 acquire",
      bool(e.acquire("a", e.config_version, a["token"]).get("token")))
close_exec(e)

print(f"\n==== {len(FAILS)} FAIL ====" if FAILS else "\n==== ALL PASS ====")
raise SystemExit(1 if FAILS else 0)
