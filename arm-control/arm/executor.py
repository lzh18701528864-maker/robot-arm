#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段一 A 执行器（独立后台程序）。

职责：传输层 + 关节层 + 运动层 + 限位/围栏检查；通过网络接口收命令、推状态。
网页只是它的一个客户端，不直接碰 ECB。

运行：
    python3 -m arm.executor --mock          # 模拟臂
    python3 -m arm.executor                 # 真机，网页里再点连接/使能
    python3 -m arm.executor --host 0.0.0.0 --port 8770   # 给局域网客户端

接口（HTTP/JSON）：
    GET  /api/config                 配置版本、接口信息
    GET  /api/state                  当前状态（轮询兜底）
    GET  /api/events?client_id=...   状态推送（SSE，默认 20 Hz）
    POST /api/acquire                拿控制权 {client_id, config_version}
    POST /api/heartbeat              心跳 {token}
    POST /api/release                释放控制权 {token}
    POST /api/connect                连接 ECB / 模拟
    POST /api/enable                 启动序列（J6→J1 逐个使能）
    POST /api/click                  虚拟点击（按模式预演或直接执行）
    POST /api/plan_click             只规划
    POST /api/execute                执行待执行计划
    POST /api/stop                   中止（原地保持）
    POST /api/jog                    点动
    POST /api/home                   回零位
    POST /api/disable_all            全部失能（二次确认）
    POST /api/move_joints            关节角数组
    POST /api/move_tcp               TCP 位姿
    POST /api/move_through           路径（每段可带下探标记）
    POST /api/tcp /api/yaw /api/exec_mode
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import mimetypes
import secrets
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from . import config as cfgmod
from . import gravity
from .calib import CalibSession
from .fence import Fence
from .joints import JointBus, StartupRefused, load_joint_configs
from .kinematics import RobotModel
from .mock import MockJointBus, VirtualJointBus
from .motion import Motion, MotionError
from .sca import MODE_CURRENT, MODE_PROFILE_POSITION, describe_error
from .recorder import Recorder

MIME = {".js": "text/javascript", ".mjs": "text/javascript",
        ".css": "text/css", ".html": "text/html; charset=utf-8",
        ".json": "application/json", ".svg": "image/svg+xml",
        ".png": "image/png", ".ico": "image/x-icon"}


class AuthError(RuntimeError):
    pass


class Executor:
    def __init__(self, cfg: dict | None = None):
        self.cfg = dict(cfg or cfgmod.load_executor())
        self.bind_host = self.cfg.get("bind_host", "127.0.0.1")
        self.bind_port = int(self.cfg.get("bind_port", 8770))
        self.ecb_ip = self.cfg.get("ecb_ip", "192.168.1.30")
        self.ecb_port = int(self.cfg.get("ecb_udp_port", 2000))
        self.web_dir = self._path(self.cfg.get("web_dir", "web"))
        self.runs_dir = self._path(self.cfg.get("runs_dir", "runs"))
        self.models_dir = self._path(self.cfg.get("models_dir", "models"))
        self.serve_web = bool(self.cfg.get("serve_web", True))
        self.push_hz = max(10.0, float(self.cfg.get("state_push_hz", 20)))
        self.hb_timeout = float(self.cfg.get("heartbeat_timeout_s", 0.5))
        self.control_stale = float(self.cfg.get("control_stale_s", 3.0))
        self.exec_mode = self.cfg.get("default_exec_mode", "preview")
        self.config_version = cfgmod.config_version()
        self.config_info = cfgmod.config_versions()
        self.web_version = self._read_web_version()

        self.robot = RobotModel(cfgmod.URDF, cfgmod.tcp_presets())
        self.joint_cfg = load_joint_configs()
        self.limits = (
            [self.joint_cfg[n].safe_limits_deg[0] for n in self.robot.joint_names],
            [self.joint_cfg[n].safe_limits_deg[1] for n in self.robot.joint_names],
        )
        self.fence = Fence(self.robot, workspace={
            "x": [-200.0, 600.0], "y": [-400.0, 400.0], "enabled": False,
        })

        self.bus = None
        self.motion: Motion | None = None
        # 离线/未使能时的零位虚拟模型（只读预演）
        self.preview_bus = VirtualJointBus(self.joint_cfg)
        self.preview_motion = Motion(self.preview_bus, self.robot, self.fence,
                                     self.limits, recorder=None)
        self.recorder: Recorder | None = None
        self.mode = None
        self.ip = ""
        self.msg = "未连接"
        self.connected = False
        self.tcp_preset = cfgmod.load_tcp().get("default", "probe")
        self.yaw_deg = 0.0
        self.preview = None
        self.pending = None
        self._home_preview = None
        self.cache = {}
        self._lock = threading.RLock()
        self._poll_stop = threading.Event()
        self._poll_thread = None
        self._exec_thread = None
        self._last_full = 0.0
        self._last_states = {}

        # 控制权 / 心跳
        self.owner: str | None = None
        self.token: str | None = None
        self.owner_since = 0.0
        self.last_hb = 0.0
        self.clients: dict[str, float] = {}
        self._watch_stop = threading.Event()
        self._watch_thread = None
        self._calib_active = False
        self.calib = CalibSession(self)
        self.drag_active = False
        self.measurements: list[dict] = []
        self._measure_path = self.runs_dir / "measure.csv"
        self._measure_header = False
        self._web_state_lock = threading.Lock()

        # 18.1/18.2/18.3：搁架、末端配置、力矩预检、停机状态
        self.park_cfg = cfgmod.load_park()
        self.park_state = cfgmod.load_park_state()
        self.end_effector = "flange"
        self.payload_kg = 0.0
        self.kt = gravity.load_kt()
        self.kt_rows: list[dict] = []
        self.hold = {"since": None, "ratio": 0.0, "warn": ""}
        self._park_thread = None
        self.park_step = {"phase": "idle", "reason": "", "z": None, "step": 0,
                          "I0": None, "I2": None, "I1": None, "contact_z": None}
        self.park_block: dict | None = None
        self.j2_bracket = str(getattr(self.joint_cfg.get("J2"), "j2_bracket", "PLA"))
        self._sync_motion_params()

    # ---------------- 路径/配置 ----------------
    def _read_web_version(self) -> str:
        try:
            return str(json.loads((self.web_dir / "version.json").read_text(encoding="utf-8"))
                       .get("version", ""))
        except Exception:  # noqa: BLE001
            return ""

    def _path(self, p: str) -> Path:
        q = Path(p)
        return q if q.is_absolute() else (cfgmod.ROOT / q)

    def refresh_config_version(self):
        self.config_version = cfgmod.config_version()
        self.config_info = cfgmod.config_versions()
        self.web_version = self._read_web_version()
        self.joint_cfg = load_joint_configs()

    def use_actual(self) -> bool:
        return bool(self.connected and self.motion is not None
                    and getattr(self.bus, "startup_done", False))

    def planning_motion(self):
        return self.motion if self.use_actual() else self.preview_motion

    def start_poll(self):
        if self._poll_thread and self._poll_thread.is_alive():
            return
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def config_payload(self) -> dict:
        return {
            "config_version": self.config_version,
            "web_version": self._read_web_version(),
            **self.config_info,
            "exec_mode": self.exec_mode,
            "default_exec_mode": self.cfg.get("default_exec_mode", "preview"),
            "state_push_hz": self.push_hz,
            "heartbeat_hz": float(self.cfg.get("heartbeat_hz", 5)),
            "heartbeat_timeout_s": self.hb_timeout,
            "control_stale_s": self.control_stale,
            "joint_names": self.robot.joint_names,
            "expected_count": int(cfgmod.load_joints().get("expected_count", 6)),
            "gripper_installed": bool(cfgmod.load_joints().get("gripper_installed", False)),
            "limits": {"lo": self.limits[0], "hi": self.limits[1]},
            "tcp_presets": self.robot.tcp_presets,
            "end_effector": self.end_effector,
            "end_effectors": list(gravity.END_EFFECTORS),
            "payload_kg": self.payload_kg,
            "park": self.park_config(),
            "ecb": {"ip": self.ecb_ip, "port": self.ecb_port},
            "bind": {"host": self.bind_host, "port": self.bind_port},
        }

    # ---------------- 控制权 ----------------
    def touch(self, client_id: str | None):
        if not client_id:
            return
        self.clients[client_id] = time.monotonic()
        if self.owner == client_id:
            self.last_hb = time.monotonic()

    def acquire(self, client_id: str, version: str) -> dict:
        if not client_id:
            raise AuthError("缺 client_id")
        if version != self.config_version:
            raise AuthError(f"配置版本不一致：客户端 {version}，执行器 {self.config_version}；"
                            "拒绝一切运动命令，请刷新/重新拉配置")
        now = time.monotonic()
        if self.owner and self.owner != client_id:
            age = now - max(self.last_hb, self.clients.get(self.owner, 0.0))
            if age < self.control_stale:
                raise AuthError(f"控制权在 {self.owner} 手里（{age:.1f}s 前还有心跳）")
        if self.owner == client_id and self.token:
            # 同一个客户端重复 acquire（页面轮询和初始化撞车）：返回原 token，
            # 不要换新 token，否则前一个请求会突然失效
            self.last_hb = now
            self.clients[client_id] = now
            return {"token": self.token, "client_id": client_id,
                    "config_version": self.config_version, "exec_mode": self.exec_mode}
        self.owner = client_id
        self.token = secrets.token_hex(12)
        self.owner_since = now
        self.last_hb = now
        self.clients[client_id] = now
        self.msg = f"{client_id} 取得控制权"
        return {"token": self.token, "client_id": client_id,
                "config_version": self.config_version, "exec_mode": self.exec_mode}

    def require_token(self, body: dict) -> str:
        token = str(body.get("token") or "")
        if not self.owner or not self.token or token != self.token:
            raise AuthError("没有控制权（另一个客户端在控制，或控制权已超时）")
        self.last_hb = time.monotonic()
        self.clients[self.owner] = self.last_hb
        return token

    def require_control(self, body: dict) -> str:
        token = str(body.get("token") or "")
        if not self.owner or not self.token or token != self.token:
            raise AuthError("没有控制权（另一个客户端在控制，或控制权已超时）")
        version = body.get("config_version")
        if version is not None and version != self.config_version:
            raise AuthError("配置版本不一致，拒绝运动命令")
        self.last_hb = time.monotonic()
        self.clients[self.owner] = self.last_hb
        return token

    def heartbeat(self, token: str) -> dict:
        if not self.token or token != self.token:
            raise AuthError("token 无效或控制权已释放")
        self.last_hb = time.monotonic()
        self.clients[self.owner] = self.last_hb
        return {"ok": True, "owner": self.owner}

    def release(self, token: str) -> dict:
        if self.token and token == self.token:
            if self.motion and self.motion.status.get("busy"):
                self.motion.stop("控制权释放")
            self.owner = None
            self.token = None
            self.msg = "控制权已释放"
        return {"ok": True}

    def _watchdog(self):
        while not self._watch_stop.is_set():
            try:
                now = time.monotonic()
                if self.calib.active:
                    # 校准会话：没有控制方、或心跳断了 >0.5 s，就结束并失能。
                    # 正在执行长命令（设零/点动等）时不打断，操作自己会检查。
                    hb = self.last_hb if self.owner else 0.0
                    if (not self.calib.busy) and ((not self.owner) or now - hb > self.hb_timeout):
                        age = (now - hb) if self.owner else 999.0
                        reason = (f"控制方心跳断了 {age:.2f}s"
                                  f"（>{self.hb_timeout}s），校准会话已结束并失能")
                        self.calib.abort(reason=reason)
                        self.msg = reason
                elif self.drag_active and (
                        (not self.owner) or now - max(self.last_hb, 0.0) > self.hb_timeout):
                    self.drag_stop()
                    self.msg = "拖动时控制方心跳断了，已切回位置保持"
                elif self.owner:
                    hb = max(self.last_hb, self.clients.get(self.owner, 0.0))
                    busy = bool(self.motion and self.motion.status.get("busy"))
                    if busy and now - hb > self.hb_timeout:
                        reason = (f"控制方心跳断了 {now-hb:.2f}s"
                                  f"（>{self.hb_timeout}s），已按中止处理：原地保持")
                        self.msg = reason
                        self.motion.stop(reason)
                    elif not busy and now - hb > self.control_stale:
                        self.owner = None
                        self.token = None
                        self.msg = "控制方心跳超时，控制权已释放"
            except Exception:  # noqa: BLE001
                traceback.print_exc()
            time.sleep(0.05)

    # ---------------- 连接 ----------------
    def connect(self, mode: str, ip: str = "", port: int = 0,
                debug: bool = False) -> dict:
        if self.calib.active:
            raise MotionError("校准会话进行中，先结束校准")
        self.disconnect()
        self.refresh_config_version()
        ip = ip or self.ecb_ip
        port = int(port or self.ecb_port)
        if mode == "mock":
            self.bus = MockJointBus()
            self.mode = "mock"
            self.ip = "mock"
            rep = self.bus.preflight()
            self.msg = "模拟模式（6 轴）"
            self.connected = True
        else:
            self.bus = JointBus(ip, port, self.joint_cfg, debug=debug)
            self.mode = "hw"
            self.ip = f"{ip}:{port}"
            rep = self.bus.preflight()
            self.connected = bool(rep.get("ok"))
            if self.connected:
                self.msg = "ECB 已连接，ID↔SN 一致；等待使能"
            else:
                self.msg = "预检未通过：" + "；".join(rep.get("mismatch", []))
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.recorder = Recorder(self.runs_dir / f"{stamp}_run.csv",
                                 note=f"connect {self.mode} {self.ip}")
        self.motion = Motion(self.bus, self.robot, self.fence, self.limits,
                             recorder=self.recorder)
        self.motion.preset = self.tcp_preset
        self.motion.yaw_deg = self.yaw_deg
        self.preview_motion.preset = self.tcp_preset
        self.preview_motion.yaw_deg = self.yaw_deg
        self._sync_motion_params()
        self.start_poll()
        return {"ok": self.connected, "mode": self.mode, "report": rep,
                "msg": self.msg, "config_version": self.config_version}

    def disconnect(self):
        if self.bus:
            try:
                if getattr(self.bus, "startup_done", False):
                    self.bus.hold_all()
            except Exception:  # noqa: BLE001
                pass
            try:
                self.bus.close()
            except Exception:  # noqa: BLE001
                pass
        if self.recorder:
            self.recorder.close()
        self.bus = None
        self.motion = None
        self.connected = False
        self.mode = None
        self.cache = {}
        self.preview = None
        self.pending = None
        self._home_preview = None
        self.msg = "未连接"

    # ---------------- 状态轮询 ----------------
    def _poll_loop(self):
        last_cfg_check = 0.0
        period = 1.0 / self.push_hz
        while not self._poll_stop.is_set():
            t0 = time.monotonic()
            try:
                now = t0
                if now - last_cfg_check > 1.0:
                    last_cfg_check = now
                    busy = bool(self.motion and self.motion.status.get("busy"))
                    if not busy and cfgmod.config_version() != self.config_version:
                        self.refresh_config_version()
                        self.msg = "配置已更新（sign/零位/限位），已热重载"
                self._refresh()
            except Exception as exc:  # noqa: BLE001
                self.msg = f"轮询异常：{exc}"
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _refresh(self):
        if self.use_actual():
            m = self.motion
            busy = m.status.get("busy")
            if busy:
                q = m.status.get("current_q")
                states = self._last_states
            else:
                full = time.monotonic() - self._last_full > 5.0
                states = self.bus.read_all(full=full)
                if full:
                    self._last_full = time.monotonic()
                self._last_states = states
                q = [states[n].out_deg for n in self.robot.joint_names]
                if any(v is None for v in q):
                    q = None
            if q is None:
                # 连接了但还没使能/读不到位置：退回零位预演模型
                states = self.preview_bus.read_all()
                q = [0.0] * 6
        else:
            states = self.preview_bus.read_all()
            q = [0.0] * 6
        with self._lock:
            self.cache = {"states": states, "q": q}
        self._sync_motion_params()
        self._hold_tick(q)
        if q is not None:
            try:
                self.cache["scene"] = self.fence.scene(q, self.tcp_preset, self.yaw_deg)
                self.cache["tcp"] = self.robot.tcp(q, self.tcp_preset, self.yaw_deg)[:3, 3].tolist()
                self.cache["flange"] = self.robot.flange(q)[:3, 3].tolist()
                fr = self.fence.check(q, self.tcp_preset, self.yaw_deg, False, self.limits)
                self.cache["fence"] = fr.as_dict()
                self.cache["near_limit"] = fr.near_limit
            except Exception as exc:  # noqa: BLE001
                self.msg = f"场景计算异常：{exc}"

    # ---------------- 状态 ----------------
    def save_web_state(self, snap: dict) -> dict:
        """页面运行时快照：只保留最新一份，覆盖写 runs/web_state.json。"""
        if not isinstance(snap, dict):
            raise MotionError("page_state 必须是 JSON object")
        snap = dict(snap)
        snap["server_received_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        raw = json.dumps(snap, ensure_ascii=False, allow_nan=False)
        if len(raw.encode("utf-8")) > 2_000_000:
            raise MotionError("page_state 超过 2 MB，拒绝")
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        path = self.runs_dir / "web_state.json"
        tmp = path.with_suffix(".json.tmp")
        with self._web_state_lock:
            tmp.write_text(raw + "\n", encoding="utf-8")
            tmp.replace(path)
        return {"ok": True, "path": str(path), "bytes": len(raw.encode("utf-8"))}

    def load_web_state(self) -> dict:
        path = self.runs_dir / "web_state.json"
        if not path.is_file():
            return {"ok": False, "error": "还没有 web_state.json；先让网页刷新并保持打开"}
        try:
            return {"ok": True, **json.loads(path.read_text(encoding="utf-8"))}
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

    def state(self) -> dict:
        with self._lock:
            cache = dict(self.cache)
            states = cache.get("states") or {}
            q = cache.get("q")
            m_state = self.motion if self.use_actual() else self.preview_motion
            ms = dict(m_state.status) if m_state else {}
            preview = self.preview
            pending = bool(self.pending)
        ms.pop("segments", None)
        joints = []
        for n in self.robot.joint_names:
            st = states.get(n)
            joints.append(st.as_dict() if st is not None else
                          {"name": n, "id": self.joint_cfg[n].id,
                           "enabled": None, "out_deg": None})
        # 六关节安全限位检查：实际位置只有在 startup_done 后才是真机角；
        # 未使能时固件不回位置，页面只显示“待启动检查”，不拿预演 q=0 冒充。
        startup = bool(getattr(self.bus, "startup_done", False))
        q_actual = cache.get("q")
        if not self.connected:
            safe_check = {"ok": None, "state": "offline",
                          "text": "未连接：连接后实际角度要等零扭矩启动读取"}
        elif not startup or q_actual is None:
            safe_check = {"ok": None, "state": "unknown",
                          "text": "未使能：实际角度由启动序列逐颗零扭矩读取；"
                                  "超安全限位会拒绝使能"}
        else:
            lo, hi = self.limits
            rows = []
            bad = []
            for i, n in enumerate(self.robot.joint_names):
                qv = float(q_actual[i])
                a, b = float(lo[i]), float(hi[i])
                ok = a - 1e-6 <= qv <= b + 1e-6
                rows.append({"joint": n, "q": round(qv, 2),
                             "lo": round(a, 1), "hi": round(b, 1), "ok": ok})
                if not ok:
                    bad.append(f"{n} {qv:+.1f}°（{a:.0f}..{b:.0f}）")
            safe_check = {
                "ok": not bad, "state": "ok" if not bad else "bad",
                "rows": rows,
                "text": ("六关节安全限位正常" if not bad else
                         "超安全限位：" + "，".join(bad)
                         + "；禁止使能/运动，先断电手动移回"),
            }
        now = time.monotonic()
        return {
            "ok": True, "ts": time.time(),
            "connected": self.connected, "mode": self.mode,
            "mock": self.mode == "mock", "ip": self.ip, "msg": self.msg,
            "startup_done": bool(getattr(self.bus, "startup_done", False)),
            "q": q, "tcp": cache.get("tcp"), "flange": cache.get("flange"),
            "scene": cache.get("scene"), "fence": cache.get("fence"),
            "near_limit": cache.get("near_limit", []), "joints": joints,
            "safe_check": safe_check,
            "motion": ms, "tcp_preset": self.tcp_preset, "yaw": self.yaw_deg,
            "limits": {"lo": self.limits[0], "hi": self.limits[1]},
            "preview": preview, "has_plan": pending,
            "recorder": str(self.recorder.path) if self.recorder else None,
            "stats": (self.bus.stats() if self.bus else None),
            "exec_mode": self.exec_mode,
            "config_version": self.config_version,
            "calib": self.calib.snapshot(),
            "drag": {"active": bool(self.drag_active)},
            "measurements": self.measurements[-20:],
            "startup_report": (getattr(self.bus, "startup_report", None)
                               if self.bus else None),
            "end_effector": self.end_effector,
            "payload_kg": self.payload_kg,
            "capacities": self.capacities(),
            "hold": {"ratio": self.hold.get("ratio", 0.0),
                     "since_s": (None if self.hold.get("since") is None else
                                 round(time.monotonic() - self.hold["since"], 1)),
                     "warn": self.hold.get("warn", ""),
                     "threshold": gravity.HOLD_WARN_RATIO},
            "park": {**self.park_config(),
                     "step": dict(self.park_step),
                     "check": self._park_check(q),
                     "block": self.park_block},
            "kt": {"rows": self.kt_rows, "kt": self.kt},
            "control": {
                "owner": self.owner,
                "age_s": None if not self.owner else round(
                    now - max(self.last_hb, self.clients.get(self.owner, 0.0)), 2),
                "stale_s": self.control_stale,
                "timeout_s": self.hb_timeout,
            },
        }

    def calibrated(self) -> tuple[bool, list[str]]:
        if self.mode == "mock":
            return True, []
        cfg = load_joint_configs()
        missing = [n for n, c in cfg.items() if not (c.zero_date and c.sign_checked)]
        return (not missing), missing

    # ---------------- 规划 ----------------
    def _ready_for_motion(self, allow_hold: bool = False):
        if self.drag_active:
            raise MotionError("零扭矩拖动中，先“锁定保持”再执行")
        if self.calib.active:
            raise MotionError("校准会话进行中，先结束校准")
        if not self.motion or not self.connected:
            raise MotionError("还没连接")
        if not getattr(self.bus, "startup_done", False):
            raise MotionError("还没完成启动/使能，不能执行")
        if self.park_block:
            raise MotionError(
                f"启停比对不过（和 parked_q 差 {self.park_block.get('max_diff_deg')}°）："
                "看过实物、确认没问题后点“已确认”才解除")
        phase = (self.motion.status or {}).get("phase")
        if phase in ("aborted", "error") and not allow_hold:
            raise MotionError("臂在中止保持状态：先点“恢复”，或走“停机 / 扶持停机”")
        ok, missing = self.calibrated()
        if not ok:
            raise MotionError("还没完成设零/方向核对：缺 " + ", ".join(missing))

    # ---------------- 18.1 / 18.2 搁架、末端配置、力矩 ----------------
    def park_config(self) -> dict:
        """按 park_side 调整搁架位置和三个停机点（q1 取 ±90°）。"""
        pk = json.loads(json.dumps(self.park_cfg))   # deep copy
        taught = bool(pk.get("park_taught"))
        if taught:
            qp = list((pk.get("poses") or {}).get("park") or [0] * 6)
            # 教的最终位置决定 J1 在哪一侧；tuck/pre 仍用该侧的 ±90°
            side = -1 if float(qp[0]) < 0 else 1
            pk["park_side"] = side
        side = int(pk.get("park_side", -1))
        sh = pk.get("shelf") or {}
        if side > 0 and sh.get("center"):
            sh["center"] = [-float(sh["center"][0]), -float(sh["center"][1])]
            sh["rotate_deg"] = 90.0
        pk["park_side"] = side
        pk["shelf"] = sh
        q1 = 90.0 * side
        poses = {}
        for key in ("tuck", "pre", "park"):
            q = list((pk.get("poses") or {}).get(key) or [0] * 6)
            # 教过的 park 保留 q1；tuck/pre 仍按 park_side 的 ±90°
            if not (taught and key == "park"):
                q[0] = q1
            poses[key] = q
        pk["poses"] = poses
        return pk

    def current_limits(self) -> dict:
        """回读限流（A）；没有回读值时退回配置里的要求值。"""
        out = {}
        states = self.cache.get("states") or {}
        for n in self.robot.joint_names:
            st = states.get(n)
            v = getattr(st, "current_limit_a", None) if st is not None else None
            if v is None:
                v = getattr(self.joint_cfg.get(n), "required_current_limit_a", None)
            out[n] = None if v is None else float(v)
        return out

    def _sync_motion_params(self):
        limits = self.current_limits()
        for m in (self.motion, self.preview_motion):
            if m is None:
                continue
            m.end_effector = self.end_effector
            m.payload_kg = float(self.payload_kg)
            m.limit_a = {k: v for k, v in limits.items() if v is not None}
            m.kt = self.kt
            m.j2_bracket = self.j2_bracket

    def capacities(self) -> dict:
        cap = gravity.capacities(
            {k: v for k, v in self.current_limits().items() if v is not None},
            self.kt, self.j2_bracket)
        return {k: (None if not math.isfinite(v) else round(float(v), 3))
                for k, v in cap.items()}

    def set_end_effector(self, end_effector: str, payload_kg=None) -> dict:
        ee = str(end_effector or "")
        if ee not in gravity.END_EFFECTORS:
            raise MotionError("末端只能是 " + " / ".join(gravity.END_EFFECTORS))
        if payload_kg is not None:
            self.payload_kg = max(0.0, float(payload_kg))
        self.end_effector = ee
        self._sync_motion_params()
        warn = ""
        if ee == "gripper":
            j2a = self.current_limits().get("J2")
            if j2a is None or j2a <= 4.5:
                warn = (f"带爪但 J2 回读限流 {j2a if j2a is not None else '未知'} A ≤ 4.5 A；"
                        "按 16.3 第 2 条改 6 A（用户在场）。容量仍按回读值算，"
                        "带爪、前臂水平的姿态会被 18.2 预检拒绝。")
                self.msg = warn
        return {"ok": True, "end_effector": ee, "payload_kg": self.payload_kg,
                "capacities": self.capacities(), "warn": warn}

    def _hold_tick(self, q):
        """18.2 第 4 条：保持/停留时 τ/容量 >70% 持续 30 s → 提醒。"""
        if not q or not self.use_actual() or not getattr(self.bus, "startup_done", False):
            self.hold = {"since": None, "ratio": 0.0, "warn": ""}
            return
        if (self.motion and self.motion.status.get("busy")) or self.drag_active \
                or self.calib.active:
            self.hold["since"] = None
            return
        try:
            ratios = gravity.hold_ratios(self.robot, q, self.end_effector, self.payload_kg,
                                         {k: v for k, v in self.current_limits().items() if v},
                                         self.kt, self.j2_bracket)
        except Exception:  # noqa: BLE001
            return
        mx = max([v for v in ratios.values() if v is not None] or [0.0])
        now = time.monotonic()
        self.hold["ratio"] = round(float(mx), 3)
        if mx > gravity.HOLD_WARN_RATIO:
            if self.hold.get("since") is None:
                self.hold["since"] = now
            if now - self.hold["since"] > gravity.HOLD_WARN_S and not self.hold.get("warn"):
                self.hold["warn"] = (f"保持力矩高（{mx * 100:.0f}% 容量，超过 70% 已 "
                                     f"{now - self.hold['since']:.0f} s），请恢复或停机")
                self.msg = self.hold["warn"]
        else:
            self.hold["since"] = None
            self.hold["warn"] = ""

    def _park_check(self, q) -> dict:
        parked = (self.park_state or {}).get("parked_q")
        if not parked:
            return {"has_parked": False, "ok": None}
        if not q:
            return {"has_parked": True, "ok": None}
        diffs = {n: round(abs(float(a) - float(b)), 2)
                 for n, a, b in zip(self.robot.joint_names, q, parked)}
        d = max(diffs.values())
        return {"has_parked": True, "ok": d <= float(
                    (self.park_cfg.get("restart_check_deg") or 2.0)),
                "max_diff_deg": round(d, 2), "diffs": diffs, "parked_q": parked,
                "ts": (self.park_state or {}).get("ts", "")}

    def plan_click(self, x: float, y: float, z: float = 20.0,
                   yaw: float | None = None) -> dict:
        m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动，点击已忽略")
        if yaw is not None:
            self.yaw_deg = float(yaw)
            if self.motion:
                self.motion.yaw_deg = self.yaw_deg
            self.preview_motion.yaw_deg = self.yaw_deg
        segs = m.plan_click(float(x), float(y), float(z), self.yaw_deg, fast=True)
        self.pending = segs
        last = segs[-1]
        self.preview = {
            "segments": brief_segments(segs),
            "scene": self.fence.scene(last["q_target"], self.tcp_preset, self.yaw_deg),
            "q": last["q_target"],
            "tcp": self.robot.tcp(last["q_target"], self.tcp_preset, self.yaw_deg)[:3, 3].tolist(),
            "fence": last["fence"],
            "ik": last.get("ik"),
            "target": [float(x), float(y), float(z)],
            "yaw": self.yaw_deg,
            "near_limit": last.get("near_limit", []),
            "offline": not self.use_actual(),
        }
        return {"ok": True, "preview": self.preview,
                "segments": brief_segments(segs), "started": False}

    def click(self, x: float, y: float, z: float = 20.0,
              yaw: float | None = None) -> dict:
        r = self.plan_click(x, y, z, yaw)
        if self.exec_mode == "direct":
            self._ready_for_motion()
            r["started"] = True
            self._start_exec(self.pending, preview=False, what="虚拟点击",
                             motion=self.motion)
            r["msg"] = "直接执行"
        else:
            r["msg"] = ("预演（离线，零位模型）：确认后点执行" if not self.use_actual()
                        else "预演：确认后点执行")
        return r

    def execute(self, preview: bool | None = None, use_pending: bool = True) -> dict:
        # 默认按预演处理，避免漏传 preview 时直接动电机
        preview = True if preview is None else bool(preview)
        segs = self.pending if use_pending and self.pending else None
        if not segs:
            raise MotionError("没有待执行的计划")
        m = self.planning_motion()
        if not preview:
            self._ready_for_motion()
            m = self.motion
        for seg in segs:
            m._check_q(seg["q_target"], seg.get("descend", False))
        return self._start_exec(segs, preview, "执行", motion=m)

    def _start_exec(self, segments, preview: bool, what: str,
                    motion: Motion | None = None):
        if self._exec_thread and self._exec_thread.is_alive():
            raise MotionError("已经在执行")
        m = motion or self.motion
        if m is None:
            raise MotionError("还没连接，不能执行")
        if not preview:
            pr = (self.robot.tcp_presets or {}).get(self.tcp_preset) or {}
            if pr.get("measured") is False:
                raise MotionError(
                    f"TCP 预设 {self.tcp_preset} 还没实测（measured=false）："
                    "只允许预演；先用卡尺量 L、写入 config/tcp.json 并置 measured=true")

        def worker():
            try:
                if not preview:
                    qnow = m.read_q()
                    q0 = segments[0].get("q_from") if segments else None
                    if q0 is not None and max(abs(a - b) for a, b in zip(qnow, q0)) > 3.0:
                        raise MotionError("当前姿态和预演时不一致，请重新预演后再执行")
                m.execute(segments, preview=preview)
            except MotionError as exc:
                self.msg = f"{what}中止：{exc}"
            except Exception as exc:  # noqa: BLE001
                self.msg = f"{what}失败：{exc}"
                traceback.print_exc()
        self._exec_thread = threading.Thread(target=worker, daemon=True)
        self._exec_thread.start()
        return {"ok": True, "started": True, "preview": preview}

    def jog(self, joint: str, delta: float, preview: bool = False) -> dict:
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        joint = joint.upper()
        if joint not in self.robot.joint_names:
            raise MotionError(f"未知关节 {joint}")
        q = m.read_q()
        i = self.robot.joint_names.index(joint)
        q2 = list(q)
        q2[i] += float(delta)
        segs = m._plan_move(q, q2, f"点动 {joint}{delta:+.0f}°", False)
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        return self._start_exec(segs, False, f"点动 {joint}", motion=self.motion)

    def home(self, preview: bool = False) -> dict:
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        q_home = None
        if not preview:
            hp = self._home_preview
            qnow = m.read_q()
            if (not hp or time.time() - hp["t"] > 60.0
                    or max(abs(a - b) for a, b in zip(hp["q"], qnow)) > 0.02):
                raise MotionError("回零位必须先对当前姿态预演；请先点一次“回零位（预演）”，检查轨迹/围栏后再执行")
            self._home_preview = None
        else:
            q_home = list(m.read_q())
        segs = m.plan_home()
        if preview and q_home is not None:
            self._home_preview = {"q": q_home, "t": time.time()}
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            # 回零可能是“抬高 → 回零位”两段；把完整分段一并给网页显示
            return {"ok": True, "preview": self.preview,
                    "segments": brief_segments(segs), "started": False}
        return self._start_exec(segs, False, "回零", motion=self.motion)

    def move_joints(self, q, preview: bool = False, descend: bool = False) -> dict:
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        if len(q) != 6:
            raise MotionError("需要 6 个关节角")
        segs = m.plan_joint_move(q, descend=descend, label="move_joints")
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        return self._start_exec(segs, False, "move_joints", motion=self.motion)

    def move_tcp(self, target, axis=(0, 0, -1), yaw=None, preview: bool = False,
                 descend: bool = False, preset: str | None = None,
                 free_yaw: bool = False) -> dict:
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        if yaw is not None:
            self.yaw_deg = float(yaw)
            if self.motion:
                self.motion.yaw_deg = self.yaw_deg
            self.preview_motion.yaw_deg = self.yaw_deg
        segs = m.plan_tcp_move(target, axis, self.yaw_deg,
                               preset=preset, descend=descend,
                               label="move_tcp", free_yaw=free_yaw,
                               fast=preview)
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["target"] = [round(float(v), 2) for v in target]
        self.preview["source"] = "tcp"
        self.preview["free_yaw"] = bool(free_yaw)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        return self._start_exec(segs, False, "move_tcp", motion=self.motion)

    def move_through(self, waypoints, descend_flags=None, labels=None,
                     preview: bool = False) -> dict:
        if not preview:
            self._ready_for_motion()
            m = self.motion
        else:
            m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动")
        segs = m.plan_through(waypoints, descend_flags, labels)
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        return self._start_exec(segs, False, "move_through", motion=self.motion)

    # ---------------- 18.3 停机 / 从停机启动 ----------------
    def _sample_currents(self, names, n=None) -> dict:
        n = int(n or self.park_cfg.get("sample_n") or 20)
        acc = {k: [] for k in names}
        for _ in range(n):
            st = self.bus.read_all(full=True)
            for k in names:
                v = getattr(st.get(k), "current_a", None)
                if v is not None:
                    acc[k].append(float(v))
            time.sleep(0.02)
        return {k: (round(sum(v) / len(v), 4) if v else None) for k, v in acc.items()}

    def park(self, preview: bool = False, prereq: dict | None = None) -> dict:
        m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动，先按停")
        if self.end_effector != "flange":
            raise MotionError(
                "停机只对空法兰开放（18.1a：带爪失能后静力余量只有 1.1 倍；"
                "腕托做好之前，带爪请用“扶持停机”）")
        if not preview:
            self._ready_for_motion(allow_hold=True)   # 接住失败后允许重做停机
            m = self.motion
            st = self.bus.read_all(full=True)
            alarms = [f"{n}(0x{s.error:04X})" for n, s in st.items() if s.error]
            if alarms:
                raise MotionError("有报警位非零：" + "，".join(alarms)
                                  + "；先走清报警流程再停机")
        missing = [label for key, label in (
            ("gripper_empty", "夹爪里没有东西"),
            ("workspace_clear", "工作区已清空"),
            ("shelf_ready", "搁架已放好")) if not (prereq or {}).get(key)]
        if missing:
            raise MotionError("停机前提没勾选：" + "、".join(missing))
        pk = self.park_config()
        q0 = m.read_q()
        plan = m.plan_park(q0, pk)
        segs = plan["lift"] + plan["tuck"] + plan["pre"]
        self.pending = segs
        last = plan["park"][-1] if plan["park"] else segs[-1]
        self.preview = preview_payload(last, self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        self.preview["park"] = {"poses": pk["poses"], "shelf": pk["shelf"],
                                "note": "P 只是名义点；实际按 1 mm 分步下降找接触（18.3 第 4 步）"}
        if preview:
            return {"ok": True, "preview": self.preview,
                    "segments": brief_segments(segs + plan["park"]), "started": False,
                    "phases": {"lift": len(plan["lift"]), "tuck": len(plan["tuck"]),
                               "pre": len(plan["pre"]), "descent": "1 mm/步，最多到 z=130.5"}}
        self._start_park(plan)
        return {"ok": True, "started": True, "msg": "停机流程已启动（到预停机点后分步下降）"}

    def teach_park(self) -> dict:
        """把当前实际六关节位置记录为搁架停机位置（只记录，不运动）。"""
        if self.end_effector != "flange":
            raise MotionError("当前不是空法兰：停机只对 flange 开放，先切回 flange 再教")
        if not self.connected or not getattr(self.bus, "startup_done", False):
            raise MotionError("还没使能：实际角度要先完成零扭矩启动才能读，不能记录")
        if self.motion is None:
            raise MotionError("还没连接")
        if self.motion.status.get("busy") or self.drag_active:
            raise MotionError("正在运动/零扭矩拖动：先停住/退出拖动，等保持稳定再记录")
        if self._park_thread and self._park_thread.is_alive():
            raise MotionError("停机流程正在跑，不能改停机位置")
        q = list(self.motion.read_q())
        if len(q) != 6 or any(v is None for v in q):
            raise MotionError("读不到完整六关节位置，不能记录")
        lo, hi = self.limits
        bad = [f"{n} {q[i]:+.1f}°（{lo[i]:.0f}..{hi[i]:.0f}）"
               for i, n in enumerate(self.robot.joint_names)
               if not (lo[i] - 1e-6 <= q[i] <= hi[i] + 1e-6)]
        if bad:
            raise MotionError("当前位置超安全限位，不能记录：" + "，".join(bad))
        pk = json.loads(json.dumps(self.park_cfg))
        poses = pk.get("poses") or {}
        poses["park"] = [round(float(v), 4) for v in q]
        pk["poses"] = poses
        pk["park_taught"] = True
        pk["park_taught_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        pk["park_taught_source"] = "web: current actual q"
        pk["park_side"] = -1 if float(q[0]) < 0 else 1
        if not pk.get("version"):
            pk["version"] = "2026-11-12.1"
        cfgmod.save_park(pk)
        self.park_cfg = pk
        self.msg = ("停机位置已记录："
                    + " ".join(f"{n}{v:+.1f}" for n, v in zip(self.robot.joint_names, q)))
        return {"ok": True, "q": [round(float(v), 4) for v in q],
                "park_side": int(pk["park_side"]),
                "taught_at": pk["park_taught_at"], "msg": self.msg}

    def park_ack(self) -> dict:
        """18.3 从停机启动比对不过：用户看过实物后点“已确认”，解除硬拦。"""
        if not self.park_block:
            return {"ok": True, "msg": "当前没有启停比对拦截"}
        self.park_block = None
        self.msg = "已确认启停比对差异，解除运动拦截"
        return {"ok": True, "msg": self.msg}

    def _start_park(self, plan: dict):
        if self._park_thread and self._park_thread.is_alive():
            raise MotionError("停机流程已经在跑")
        self.park_step = {"phase": "moving_pre", "reason": "", "z": None, "step": 0,
                          "I0": None, "I2": None, "I1": None, "contact_z": None}

        def worker():
            m = self.motion
            gap = float(self.park_cfg.get("disable_gap_s") or 0.5)
            settle = float(self.park_cfg.get("settle_s") or 1.0)
            tol = float(self.park_cfg.get("disable_check_deg") or 1.0)
            ratio = float(self.park_cfg.get("catch_ratio") or 0.5)
            step_mm = float(self.park_cfg.get("descent_step_mm") or 1.0)
            z_min = float(self.park_cfg.get("descent_min_j4_z") or 130.5)
            q1 = list(plan["poses"]["pre"])[0]
            pre_q = list(plan["poses"]["pre"])

            def q_at_j4z(z: float) -> list[float]:
                c = max(-1.0, min(1.0, (z - 120.5) / 203.5))
                q2 = math.degrees(math.acos(c))
                return [q1, q2, 90.0 - q2, 90.0, 0.0, 0.0]

            try:
                # 1–3：抬高、收拢、到预停机点，读 I₀
                m.execute(plan["lift"] + plan["tuck"] + plan["pre"], preview=False)
                time.sleep(settle)
                i0 = self._sample_currents(["J2", "J3"])
                self.park_step.update({"phase": "at_pre", "I0": i0})
                sgn = 1.0 if (i0.get("J2") or 0.0) >= 0 else -1.0
                # 4：1 mm/步分步下降找接触（q2+q3=90，前臂保持水平）
                z = 168.0
                cur_q = m.read_q()
                lowest = None
                contact = False
                while z > z_min + 1e-6:
                    nz = max(z_min, round(z - step_mm, 3))
                    segs = m._plan_move(cur_q, q_at_j4z(nz), f"分步下降 z={nz:.1f}")
                    m.execute(segs, preview=False)
                    time.sleep(0.15)
                    cur_q = m.read_q()
                    z = nz
                    i2 = self._sample_currents(["J2"]).get("J2")
                    lowest = i2
                    self.park_step.update({"phase": "descending", "z": round(z, 2),
                                           "step": int(round((168.0 - z) / step_mm)),
                                           "I2": i2})
                    if i2 is not None and sgn * i2 <= ratio * abs(i0.get("J2") or 0.0):
                        contact = True
                        self.park_step["contact_z"] = round(z, 2)
                        break
                if not contact:
                    back = m._plan_move(cur_q, pre_q, "回预停机点")
                    m.execute(back, preview=False)
                    self.park_step.update({"phase": "catch_failed",
                                           "lowest_I2": lowest, "z": round(z, 2)})
                    self.msg = (f"搁架没接住：I₀ {i0.get('J2')} A，降到 z={z:.1f} 时 J2 电流 "
                                f"{lowest} A（要 ≤{ratio:g}·I₀）。已回预停机点保持，没有失能。"
                                "检查搁架位置/垫片后可以直接再按“停机”重做，或点“恢复”。")
                    return
                # 再多走一步让搁架接实，然后读 I₁
                if z > z_min + 1e-6:
                    nz = max(z_min, round(z - step_mm, 3))
                    segs = m._plan_move(cur_q, q_at_j4z(nz), f"再走一步 z={nz:.1f}")
                    m.execute(segs, preview=False)
                    z = nz
                    cur_q = m.read_q()
                time.sleep(settle)
                i1 = self._sample_currents(["J2", "J3"])
                self.park_step.update({"phase": "at_park", "I1": i1, "z": round(z, 2)})
                # 7：写 parked_q（取失能前最后一次读到的值）
                q_park = m.read_q()
                self.park_state = {"parked_q": [round(float(v), 4) for v in q_park],
                                   "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
                                   "I0": i0, "I1": i1, "contact_z": round(z, 2),
                                   "end_effector": self.end_effector}
                cfgmod.save_park_state(self.park_state)
                # 6：按序失能，每颗之后检查其余还使能的关节
                order = list(self.park_cfg.get("disable_order") or
                             ["J6", "J5", "J4", "J3", "J2", "J1"])
                for name in order:
                    raw_before = self.bus.read_positions_turns()
                    if name not in raw_before:
                        self.bus.hold_all()
                        self.park_step.update({"phase": "disable_aborted",
                                               "reason": f"{name} 失能前读不到位置"})
                        self.msg = (f"停机在失能 {name} 前读不到它的位置，"
                                    "已停在保持、没有继续失能")
                        return
                    self.bus.disable_one(name)
                    time.sleep(gap)
                    raw_after = self.bus.read_positions_turns()
                    moved = {}
                    for k, v in raw_before.items():
                        if k == name or k not in raw_after:
                            continue
                        deg = abs(self.joint_cfg[k].out_deg(raw_after[k])
                                  - self.joint_cfg[k].out_deg(v))
                        if deg > tol:
                            moved[k] = round(deg, 2)
                    if moved:
                        self.bus.enable_one_hold(name, raw_before.get(name, 0.0))
                        self.park_step.update({"phase": "disable_aborted",
                                               "moved": moved})
                        self.msg = (f"失能 {name} 时这些关节动了 {moved}°，"
                                    f"已把 {name} 重新使能到保持并停止停机流程；"
                                    "请检查支撑后再处理。")
                        return
                    self.park_step["phase"] = f"disabled_{name}"
                self.park_step.update({"phase": "parked"})
                self.msg = "已停机（搁架接住，六颗按 J6→J1 失能），可以关电源"
            except MotionError as exc:
                self.park_step.update({"phase": "aborted", "reason": str(exc)})
                self.msg = f"停机中止：{exc}（臂保持使能，没有失能）"
            except Exception as exc:  # noqa: BLE001
                traceback.print_exc()
                self.park_step.update({"phase": "error", "reason": str(exc)})
                self.msg = f"停机失败：{exc}"

        self._park_thread = threading.Thread(target=worker, daemon=True)
        self._park_thread.start()

    def unpark(self, preview: bool = False, confirmed: bool = False) -> dict:
        """从停机启动：J2/J3 同步抬回预停机点（离开搁架）。"""
        m = self.planning_motion()
        if m.status.get("busy"):
            raise MotionError("正在运动，先按停")
        if not preview:
            self._ready_for_motion()
            m = self.motion
        q0 = m.read_q()
        chk = self._park_check(q0)
        if chk.get("has_parked") and chk.get("ok") is False and not confirmed:
            raise MotionError(
                f"当前姿态和 parked_q 差 {chk.get('max_diff_deg')}°（>2°），"
                "停机后可能有人动过臂；请先确认再点“确认抬回”")
        segs = m.plan_unpark(q0, self.park_config())
        self.pending = segs
        self.preview = preview_payload(segs[-1], self.robot, self.fence,
                                       self.tcp_preset, self.yaw_deg)
        self.preview["offline"] = not self.use_actual()
        if preview:
            return {"ok": True, "preview": self.preview, "started": False}
        self._start_exec(segs, False, "抬回预停机点", motion=self.motion)
        return {"ok": True, "started": True,
                "msg": "抬回预停机点；离开搁架后可以把搁架拿走"}

    def recover(self) -> dict:
        """16.4 第 7 条：中止后保持状态的出口“恢复”。"""
        if not self.bus or not self.motion:
            raise MotionError("还没连接")
        if self.motion.status.get("busy"):
            raise MotionError("还在运动/预演，不能恢复")
        if self.motion.status.get("phase") not in ("aborted", "error", "stopping"):
            raise MotionError("当前不在中止保持状态")
        self.bus.hold_all()
        with self.motion._lock:
            self.motion.status.update({"phase": "idle", "reason": "", "label": ""})
        self.hold["since"] = None
        self.hold["warn"] = ""
        self.pending = None
        self.preview = None
        self.msg = "已恢复：原地保持；可以重新预演或执行"
        return {"ok": True, "msg": self.msg}

    def support_park(self, confirmed: bool = False) -> dict:
        """扶持停机（备用）：回零位 + 用户扶住 + 二次确认 → 六颗一起失能。"""
        if not confirmed:
            raise MotionError("扶持停机要二次确认（提示：臂在零位且已扶住）")
        if not self.bus or not self.motion:
            raise MotionError("还没连接")
        if self.motion.status.get("busy"):
            raise MotionError("正在运动；扶持停机要求臂已在零位并扶住")
        q = self.motion.read_q()
        if max(abs(float(v)) for v in q) > 2.0:
            raise MotionError("臂不在零位；先回零位并用手扶住前臂，再扶持停机")
        ok = self.bus.disable_all(True, "扶持停机（用户扶住）")
        self.park_step = {"phase": "support_parked", "reason": "", "I0": None, "I1": None}
        self.pending = None
        self.preview = None
        self.msg = "扶持停机：已六颗一起失能"
        return {"ok": bool(ok)}

    # ---------------- 18.4 Kt 标定 ----------------
    def kt_points(self) -> dict:
        return {"ok": True, "points": gravity.KT_POINTS,
                "j5_points": gravity.KT_J5_POINTS,
                "kt": self.kt, "rows": self.kt_rows,
                "note": "J5 一组要等夹爪装回；挂重只在保持状态下加减，人不要站在 +x"}

    def kt_goto(self, q, preview: bool = False) -> dict:
        return self.move_joints(q, preview=preview, descend=False)

    def kt_sample(self, joints, load_kg: float = 0.0, label: str = "") -> dict:
        if not self.use_actual():
            raise MotionError("Kt 标定要连真机并已使能")
        if not getattr(self.bus, "startup_done", False):
            raise MotionError("还没使能")
        names = [str(n).upper() for n in (joints or [])]
        if not names:
            raise MotionError("缺 joints")
        # 18.4：每个点到位后零速保持 2 s 再取样
        time.sleep(2.0)
        q = self.motion.read_q()
        tgt = (self.motion.status or {}).get("target_q")
        tau = gravity.torques(self.robot, q, "flange", float(load_kg))
        cur = self._sample_currents(names)
        fs, lim, raw = {}, {}, {}
        for n in names:
            fs[n] = self.bus.current_full_scale_a(n)      # 0x53 原始满量程
            lim[n] = self.bus.current_limit_a(n)          # 0x59 × 0x53，A
            raw[n] = self.bus.current_raw_counts(n)       # (0x04, 0x59) 原始计数
        rows = []
        for n in names:
            i = cur.get(n)
            if i is None:
                raise MotionError(f"{n} 读不到电流")
            idx = int(n[1]) - 1
            err = None
            if tgt and len(tgt) == 6:
                err = round(abs(float(tgt[idx]) - float(q[idx])), 3)
            row = {"joint": n, "current": round(float(i), 5),
                   "tau": round(float(tau[idx]), 5),
                   "load_kg": float(load_kg), "label": label,
                   "q": [round(float(v), 3) for v in q],
                   "err_deg": err,
                   "full_scale_a": fs.get(n),
                   "limit_a": lim.get(n),
                   "raw_current": (raw.get(n) or (None, None))[0],
                   "raw_limit": (raw.get(n) or (None, None))[1]}
            self.kt_rows.append(row)
            rows.append(row)
        return {"ok": True, "rows": rows, "all_rows": self.kt_rows}

    def kt_fit(self, save: bool = True) -> dict:
        fit = gravity.fit_kt(self.kt_rows) if self.kt_rows else {}
        for name, rec in fit.items():
            rec["kt_date"] = time.strftime("%Y-%m-%d")
        if save and fit:
            data = dict(self.kt or {})
            data.update(fit)
            data["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
            gravity.save_kt(data)
            self.kt = gravity.load_kt()
            self._sync_motion_params()
            self.refresh_config_version()
        yes = [n for n, r in fit.items()
               if r.get("r2") is not None and r["r2"] >= 0.95]
        return {"ok": True, "fit": fit, "saved": bool(save and fit),
                "r2_ok": yes, "capacities": self.capacities()}

    def kt_reset(self) -> dict:
        self.kt_rows = []
        return {"ok": True, "rows": []}

    # ---------------- 维护：只写限位/限流/限速，不动零位 ----------------
    def write_limits_all(self) -> dict:
        if not self.bus or not self.motion:
            raise MotionError("还没连接")
        if self.calib.active or self.motion.status.get("busy"):
            raise MotionError("校准/运动进行中，先停")
        if self.drag_active:
            raise MotionError("零扭矩拖动中，先锁定保持")
        self.pending = None
        self.preview = None
        rows = []
        for name in self.bus.order:
            m = self.bus.motors[name]
            c = self.bus.cfg[name]
            was_on = m.power() == 1
            if not was_on:
                state_changed = True
                try:
                    m.set_mode(MODE_CURRENT)
                    m.set_current_a(0.0)
                except Exception:  # noqa: BLE001
                    pass
                if not m.enable():
                    raise MotionError(f"{name} 使能无应答")
                m.set_mode(MODE_CURRENT)
                m.set_current_a(0.0)
                time.sleep(0.15)
            lo, hi = c.eeprom_limits_deg
            lo_t = lo / 10.0
            hi_t = hi / 10.0
            m.set_pos_limits_turns(lo_t, hi_t)
            if c.required_current_limit_a is not None:
                m.set_current_limit_a(c.required_current_limit_a)
            m.set_speed_limit_out_deg_s(c.eeprom_speed_limit_out_deg_s)
            m.save()
            time.sleep(0.3)
            lim = m.pos_limits_turns()
            spd = m.speed_limit_rpm()
            cur = m.current_limit_a()
            ok = (lim is not None and abs(lim[0] - lo_t) < 0.01
                  and abs(lim[1] - hi_t) < 0.01
                  and spd is not None and abs(spd - 360.0) < 1.0)
            if c.required_current_limit_a is not None:
                ok = ok and cur is not None and abs(cur - c.required_current_limit_a) < 0.05
            rows.append({"name": name, "ok": bool(ok),
                         "limits_deg": [lo, hi],
                         "limits_readback": None if lim is None else
                         [round(lim[0] * 10.0, 1), round(lim[1] * 10.0, 1)],
                         "current_limit_a": None if cur is None else round(cur, 2),
                         "speed_limit_rpm": None if spd is None else round(spd, 1)})
            if was_on:
                pos = m.pos_turns()
                if pos is not None:
                    m.set_target_turns(pos)
                    m.set_mode(MODE_PROFILE_POSITION)
                    self.bus._write_profile(name, c.motion_speed_deg_s)
                    m.set_target_turns(pos)
            else:
                m.disable()
        bad = [r["name"] for r in rows if not r["ok"]]
        self.msg = ("限位/限速已按配置写入并回读" if not bad
                    else "写限位回读不符：" + ", ".join(bad))
        return {"ok": not bad, "rows": rows, "msg": self.msg}

    # ---------------- 清除报警锁存（不发运动/使能） ----------------
    def clear_errors(self, joint: str | None = None) -> dict:
        if not self.bus:
            raise MotionError("还没连接")
        if self.calib.active or self.drag_active or self.motion.status.get("busy"):
            raise MotionError("校准/拖动/运动进行中，先停")
        names = [joint.upper()] if joint else list(self.bus.order)
        state_changed = False
        rows = []
        for name in names:
            if name not in self.bus.motors:
                raise MotionError(f"未知关节 {name}")
            m = self.bus.motors[name]
            was_on = False
            before = None
            pos_deg = None
            try:
                was_on = m.power() == 1
                if was_on:
                    before = m.error()
            except Exception:  # noqa: BLE001
                pass
            # 失能时很多参数读不到；为了读/清报警，短暂零扭矩使能这颗
            if not was_on:
                state_changed = True
                try:
                    m.set_mode(MODE_CURRENT)
                    m.set_current_a(0.0)
                except Exception:  # noqa: BLE001
                    pass
                if not m.enable():
                    rows.append({"name": name, "cleared": False, "before": None,
                                 "before_text": None, "after": None, "after_text": None,
                                 "error": "使能无应答，无法清报警"})
                    continue
                m.set_mode(MODE_CURRENT)
                m.set_current_a(0.0)
                time.sleep(0.15)
                try:
                    before = m.error()
                except Exception:  # noqa: BLE001
                    pass
            try:
                pp = m.pos_turns()
                if pp is not None:
                    pos_deg = round(float(self.bus.cfg[name].out_deg(pp)), 2)
            except Exception:  # noqa: BLE001
                pass
            ok = False
            try:
                ok = bool(m.clear_error())
            except Exception:  # noqa: BLE001
                ok = False
            time.sleep(0.12)
            after = None
            try:
                after = m.error()
            except Exception:  # noqa: BLE001
                pass
            if not was_on:
                try:
                    m.disable()
                except Exception:  # noqa: BLE001
                    pass
            lo_s, hi_s = self.bus.cfg[name].safe_limits_deg
            rows.append({"name": name, "cleared": ok, "pos_deg": pos_deg,
                         "out_of_limit": None if pos_deg is None else
                         bool(pos_deg < lo_s - 0.5 or pos_deg > hi_s + 0.5),
                         "before": None if before is None else f"0x{before:04X}",
                         "before_text": None if before is None else describe_error(before),
                         "after": None if after is None else f"0x{after:04X}",
                         "after_text": None if after is None else describe_error(after)})
        bad = [r["name"] for r in rows if r.get("error") or r["after"] not in (None, "0x0000")]
        out_bad = [r["name"] for r in rows if r.get("out_of_limit")]
        self.msg = ("报警锁存已清除；请先手查机械，再重新使能" if not bad
                    else "清除后仍有报警：" + ", ".join(bad))
        if out_bad:
            self.msg += "；但这些关节当前位置超出安全限位，先断电手动移回：" + ", ".join(out_bad)
        if state_changed:
            self.bus.startup_done = False
            self.pending = None
            self.preview = None
            self.msg += "；有电机被短暂使能后又关了，现为未使能状态，必须重新使能"
        else:
            bad_ready = self.bus.verify_ready()
            if bad_ready:
                self.bus.startup_done = False
                self.pending = None
                self.preview = None
                self.msg += ("；但六颗未全部回到 S 曲线位置模式，已置为未使能："
                             + "，".join(f"{n}({m})" for n, m in bad_ready))
        return {"ok": not bad and not out_bad, "rows": rows, "msg": self.msg}

    # ---------------- 零扭矩拖动 / 手动测量 ----------------
    def drag_start(self, confirmed: bool = False) -> dict:
        if not confirmed:
            raise MotionError("零扭矩拖动要确认臂已扶住/支撑")
        if not self.bus or not self.motion:
            raise MotionError("还没连接")
        if self.mode == "mock":
            raise MotionError("模拟模式不能零扭矩拖动")
        if self.calib.active:
            raise MotionError("校准会话进行中")
        if not getattr(self.bus, "startup_done", False):
            raise MotionError("还没完成启动/使能，不能拖动")
        if self.motion.status.get("busy"):
            raise MotionError("正在运动，先停")
        if self.drag_active:
            return {"ok": True, "drag_active": True}
        self.pending = None
        self.preview = None
        for name in self.bus.order:
            m = self.bus.motors[name]
            try:
                m.set_mode(MODE_CURRENT)
                m.set_current_a(0.0)
            except Exception:  # noqa: BLE001
                pass
        time.sleep(0.1)
        bad = []
        for name in self.bus.order:
            m = self.bus.motors[name]
            md = m.mode()
            cur = m.current_a()
            if md is not None and md != MODE_CURRENT:
                bad.append(f"{name} 模式 {md}")
            if cur is not None and abs(cur) > 0.3:
                bad.append(f"{name} 电流 {cur:.2f} A")
        if bad:
            self.drag_stop()
            raise MotionError("切零扭矩失败：" + "；".join(bad))
        self.drag_active = True
        self.msg = "零扭矩拖动中（所有关节 0 A，臂会掉，扶住）"
        return {"ok": True, "drag_active": True, "msg": self.msg}

    def drag_stop(self) -> dict:
        if not self.bus:
            return {"ok": True, "drag_active": False}
        for name in self.bus.order:
            m = self.bus.motors[name]
            c = self.bus.cfg[name]
            try:
                pos = m.pos_turns()
                if pos is None:
                    continue
                m.set_target_turns(pos)
                m.set_mode(MODE_PROFILE_POSITION)
                self.bus._write_profile(name, c.motion_speed_deg_s)
                m.set_target_turns(pos)
            except Exception:  # noqa: BLE001
                pass
        self.drag_active = False
        self.pending = None
        self.preview = None
        bad = self.bus.verify_ready()
        if bad:
            # 再补发一次目标/模式；仍未恢复就要求重新使能
            for n, _why in bad:
                try:
                    mm = self.bus.motors[n]
                    cc = self.bus.cfg[n]
                    pp = mm.pos_turns()
                    if pp is not None:
                        mm.set_target_turns(pp)
                        mm.set_mode(MODE_PROFILE_POSITION)
                        self.bus._write_profile(n, cc.motion_speed_deg_s)
                        mm.set_target_turns(pp)
                except Exception:  # noqa: BLE001
                    pass
            time.sleep(0.1)
            bad = self.bus.verify_ready()
        if bad:
            self.bus.startup_done = False
            self.msg = ("已退出拖动，但这些关节没有恢复 S 曲线位置模式："
                        + "，".join(f"{n}({m})" for n, m in bad)
                        + "；已置为未使能，请点“重新使能（恢复 S 曲线）”")
            return {"ok": True, "drag_active": False, "warning": self.msg,
                    "bad_modes": [{"joint": n, "reason": m} for n, m in bad]}
        self.msg = "已切回位置保持"
        return {"ok": True, "drag_active": False, "msg": self.msg}

    def _write_measure_csv(self, rec: dict):
        import csv
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        new = not self._measure_path.exists()
        with self._measure_path.open("a", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(["time", "preset", "x", "y", "z", "actual_x", "actual_y",
                            "actual_z", "err_mm", "q1..q6", "note"])
            w.writerow([rec["time"], rec["preset"],
                        f"{rec['tcp'][0]:.3f}", f"{rec['tcp'][1]:.3f}", f"{rec['tcp'][2]:.3f}",
                        "" if rec["actual"] is None else f"{rec['actual'][0]:.3f}",
                        "" if rec["actual"] is None else f"{rec['actual'][1]:.3f}",
                        "" if rec["actual"] is None else f"{rec['actual'][2]:.3f}",
                        "" if rec["err_mm"] is None else f"{rec['err_mm']:.3f}",
                        " ".join(f"{v:.3f}" for v in rec["q"]), rec.get("note", "")])

    def record_measurement(self, actual=None, note: str = "") -> dict:
        if not self.motion or not self.connected:
            raise MotionError("还没连接")
        if self.motion.status.get("busy"):
            raise MotionError("正在运动，等停稳再记录")
        q = self.motion.read_q()
        tcp = self.robot.tcp(q, self.tcp_preset, self.yaw_deg)[:3, 3]
        actual_v = None
        if actual is not None:
            try:
                actual_v = [float(actual[0]), float(actual[1]), float(actual[2])]
            except Exception as exc:  # noqa: BLE001
                raise MotionError(f"实际坐标不合法：{exc}") from exc
        err = None
        if actual_v is not None:
            err = float(sum((float(tcp[i]) - actual_v[i]) ** 2 for i in range(3)) ** 0.5)
        rec = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "preset": self.tcp_preset,
            "q": [round(float(v), 4) for v in q],
            "tcp": [round(float(v), 3) for v in tcp],
            "actual": None if actual_v is None else [round(v, 3) for v in actual_v],
            "err_mm": None if err is None else round(err, 3),
            "note": note,
            "drag": bool(self.drag_active),
        }
        self.measurements.append(rec)
        if len(self.measurements) > 200:
            self.measurements = self.measurements[-200:]
        self._write_measure_csv(rec)
        self.msg = f"记录点 #{len(self.measurements)}：{rec['tcp']} mm"
        return {"ok": True, "record": rec, "measurements": self.measurements[-20:]}

    def clear_measurements(self) -> dict:
        self.measurements = []
        self.msg = "测量记录已清空（CSV 保留）"
        return {"ok": True, "measurements": []}

    def reach_tcp(self, target, axis=(0, 0, -1), free_yaw: bool = True) -> dict:
        """拖动时的快速可达性检查：0.25 s 预算，不做路径规划、不发命令。"""
        if not target or len(target) != 3:
            raise MotionError("target 需要 3 个数")
        t = [float(target[0]), float(target[1]), float(target[2])]
        rz = t[2]
        r = (t[0] ** 2 + t[1] ** 2) ** 0.5
        if rz < 20.0:
            return {"ok": True, "reachable": False, "target": t, "reason": "z < 20 mm（围栏/桌面）",
                    "pos_err_mm": None, "axis_err_deg": None}
        if rz < 200.0 and r < 90.0:
            return {"ok": True, "reachable": False, "target": t, "reason": "底座禁区 r<90 且 z<200",
                    "pos_err_mm": None, "axis_err_deg": None}
        if r > 480.0 or rz > 620.0:
            return {"ok": True, "reachable": False, "target": t, "reason": "超出粗略可达范围（r>480 或 z>620）",
                    "pos_err_mm": None, "axis_err_deg": None}
        q0 = self.planning_motion().read_q()
        try:
            res = self.robot.ik(t, axis, None if free_yaw else self.yaw_deg, q0,
                                self.tcp_preset, (self.limits[0], self.limits[1]),
                                max_iters=45, time_budget_s=0.25)
            return {"ok": True, "reachable": bool(res.ok), "target": t,
                    "pos_err_mm": round(float(res.pos_err_mm), 2),
                    "axis_err_deg": round(float(res.axis_err_deg), 2),
                    "q": [round(float(v), 2) for v in res.q_deg]}
        except Exception as exc:  # noqa: BLE001
            return {"ok": True, "reachable": False, "target": t, "reason": str(exc),
                    "pos_err_mm": None, "axis_err_deg": None}

    def reach(self, step: float = 20.0) -> dict:
        pts = []
        for x in range(-200, 601, max(5, int(step))):
            for y in range(-400, 401, max(5, int(step))):
                try:
                    res = self.robot.ik([x, y, 20.0], (0, 0, -1), 0.0,
                                        [0, 0, 0, 0, 0, 0], "probe", self.limits,
                                        structured_seeds=False, max_iters=60)
                    ok = res.ok and self.fence.check(res.q_deg, "probe", 0.0,
                                                     False, self.limits).ok
                except Exception:  # noqa: BLE001
                    ok = False
                pts.append([x, y, 1 if ok else 0])
        return {"ok": True, "step": step, "z": 20.0, "points": pts}

    # ---------------- 主循环 ----------------
    def start_watchdog(self):
        self._watch_stop.clear()
        self._watch_thread = threading.Thread(target=self._watchdog, daemon=True)
        self._watch_thread.start()


def brief_segments(segs):
    return [{k: s.get(k) for k in ("label", "q_from", "q_target", "descend", "T",
                                   "fence", "near_limit", "ik")}
            for s in segs]


def preview_payload(seg, robot, fence, preset, yaw):
    q = seg["q_target"]
    return {
        "segments": brief_segments([seg]),
        "scene": fence.scene(q, preset, yaw),
        "q": q,
        "tcp": robot.tcp(q, preset, yaw)[:3, 3].tolist(),
        "fence": seg["fence"],
        "ik": seg.get("ik"),
        "target": None,
        "yaw": yaw,
        "near_limit": seg.get("near_limit", []),
    }


EXEC: Executor | None = None


class Handler(BaseHTTPRequestHandler):
    server_version = "ArmExecutor/1.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    # ---------------- 响应 ----------------
    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, code: int, obj):
        body = json.dumps(obj, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        raw = self.rfile.read(n)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:  # noqa: BLE001
            return {}

    def _static(self, path: str):
        if not EXEC.serve_web:
            return self._json(404, {"ok": False, "error": "web server disabled"})
        rel = path.lstrip("/") or "index.html"
        base = EXEC.web_dir.resolve()
        fp = (base / rel).resolve()
        if not str(fp).startswith(str(base)) or not fp.is_file():
            return self._json(404, {"ok": False, "error": "not found"})
        data = fp.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", MIME.get(fp.suffix, "application/octet-stream"))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _events(self):
        qs = parse_qs(urlparse(self.path).query)
        client_id = (qs.get("client_id") or [""])[0]
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self._cors()
        self.end_headers()
        period = 1.0 / EXEC.push_hz
        try:
            while True:
                data = json.dumps(EXEC.state(), ensure_ascii=False, allow_nan=False)
                self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                self.wfile.flush()
                EXEC.touch(client_id)
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return
        except Exception:  # noqa: BLE001
            traceback.print_exc()
            return

    # ---------------- GET ----------------
    def do_GET(self):
        p = urlparse(self.path).path
        try:
            if p == "/api/state":
                qs = parse_qs(urlparse(self.path).query)
                cid = (qs.get("client_id") or [""])[0]
                EXEC.touch(cid)
                return self._json(200, EXEC.state())
            if p == "/api/events":
                return self._events()
            if p == "/api/config":
                return self._json(200, {"ok": True, **EXEC.config_payload()})
            if p == "/api/debug/page_state":
                return self._json(200, EXEC.load_web_state())
            if p == "/api/kt/points":
                return self._json(200, EXEC.kt_points())
            if p == "/api/ping":
                return self._json(200, {"ok": True, "t": time.time()})
            return self._static(p)
        except Exception as exc:  # noqa: BLE001
            return self._json(500, {"ok": False, "error": str(exc)})

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    # ---------------- POST ----------------
    def do_POST(self):
        p = urlparse(self.path).path
        body = self._body()
        try:
            # 控制权 / 心跳
            if p == "/api/acquire":
                r = EXEC.acquire(str(body.get("client_id") or ""),
                                 str(body.get("config_version") or ""))
                return self._json(200, {"ok": True, **r})
            if p == "/api/heartbeat":
                r = EXEC.heartbeat(str(body.get("token") or ""))
                return self._json(200, r)
            if p == "/api/release":
                return self._json(200, EXEC.release(str(body.get("token") or "")))
            if p == "/api/stop":
                # 停是安全动作：不要求控制权，任何客户端都能停
                if EXEC.calib.active:
                    EXEC.calib.abort(reason="用户按停")
                elif EXEC.drag_active:
                    EXEC.drag_stop()
                elif EXEC.motion:
                    EXEC.motion.stop()
                return self._json(200, {"ok": True})
            if p == "/api/debug/page_state":
                # 调试快照：不要求控制权，任何页面（包括只读）都可写；只写最新一份
                return self._json(200, EXEC.save_web_state(body))
            if p == "/api/reach_tcp":
                return self._json(200, EXEC.reach_tcp(
                    body.get("target"), body.get("axis", (0, 0, -1)),
                    bool(body.get("free_yaw", True))))
            if p == "/api/clear_errors":
                EXEC.require_control(body)
                return self._json(200, EXEC.clear_errors(body.get("joint")))
            if p == "/api/limits/write":
                EXEC.require_control(body)
                if not body.get("confirmed"):
                    raise MotionError("写限位要二次确认")
                return self._json(200, EXEC.write_limits_all())
            if p == "/api/drag/start":
                EXEC.require_control(body)
                return self._json(200, EXEC.drag_start(bool(body.get("confirmed"))))
            if p == "/api/drag/stop":
                EXEC.require_control(body)
                return self._json(200, EXEC.drag_stop())
            if p == "/api/measure/record":
                EXEC.require_control(body)
                return self._json(200, EXEC.record_measurement(
                    body.get("actual"), str(body.get("note") or "")))
            if p == "/api/measure/clear":
                EXEC.require_control(body)
                return self._json(200, EXEC.clear_measurements())
            if p == "/api/measure":
                return self._json(200, {"ok": True, "measurements": EXEC.measurements[-50:],
                                        "preset": EXEC.tcp_preset})
            # ---- 16.3 校准会话（只认控制权，不比对配置版本：校准本身会改配置） ----
            if p == "/api/calib/prepare":
                EXEC.require_token(body)
                EXEC._calib_active = True      # 准备期间先停掉普通轮询，别跟它抢 ECB
                try:
                    r = EXEC.calib.prepare(str(body.get("joint") or ""),
                                           bool(body.get("confirmed_supported")))
                except Exception:
                    EXEC._calib_active = False
                    raise
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/set_zero":
                EXEC.require_token(body)
                r = EXEC.calib.set_zero(str(body.get("joint") or ""))
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/jog":
                EXEC.require_token(body)
                r = EXEC.calib.jog(float(body.get("delta", 5.0)))
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/confirm_sign":
                EXEC.require_token(body)
                r = EXEC.calib.confirm_sign(int(body.get("sign", 1)))
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/return":
                EXEC.require_token(body)
                r = EXEC.calib.return_zero()
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/calib/abort":
                EXEC.require_token(body)
                r = EXEC.calib.abort(reason=str(body.get("reason") or "用户中止校准"))
                return self._json(200, {"ok": True, "calib": r})
            if p == "/api/connect":
                EXEC.require_control(body)
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("正在运动/预演，先按停再重连")
                if EXEC.drag_active:
                    raise MotionError("零扭矩拖动中，先锁定保持")
                r = EXEC.connect(body.get("mode", "mock"),
                                 body.get("ip") or EXEC.ecb_ip,
                                 int(body.get("port") or EXEC.ecb_port))
                return self._json(200, r)
            if p == "/api/enable":
                EXEC.require_control(body)
                if not EXEC.bus:
                    raise MotionError("还没连接")
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("正在运动/预演，先按停")
                if EXEC.drag_active:
                    raise MotionError("零扭矩拖动中，先锁定保持")
                rep = EXEC.bus.enable_all(bool(body.get("confirmed_supported")))
                EXEC.pending = None
                EXEC.preview = None
                EXEC.msg = "已使能（J6→J1），S 曲线模式就绪"
                check = {"has_parked": False, "ok": None}
                try:
                    q_now = EXEC.motion.read_q()
                    check = EXEC._park_check(q_now)
                    if check.get("has_parked") and check.get("ok") is False:
                        EXEC.park_block = {"max_diff_deg": check.get("max_diff_deg"),
                                           "diffs": check.get("diffs"),
                                           "ts": check.get("ts")}
                        EXEC.msg = (f"启停比对不过：和 parked_q 差 "
                                    f"{check.get('max_diff_deg')}°（{check.get('diffs')}）。"
                                    "已拒绝一切运动；看过实物后点“已确认”解除。")
                    elif check.get("has_parked") and check.get("ok"):
                        EXEC.park_block = None
                        EXEC.msg = ("臂在停机姿态（搁架上）。先点“抬回预停机点”离开搁架，"
                                    "再回零或做别的。")
                except Exception as exc:  # noqa: BLE001
                    EXEC.msg = f"已使能，但读姿态失败：{exc}"
                return self._json(200, {"ok": True, "report": rep, "msg": EXEC.msg,
                                        "park_check": check})
            if p == "/api/plan_click":
                EXEC.require_control(body)
                return self._json(200, EXEC.plan_click(
                    float(body["x"]), float(body["y"]),
                    float(body.get("z", 20.0)), body.get("yaw")))
            if p == "/api/click":
                EXEC.require_control(body)
                return self._json(200, EXEC.click(
                    float(body["x"]), float(body["y"]),
                    float(body.get("z", 20.0)), body.get("yaw")))
            if p == "/api/execute":
                EXEC.require_control(body)
                return self._json(200, EXEC.execute(body.get("preview")))
            if p == "/api/jog":
                EXEC.require_control(body)
                return self._json(200, EXEC.jog(body["joint"],
                                                float(body.get("delta", 1.0)),
                                                bool(body.get("preview", False))))
            if p == "/api/home":
                EXEC.require_control(body)
                return self._json(200, EXEC.home(bool(body.get("preview", False))))
            if p == "/api/move_joints":
                EXEC.require_control(body)
                return self._json(200, EXEC.move_joints(
                    body["q"], bool(body.get("preview", False)),
                    bool(body.get("descend", False))))
            if p == "/api/move_tcp":
                EXEC.require_control(body)
                return self._json(200, EXEC.move_tcp(
                    body["target"], body.get("axis", (0, 0, -1)),
                    body.get("yaw"), bool(body.get("preview", False)),
                    bool(body.get("descend", False)), body.get("preset"),
                    bool(body.get("free_yaw", False))))
            if p == "/api/move_through":
                EXEC.require_control(body)
                return self._json(200, EXEC.move_through(
                    body["waypoints"], body.get("descend_flags"),
                    body.get("labels"), bool(body.get("preview", False))))
            if p == "/api/disable_all":
                EXEC.require_control(body)
                if not EXEC.bus:
                    raise MotionError("还没连接")
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("运动/预演中不能全部失能；先按停")
                if EXEC.drag_active:
                    raise MotionError("零扭矩拖动中，先锁定保持")
                if not body.get("confirmed"):
                    raise MotionError("全部失能要二次确认")
                ok = EXEC.bus.disable_all(True, body.get("reason", "user"))
                EXEC.pending = None
                EXEC.preview = None
                EXEC.msg = "已全部失能"
                return self._json(200, {"ok": bool(ok)})
            if p == "/api/end_effector":
                EXEC.require_control(body)
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("运动/预演中不能换末端配置")
                return self._json(200, EXEC.set_end_effector(
                    str(body.get("end_effector") or ""), body.get("payload_kg")))
            if p == "/api/park/teach":
                EXEC.require_control(body)
                return self._json(200, EXEC.teach_park())
            if p == "/api/park":
                EXEC.require_control(body)
                return self._json(200, EXEC.park(
                    bool(body.get("preview", False)), body.get("prereq")))
            if p == "/api/park/ack":
                EXEC.require_control(body)
                return self._json(200, EXEC.park_ack())
            if p == "/api/park/unpark":
                EXEC.require_control(body)
                return self._json(200, EXEC.unpark(
                    bool(body.get("preview", False)), bool(body.get("confirmed"))))
            if p == "/api/recover":
                EXEC.require_control(body)
                return self._json(200, EXEC.recover())
            if p == "/api/park/support":
                EXEC.require_control(body)
                return self._json(200, EXEC.support_park(bool(body.get("confirmed"))))
            if p == "/api/debug/page_state":
                return self._json(200, EXEC.load_web_state())
            if p == "/api/kt/points":
                return self._json(200, EXEC.kt_points())
            if p == "/api/kt/goto":
                EXEC.require_control(body)
                return self._json(200, EXEC.kt_goto(body["q"],
                                                    bool(body.get("preview", False))))
            if p == "/api/kt/sample":
                EXEC.require_control(body)
                return self._json(200, EXEC.kt_sample(
                    body.get("joints"), float(body.get("load_kg", 0.0)),
                    str(body.get("label") or "")))
            if p == "/api/kt/fit":
                EXEC.require_control(body)
                return self._json(200, EXEC.kt_fit(bool(body.get("save", True))))
            if p == "/api/kt/reset":
                EXEC.require_control(body)
                return self._json(200, EXEC.kt_reset())
            if p == "/api/tcp":
                EXEC.require_control(body)
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("运动/预演中不能换 TCP 预设")
                name = body.get("preset", "probe")
                if name not in EXEC.robot.tcp_presets:
                    raise MotionError(f"未知 TCP {name}")
                EXEC.tcp_preset = name
                pr = (EXEC.robot.tcp_presets or {}).get(name) or {}
                if pr.get("yaw_deg") is not None:
                    EXEC.yaw_deg = float(pr["yaw_deg"])
                    if EXEC.motion:
                        EXEC.motion.yaw_deg = EXEC.yaw_deg
                    EXEC.preview_motion.yaw_deg = EXEC.yaw_deg
                if EXEC.motion:
                    EXEC.motion.preset = name
                EXEC.preview_motion.preset = name
                EXEC.preview = None
                EXEC.pending = None
                return self._json(200, {"ok": True, "preset": name,
                                        "yaw": EXEC.yaw_deg})
            if p == "/api/yaw":
                EXEC.require_control(body)
                if EXEC.motion and EXEC.motion.status.get("busy"):
                    raise MotionError("运动/预演中不能改偏航")
                EXEC.yaw_deg = float(body.get("yaw", 0.0))
                if EXEC.motion:
                    EXEC.motion.yaw_deg = EXEC.yaw_deg
                EXEC.preview_motion.yaw_deg = EXEC.yaw_deg
                return self._json(200, {"ok": True, "yaw": EXEC.yaw_deg})
            if p == "/api/exec_mode":
                EXEC.require_control(body)
                mode = body.get("mode", "preview")
                if mode not in ("preview", "direct"):
                    raise MotionError("mode 只能是 preview 或 direct")
                EXEC.exec_mode = mode
                EXEC.msg = f"执行模式：{'直接执行' if mode == 'direct' else '预演'}"
                return self._json(200, {"ok": True, "exec_mode": mode})
            if p == "/api/reach":
                EXEC.require_control(body)
                return self._json(200, EXEC.reach(float(body.get("step", 20.0))))
            if p == "/api/msg":
                EXEC.msg = str(body.get("msg", ""))
                return self._json(200, {"ok": True})
            return self._json(404, {"ok": False, "error": "no such api"})
        except AuthError as exc:
            return self._json(409, {"ok": False, "error": str(exc)})
        except (MotionError, StartupRefused) as exc:
            EXEC.msg = str(exc)
            return self._json(400, {"ok": False, "error": str(exc)})
        except KeyError as exc:
            return self._json(400, {"ok": False, "error": f"缺字段 {exc}"})
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            return self._json(500, {"ok": False, "error": str(exc)})


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main(argv=None) -> int:
    global EXEC
    ap = argparse.ArgumentParser(description="阶段一 A 执行器（独立后台程序）")
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--ecb-ip", default=None)
    ap.add_argument("--ecb-port", type=int, default=None)
    ap.add_argument("--mock", action="store_true", help="启动即连模拟臂")
    ap.add_argument("--open", action="store_true", help="打开浏览器")
    ap.add_argument("--no-web", action="store_true", help="只提供 API，不服务网页")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args(argv)

    cfg = cfgmod.load_executor()
    if args.host:
        cfg["bind_host"] = args.host
    if args.port:
        cfg["bind_port"] = args.port
    if args.ecb_ip:
        cfg["ecb_ip"] = args.ecb_ip
    if args.ecb_port:
        cfg["ecb_udp_port"] = args.ecb_port
    if args.no_web:
        cfg["serve_web"] = False
    EXEC = Executor(cfg)
    EXEC.start_watchdog()
    EXEC.start_poll()
    if args.mock or cfg.get("mock_on_start"):
        EXEC.connect("mock")
    srv = Server((EXEC.bind_host, EXEC.bind_port), Handler)
    url = f"http://{EXEC.bind_host}:{EXEC.bind_port}/"
    print(f"执行器 {url}  config_version={EXEC.config_version}  mode={EXEC.mode}  {EXEC.msg}")
    if args.open:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        EXEC.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
