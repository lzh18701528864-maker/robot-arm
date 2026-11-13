#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工程路径与配置读取。"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = ROOT / "config"
URDF = ROOT / "urdf" / "gluon_2l6_4l3.urdf"
WEB_DIR = ROOT / "web"
RUNS_DIR = ROOT / "runs"
MODELS_DIR = ROOT / "models"


def load_json(path: Path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {} if default is None else default


def load_joints() -> dict:
    return load_json(CONFIG_DIR / "joints.json", {})


def load_tcp() -> dict:
    return load_json(CONFIG_DIR / "tcp.json", {})


def load_executor() -> dict:
    return load_json(CONFIG_DIR / "executor.json", {})


def load_park() -> dict:
    return load_json(CONFIG_DIR / "park.json", {})


def load_park_state() -> dict:
    return load_json(CONFIG_DIR / "park_state.json", {})


def save_park_state(data: dict) -> None:
    (CONFIG_DIR / "park_state.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def save_park(data: dict) -> None:
    """原子写 config/park.json（记录教的停机位置）。"""
    path = CONFIG_DIR / "park.json"
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    os.replace(tmp, path)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:12]


def config_version() -> str:
    """关节配置 + TCP 配置 + URDF 的联合版本。任何一个字节变了都会变。"""
    return f"{_sha(CONFIG_DIR / 'joints.json')}-{_sha(CONFIG_DIR / 'tcp.json')}-{_sha(URDF)}"


def config_versions() -> dict:
    return {
        "config_version": config_version(),
        "joints_sha": _sha(CONFIG_DIR / "joints.json"),
        "tcp_sha": _sha(CONFIG_DIR / "tcp.json"),
        "urdf_sha": _sha(URDF),
        "joints_version": load_joints().get("version", ""),
        "tcp_version": load_tcp().get("version", ""),
    }


def tcp_presets() -> dict:
    return load_tcp().get("presets", {})


def soft_limits() -> tuple[list[float], list[float]]:
    """按 joint_order 返回软件限位（deg）。"""
    cfg = load_joints()
    order = cfg.get("joint_order", ["J1", "J2", "J3", "J4", "J5", "J6"])
    lo, hi = [], []
    for name in order:
        lim = cfg["joints"][name]["soft_limits_deg"]
        lo.append(float(lim[0]))
        hi.append(float(lim[1]))
    return lo, hi


def safe_limits() -> tuple[list[float], list[float]]:
    """安全限位（比软件限位再收 2°），运动/启动实际用这一层。"""
    cfg = load_joints()
    order = cfg.get("joint_order", ["J1", "J2", "J3", "J4", "J5", "J6"])
    lo, hi = [], []
    for name in order:
        j = cfg["joints"][name]
        lim = j.get("safe_limits_deg", j["soft_limits_deg"])
        lo.append(float(lim[0]))
        hi.append(float(lim[1]))
    return lo, hi
