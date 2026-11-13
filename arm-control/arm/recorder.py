#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CSV 记录（原 B7）：每次运行一个文件。

两类行：
  * command_*：命令（含预演计划，kind=plan，不发写命令）
  * poll：每次轮询（时间、关节、位置、电流、报警）
"""
from __future__ import annotations

import csv
import datetime as _dt
import threading
from pathlib import Path

from . import config


class Recorder:
    def __init__(self, path: Path | None = None, note: str = ""):
        config.RUNS_DIR.mkdir(parents=True, exist_ok=True)
        if path is None:
            stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = config.RUNS_DIR / f"{stamp}_run.csv"
        self.path = Path(path)
        self._lock = threading.Lock()
        self._fh = self.path.open("w", newline="", encoding="utf-8")
        self._w = csv.writer(self._fh)
        self._w.writerow(["time", "t_rel", "kind", "joint", "pos_deg",
                          "target_deg", "current_a", "error", "note"])
        self._t0 = _dt.datetime.now()
        if note:
            self.note(note)
        self._fh.flush()

    def _ts(self):
        now = _dt.datetime.now()
        return now.isoformat(timespec="milliseconds"), round((now - self._t0).total_seconds(), 3)

    def note(self, text: str):
        with self._lock:
            t, tr = self._ts()
            self._w.writerow([t, tr, "note", "", "", "", "", "", text])
            self._fh.flush()

    def command(self, kind: str, note: str = "", joint: str = "",
                target_deg: float | None = None, **kw):
        with self._lock:
            t, tr = self._ts()
            self._w.writerow([t, tr, kind, joint,
                              "" if kw.get("pos_deg") is None else f"{kw['pos_deg']:.4f}",
                              "" if target_deg is None else f"{target_deg:.4f}",
                              kw.get("current_a", ""), kw.get("error", ""), note])
            self._fh.flush()

    def poll(self, states: dict):
        with self._lock:
            t, tr = self._ts()
            for name, st in states.items():
                self._w.writerow([
                    t, tr, "poll", name,
                    "" if st.out_deg is None else f"{st.out_deg:.4f}",
                    "" if st.target_deg is None else f"{st.target_deg:.4f}",
                    "" if st.current_a is None else f"{st.current_a:.3f}",
                    "" if st.error is None else f"0x{st.error:04X}",
                    "",
                ])
            self._fh.flush()

    def close(self):
        try:
            self._fh.close()
        except Exception:  # noqa: BLE001
            pass
