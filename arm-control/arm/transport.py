#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ECB UDP 传输层：帧、CRC、握手、枚举、请求/应答、重试。

ECB 只是网口→CAN 网桥，线上跑的还是 SCA 指令。帧格式沿用 ecb-test/ecb_test.py
（已在实机上验证）：
    EE | addr | cmd | len_hi len_lo | data | crc16(data) 低字节在前 | ED
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass

HEAD = 0xEE
TAIL = 0xED
DEFAULT_IP = "192.168.1.30"
DEFAULT_PORT = 2000


class EcbError(RuntimeError):
    pass


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def make_frame(addr: int, cmd: int, data: bytes = b"") -> bytes:
    if not 0 <= addr <= 0xFF:
        raise ValueError("地址超出 0x00~0xFF")
    body = bytes([HEAD, addr, cmd]) + len(data).to_bytes(2, "big") + data
    if data:
        body += struct.pack("<H", crc16(data))
    return body + bytes([TAIL])


@dataclass
class Frame:
    addr: int
    cmd: int
    data: bytes
    raw: bytes
    crc: bytes | None = None

    @property
    def crc_ok(self) -> bool | None:
        if self.crc is None:
            return None
        return struct.pack("<H", crc16(self.data)) == self.crc


def parse_frames(buf: bytes) -> list[Frame]:
    out: list[Frame] = []
    i = 0
    while i + 6 <= len(buf):
        if buf[i] != HEAD:
            i += 1
            continue
        n = int.from_bytes(buf[i + 3:i + 5], "big")
        j = i + 5 + n
        data = buf[i + 5:j]
        crc_frame = j + 3 <= len(buf) and buf[j + 2] == TAIL
        no_crc = j + 1 <= len(buf) and buf[j] == TAIL
        if crc_frame and buf[j:j + 2] == struct.pack("<H", crc16(data)):
            out.append(Frame(buf[i + 1], buf[i + 2], data, buf[i:j + 3], buf[j:j + 2]))
            i = j + 3
        elif no_crc:
            out.append(Frame(buf[i + 1], buf[i + 2], data, buf[i:j + 1], None))
            i = j + 1
        elif crc_frame:
            out.append(Frame(buf[i + 1], buf[i + 2], data, buf[i:j + 3], buf[j:j + 2]))
            i = j + 3
        else:
            i += 1
    return out


class EcbTransport:
    """同步请求/应答 + 重试。线程安全（一把锁串行化请求）。"""

    def __init__(self, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
                 timeout: float = 0.25, retries: int = 3, debug: bool = False):
        self.ip = ip
        self.port = int(port)
        self.timeout = float(timeout)
        self.retries = int(retries)
        self.debug = debug
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("", 0))
        self._lock = threading.RLock()
        self.tx = 0
        self.rx = 0
        self.tx_fail = 0
        self.timeouts = 0
        self._buf: list[Frame] = []
        self.last_rx_time: float | None = None

    def local_ip(self) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((self.ip, self.port))
            return s.getsockname()[0]
        except OSError:
            return "?"
        finally:
            s.close()

    # ---------------- 底层 ----------------
    def _send(self, raw: bytes) -> None:
        try:
            self._sock.sendto(raw, (self.ip, self.port))
        except OSError as exc:
            self.tx_fail += 1
            raise EcbError(f"UDP 发不出去：{exc}") from exc
        self.tx += 1

    def _collect(self, seconds: float) -> list[Frame]:
        frames: list[Frame] = []
        end = time.monotonic() + seconds
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            self._sock.settimeout(min(0.02, left))
            try:
                data, _ = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            got = parse_frames(data)
            self.rx += len(got)
            self.last_rx_time = time.monotonic()
            frames.extend(got)
        self._buf.extend(frames)
        return frames

    def request(self, addr: int, cmd: int, data: bytes = b"",
                timeout: float | None = None, retries: int | None = None) -> Frame | None:
        timeout = self.timeout if timeout is None else timeout
        retries = self.retries if retries is None else retries
        raw = make_frame(addr, cmd, data)
        with self._lock:
            for _ in range(max(1, retries)):
                self._send(raw)
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    for f in self._collect(min(0.02, end - time.monotonic())):
                        # ECB 握手应答的 CRC 两字节是 00 00（不参与校验），
                        # 其它指令仍跳过 CRC 不符的帧。
                        if f.crc_ok is False and not (f.addr == 0 or f.cmd == 0x44):
                            continue
                        if f.addr == addr and f.cmd == cmd:
                            # 清掉同批里的旧帧
                            return f
                self.timeouts += 1
        return None

    def send_cmd(self, addr: int, cmd: int, data: bytes = b"") -> None:
        """只发不等回（有些固件的使能/失能应答不可靠，但状态会变）。"""
        with self._lock:
            self._send(make_frame(addr, cmd, data))

    def read_many(self, items, timeout: float = 0.05,
                  retries: int = 2) -> dict[tuple[int, int], bytes]:
        """一次发多颗的读请求，再收集回包（items=[(addr,cmd,n)]）。

        单条请求的固件/网桥回包约 20 ms；串行 6 颗要 130 ms。
        并发发 6 条通常 10~20 ms 收齐，丢的再补发一轮。
        """
        results: dict[tuple[int, int], bytes] = {}
        pending = list(items)
        with self._lock:
            for _ in range(max(1, retries)):
                if not pending:
                    break
                for addr, cmd, _n in pending:
                    self._send(make_frame(addr, cmd))
                want = {(a, c): n for a, c, n in pending}
                end = time.monotonic() + timeout
                while time.monotonic() < end and len(results) < len(want):
                    for f in self._collect(min(0.01, end - time.monotonic())):
                        key = (f.addr, f.cmd)
                        if key in want and len(f.data) >= want[key] and key not in results:
                            results[key] = f.data[:want[key]]
                pending = [it for it in pending if (it[0], it[1]) not in results]
                if pending:
                    self.timeouts += len(pending)
        return results

    def read(self, addr: int, cmd: int, n: int, **kw) -> bytes | None:
        f = self.request(addr, cmd, b"", **kw)
        if f is None or len(f.data) < n:
            return None
        return f.data[:n]

    def write(self, addr: int, cmd: int, data: bytes = b"", timeout: float = 0.5,
              retries: int | None = None) -> bool:
        f = self.request(addr, cmd, data, timeout=timeout, retries=retries)
        return bool(f is not None and f.data and f.data[0] == 1)

    # ---------------- 高层 ----------------
    def hello(self, timeout: float = 0.8) -> bool:
        f = self.request(0x00, 0x44, timeout=timeout, retries=2)
        return bool(f is not None and f.cmd == 0x44
                    and (not f.data or f.data[0] == 1))

    def scan(self, wait: float = 2.0) -> dict[int, int]:
        with self._lock:
            self._send(make_frame(0x00, 0x02))
            frames = self._collect(wait)
        out: dict[int, int] = {}
        for f in frames:
            if f.addr > 0 and f.cmd == 0x02 and len(f.data) >= 4:
                out[f.addr] = int.from_bytes(f.data[:4], "big")
        return out

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:  # noqa: BLE001
            pass

    @property
    def stats(self) -> dict:
        return {"tx": self.tx, "rx": self.rx, "tx_fail": self.tx_fail,
                "timeouts": self.timeouts,
                "last_rx_age_s": (None if self.last_rx_time is None
                                  else round(time.monotonic() - self.last_rx_time, 3))}
