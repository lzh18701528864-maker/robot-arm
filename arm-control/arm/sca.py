#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SCA / QDD 单关节协议（ECB 网口）。

关键换算（SDK `SCA_Protocol.c` / `SCA_API.c` 核对）：
  * 位置 0x06/0x0A：电机侧圈数（R），_IQ24；输出端 1 R = 360/36 = 10°
  * 电流 0x04/0x59：满量程比例，_IQ24；满量程 = 0x53（A，_IQ8）
  * 速度 0x05/0x5B：满量程 6000 RPM，_IQ24
  * 位置梯形 0x1C..0x1E / 0x1F..0x21：真实值 = raw/2^24 × 960，
    单位 RPM / RPM·s⁻¹（Profile_Scal = 2^4×60 = 960）
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from .transport import EcbTransport, Frame

IQ24 = 1 << 24
IQ8 = 1 << 8
IQ10 = 1 << 10
GEAR = 36.0
DEG_PER_MOTOR_TURN = 360.0 / GEAR
VELOCITY_FULL_RPM = 6000.0
PROFILE_SCALE = 960.0
BATTERY_SCALE = 100.0

# 读
R_POSITION = 0x06
R_VELOCITY = 0x05
R_CURRENT = 0x04
R_VOLTAGE = 0x45
R_CURRENT_MAX = 0x53
R_TEMP_MOTOR = 0x5F
R_TEMP_INV = 0x60
R_ERROR = 0xFF
R_POWER = 0x2B
R_MODE = 0x55
R_CURRENT_LIMIT = 0x59
R_VEL_LIMIT = 0x5B
R_PROFILE_VEL = 0x1C
R_PROFILE_ACC = 0x1D
R_PROFILE_DEC = 0x1E
R_POS_LIMIT_H = 0x85
R_POS_LIMIT_L = 0x86
R_POS_OFFSET = 0x8A
R_BATTERY = 0xB7
R_CVP = 0x94
R_SN = 0x02
R_CLEAR_HOME = 0x88  # 写

# 写
W_ENABLE = 0x2A
W_MODE = 0x07
W_TARGET = 0x0A
W_CLEAR_ERROR = 0xFE
W_SET_CURRENT_LIMIT = 0x58
W_SET_VEL_LIMIT = 0x5A
W_SET_PROFILE_VEL = 0x1F
W_SET_PROFILE_ACC = 0x20
W_SET_PROFILE_DEC = 0x21
W_SET_POS_LIMIT_H = 0x83
W_SET_POS_LIMIT_L = 0x84
W_SET_HOME = 0x87
W_SET_POS_OFFSET = 0x89
W_CLEAR_HOME = 0x88
W_SAVE = 0x0D
W_HANDSHAKE = 0x00

MODE_CURRENT = 0x01
MODE_VELOCITY = 0x02
MODE_POSITION = 0x03
MODE_PROFILE_POSITION = 0x06
MODE_PROFILE_VELOCITY = 0x07
MODE_HOMING = 0x08

ERROR_BITS = [
    (0x0001, "过压"), (0x0002, "欠压"), (0x0004, "堵转"), (0x0008, "过温"),
    (0x0010, "读写参数异常"), (0x0020, "多圈计数异常"),
    (0x0040, "逆变器温度传感器异常"), (0x0080, "CAN 通信异常"),
    (0x0100, "电机温度传感器异常"), (0x0200, "位置模式阶跃过大"),
    (0x0400, "驱动保护"), (0x0800, "编码器故障"),
]
FATAL_BITS = {0x0004, 0x0008, 0x0020}


def describe_error(code: int) -> str:
    if not code:
        return "正常"
    names = [n for b, n in ERROR_BITS if code & b]
    return " / ".join(names) if names else f"未知 0x{code:04X}"


def turns_to_out_deg(turns: float) -> float:
    return turns * DEG_PER_MOTOR_TURN


def out_deg_to_turns(deg: float) -> float:
    return deg / DEG_PER_MOTOR_TURN


@dataclass
class MotorTelemetry:
    enabled: bool | None = None
    pos_turns: float | None = None
    out_deg: float | None = None
    velocity_rpm: float | None = None
    current_a: float | None = None
    voltage_v: float | None = None
    temp_c: float | None = None
    error: int | None = None
    mode: int | None = None
    current_max_a: float | None = None
    current_limit_a: float | None = None
    speed_limit_rpm: float | None = None
    pos_limit_lo_r: float | None = None
    pos_limit_hi_r: float | None = None
    profile_rpm: tuple | None = None


class ScaMotor:
    def __init__(self, link: EcbTransport, mid: int, gear: float = GEAR):
        self.link = link
        self.mid = int(mid)
        self.gear = float(gear)
        self.last_reply: float | None = None
        self._current_max: float | None = None

    # ---------------- 读 ----------------
    def _r(self, cmd: int, n: int, timeout: float = 0.25, retries: int | None = None) -> bytes | None:
        d = self.link.read(self.mid, cmd, n, timeout=timeout, retries=retries)
        if d is not None:
            import time
            self.last_reply = time.monotonic()
        return d

    def sn(self) -> int | None:
        d = self._r(R_SN, 4)
        return int.from_bytes(d, "big") if d else None

    def power(self, timeout: float = 0.25, retries: int | None = None) -> int | None:
        d = self._r(R_POWER, 1, timeout=timeout, retries=retries)
        return d[0] if d else None

    def pos_turns(self, timeout: float = 0.25, retries: int | None = None) -> float | None:
        d = self._r(R_POSITION, 4, timeout=timeout, retries=retries)
        return struct.unpack(">i", d)[0] / IQ24 if d else None

    def velocity_rpm(self) -> float | None:
        d = self._r(R_VELOCITY, 4)
        return struct.unpack(">i", d)[0] / IQ24 * VELOCITY_FULL_RPM if d else None

    def current_max_a(self, timeout: float = 0.25, retries: int | None = None) -> float | None:
        d = self._r(R_CURRENT_MAX, 2, timeout=timeout, retries=retries)
        if d:
            self._current_max = struct.unpack(">h", d)[0] / IQ8
        return self._current_max

    def current_a(self, timeout: float = 0.25, retries: int | None = None) -> float | None:
        cm = self.current_max_a(timeout=timeout, retries=retries)
        d = self._r(R_CURRENT, 4, timeout=timeout, retries=retries)
        if not d or not cm:
            return None
        return struct.unpack(">i", d)[0] / IQ24 * cm

    def voltage_v(self) -> float | None:
        d = self._r(R_VOLTAGE, 2)
        return struct.unpack(">H", d)[0] / IQ10 if d else None

    def battery_v(self) -> float | None:
        """0xB7：×100。注意这多半是 430 供电，不是多圈备份电池。"""
        d = self._r(R_BATTERY, 2)
        return struct.unpack(">H", d)[0] / BATTERY_SCALE if d else None

    def temp_c(self) -> float | None:
        d = self._r(R_TEMP_MOTOR, 2)
        return struct.unpack(">h", d)[0] / 256.0 if d else None

    def error(self, timeout: float = 0.25, retries: int | None = None) -> int | None:
        d = self._r(R_ERROR, 2, timeout=timeout, retries=retries)
        return struct.unpack(">H", d)[0] if d else None

    def mode(self, timeout: float = 0.25, retries: int | None = None) -> int | None:
        d = self._r(R_MODE, 1, timeout=timeout, retries=retries)
        return d[0] if d else None

    def current_raw_counts(self, timeout: float = 0.25):
        """18.4：0x04 和 0x59 的原始 24 位计数（P08-17），Lite 量程以后重算用。"""
        import struct as _struct
        d4 = self._r(R_CURRENT, 4, timeout=timeout)
        d59 = self._r(R_CURRENT_LIMIT, 4, timeout=timeout)
        return (None if not d4 else _struct.unpack(">i", d4)[0],
                None if not d59 else _struct.unpack(">i", d59)[0])

    def current_limit_a(self) -> float | None:
        cm = self.current_max_a()
        d = self._r(R_CURRENT_LIMIT, 4)
        if not d or not cm:
            return None
        return struct.unpack(">i", d)[0] / IQ24 * cm

    def speed_limit_rpm(self) -> float | None:
        d = self._r(R_VEL_LIMIT, 4)
        return struct.unpack(">i", d)[0] / IQ24 * VELOCITY_FULL_RPM if d else None

    def pos_limits_turns(self) -> tuple[float, float] | None:
        hi = self._r(R_POS_LIMIT_H, 4)
        lo = self._r(R_POS_LIMIT_L, 4)
        if not hi or not lo:
            return None
        return (struct.unpack(">i", lo)[0] / IQ24,
                struct.unpack(">i", hi)[0] / IQ24)

    def pos_offset_turns(self) -> float | None:
        d = self._r(R_POS_OFFSET, 4)
        return struct.unpack(">i", d)[0] / IQ24 if d else None

    def profile(self) -> tuple[float, float, float] | None:
        vals = []
        for cmd in (R_PROFILE_VEL, R_PROFILE_ACC, R_PROFILE_DEC):
            d = self._r(cmd, 4)
            if not d:
                return None
            vals.append(struct.unpack(">i", d)[0] / IQ24 * PROFILE_SCALE)
        return vals[0], vals[1], vals[2]

    # ---------------- 写 ----------------
    def _w(self, cmd: int, data: bytes = b"", timeout: float = 0.5) -> bool:
        ok = self.link.write(self.mid, cmd, data, timeout=timeout)
        if ok:
            import time
            self.last_reply = time.monotonic()
        return ok

    def enable(self, tries: int = 5) -> bool:
        """使能。这颗固件的 0x2A 应答经常丢，返回 False 但状态已变；
        所以以 0x2B 回读为准。"""
        if self.power() == 1:
            return True
        for _ in range(tries):
            try:
                self.link.send_cmd(self.mid, W_ENABLE, b"\x01")
            except Exception:  # noqa: BLE001
                return False
            for _ in range(4):
                time.sleep(0.15)
                if self.power() == 1:
                    return True
        return False

    def disable(self, tries: int = 3) -> bool:
        if self.power() == 0:
            return True
        for _ in range(tries):
            try:
                self.link.send_cmd(self.mid, W_ENABLE, b"\x00")
            except Exception:  # noqa: BLE001
                return False
            for _ in range(4):
                time.sleep(0.15)
                if self.power() == 0:
                    return True
        return False

    def clear_error(self) -> bool:
        # 0xFE 只发；调用方用 error() 回读确认
        return self._send_only(W_CLEAR_ERROR)

    def _send_only(self, cmd: int, data: bytes = b"") -> bool:
        try:
            self.link.send_cmd(self.mid, cmd, data)
            return True
        except Exception:  # noqa: BLE001
            return False

    def set_mode(self, mode: int) -> bool:
        # 模式写不等回包；调用方必须用 mode() 回读确认
        ok = self._send_only(W_MODE, bytes([mode]))
        if ok:
            time.sleep(0.01)
        return ok

    def set_current_a(self, amp: float) -> bool:
        """0x08 设电流（无回包）；amp 为真实安培，按 0x53 满量程归一。"""
        # 0 A 的 raw 就是 0，不需要满量程；失能时读不到 0x53 也能先发 0
        if abs(amp) < 1e-9:
            raw = 0
        else:
            cm = self.current_max_a()
            if not cm:
                return False
            raw = int(round(max(-1.0, min(1.0, amp / cm)) * IQ24))
        try:
            self.link.send_cmd(self.mid, 0x08, struct.pack(">i", raw))
        except Exception:  # noqa: BLE001
            return False
        return True

    def set_target_turns(self, turns: float) -> bool:
        raw = int(round(max(-127.999, min(127.999, turns)) * IQ24))
        # 位置目标不等回包；运动循环用实际位置/报警/限位确认
        return self._send_only(W_TARGET, struct.pack(">i", raw))

    def set_pos_limits_turns(self, lo: float, hi: float) -> bool:
        ok1 = self._w(W_SET_POS_LIMIT_H,
                      struct.pack(">i", int(round(max(-127.999, min(127.999, hi)) * IQ24))))
        ok2 = self._w(W_SET_POS_LIMIT_L,
                      struct.pack(">i", int(round(max(-127.999, min(127.999, lo)) * IQ24))))
        return ok1 and ok2

    def set_current_limit_a(self, amp: float) -> bool:
        cm = self.current_max_a()
        if not cm:
            return False
        raw = int(round(max(0.0, min(1.0, amp / cm)) * IQ24))
        return self._w(W_SET_CURRENT_LIMIT, struct.pack(">i", raw))

    def set_speed_limit_out_deg_s(self, deg_s: float) -> bool:
        rpm = deg_s / DEG_PER_MOTOR_TURN * 60.0
        raw = int(round(max(0.0, min(6000.0, rpm)) / VELOCITY_FULL_RPM * IQ24))
        return self._w(W_SET_VEL_LIMIT, struct.pack(">i", raw))

    def set_profile_out(self, vel_deg_s: float, acc_deg_s2: float,
                        dec_deg_s2: float | None = None) -> bool:
        """写位置梯形参数；输入是输出端 deg/s、deg/s²。"""
        dec_deg_s2 = vel_deg_s if dec_deg_s2 is None else dec_deg_s2
        def w(cmd, deg_s):
            # deg/s → 电机 RPM；deg/s² → 电机 RPM/s（同一换算）
            rpm = deg_s / DEG_PER_MOTOR_TURN * 60.0
            raw = int(round(rpm / PROFILE_SCALE * IQ24))
            return self._send_only(cmd, struct.pack(">i", raw))
        ok = (w(W_SET_PROFILE_VEL, vel_deg_s)
              and w(W_SET_PROFILE_ACC, acc_deg_s2)
              and w(W_SET_PROFILE_DEC, dec_deg_s2))
        if ok:
            time.sleep(0.01)
        return ok

    def set_home_turns(self, turns: float = 0.0) -> bool:
        """0x87 设 Home 值/把当前位置设为零。发完以位置回读为准。"""
        raw = int(round(turns * IQ24))
        try:
            self.link.send_cmd(self.mid, W_SET_HOME, struct.pack(">i", raw))
        except Exception:  # noqa: BLE001
            return False
        time.sleep(0.35)
        p = self.pos_turns(timeout=0.3, retries=1)
        return p is not None and abs(p * DEG_PER_MOTOR_TURN) < 0.05

    def clear_homing(self) -> bool:
        """0x88 清 Homing 数据。只负责发；真正的“设为零”是紧接着的 0x87。"""
        try:
            self.link.send_cmd(self.mid, W_CLEAR_HOME)
        except Exception:  # noqa: BLE001
            return False
        time.sleep(0.15)
        return True

    def save(self) -> bool:
        """0x0D 存 EEPROM。这颗固件不回确认（实测），只发不等，等它写完。"""
        try:
            self.link.send_cmd(self.mid, W_SAVE)
        except Exception:  # noqa: BLE001
            return False
        time.sleep(0.8)
        return True

    def telemetry(self, full: bool = True) -> MotorTelemetry:
        t = MotorTelemetry()
        p = self.power()
        t.enabled = None if p is None else (p == 1)
        t.pos_turns = self.pos_turns()
        if t.pos_turns is not None:
            t.out_deg = turns_to_out_deg(t.pos_turns)
        if not t.enabled:
            return t
        if full:
            t.velocity_rpm = self.velocity_rpm()
            t.current_a = self.current_a()
            t.voltage_v = self.voltage_v()
            t.temp_c = self.temp_c()
            t.error = self.error()
            t.mode = self.mode()
            t.current_limit_a = self.current_limit_a()
            t.speed_limit_rpm = self.speed_limit_rpm()
            lim = self.pos_limits_turns()
            if lim:
                t.pos_limit_lo_r, t.pos_limit_hi_r = lim
            t.profile_rpm = self.profile()
        else:
            t.error = self.error()
        return t
