#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容入口：旧命令 `python3 -m arm.server` 现在启动独立执行器。"""
from __future__ import annotations

import sys

from .executor import main

if __name__ == "__main__":
    sys.exit(main())
