#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LTM_HAL 开环例程（板级可选）一键构建 + 烧录

板级映射（HAL SDK 包、工具链、链接脚本、J-Link 器件名/时钟）来自 boards/<board>/board.cmake，
构建与烧录逻辑在 tools/ltm_example.py，本文件只声明差异。
"""
import os
import sys


def _ltm_root(start):
    """向上找到含 boards/ 与 LTM_FOC/ 的仓库根"""
    d = start
    while not (os.path.isdir(os.path.join(d, "boards")) and
               os.path.isdir(os.path.join(d, "LTM_FOC"))):
        p = os.path.dirname(d)
        if p == d:
            sys.exit("[FAIL] 找不到仓库根（需含 boards/ 与 LTM_FOC/）")
        d = p
    return d


ROOT     = os.path.dirname(os.path.abspath(__file__))
LTM_ROOT = _ltm_root(ROOT)
sys.path.insert(0, os.path.join(LTM_ROOT, "tools"))

from ltm_example import main, run


if __name__ == "__main__":
    main(project_dir=ROOT, ltm_root=LTM_ROOT, image="ltm_hal_basic",
         default_board="ra6m4", description="LTM_HAL 开环例程（板级可选）一键构建 + 烧录")
