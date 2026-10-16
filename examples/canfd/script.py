#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""examples/canfd 一键构建 + 烧录

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


def prebuild(board):
    """先编依赖的 LTM_FOC 库（机型参数取自 boards/<board>/user_param_def.h）"""
    return run([sys.executable, os.path.join(LTM_ROOT, "LTM_FOC", "script.py"),
                "-s", "--board", board])


if __name__ == "__main__":
    main(project_dir=ROOT, ltm_root=LTM_ROOT, image="ltm_foc_canfd",
         default_board="ra6m4", description="examples/canfd 一键构建 + 烧录", prebuild=prebuild)
