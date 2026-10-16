#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""转发入口：LTM_HAL 各平台的构建脚本统一在 LTM_HAL/script.py（--hal <平台>）。

保留本文件，是为了让 `cd ltm_hal_xxx && python script.py -s` 这类用法继续可用；
构建逻辑不在本文件里。
"""
import os
import subprocess
import sys

HERE    = os.path.dirname(os.path.abspath(__file__))
UNIFIED = os.path.join(os.path.dirname(HERE), "script.py")

sys.exit(subprocess.run(
    [sys.executable, UNIFIED, "--hal", os.path.basename(HERE)] + sys.argv[1:]).returncode)
