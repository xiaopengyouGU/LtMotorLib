#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LtMotorLib 一键 CI：本地与 CI runner 共用（只构建 + 跑主机侧测试，不烧录）。

覆盖：
  LTM_CTRL     arm/pc 静态库 + 主机侧压测
  LTM_FOC      ra6m4 / stm32g474 胶水层库
  LTM_HAL      三个平台 SDK + 共享例程（LTM_HAL/examples）
  examples     basic / canfd（ra6m4）
  BootLoader   ra8t2 固件 + 平台无关测试

用法:
  python tools/ci.py                 # 全部步骤
  python tools/ci.py --list          # 列出步骤
  python tools/ci.py --only hal      # 只跑名字含 hal 的步骤（可给多个子串）
  python tools/ci.py --stop-on-fail  # 第一个失败就停

环境：
  LTM_CMAKE_GENERATOR  覆盖 CMake 生成器（Linux CI 用 "Unix Makefiles"，默认 MinGW Makefiles）
  非 ASCII 临时目录会自动改用 <仓库>/.ci_tmp（LTO 对中文 TMP 路径不友好）
"""
import argparse
import os
import subprocess
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY   = sys.executable

STEPS = [
    ("ctrl-lib",        [r"LTM_CTRL\script.py", "-s", "arm", "pc"]),
    ("ctrl-stress",     [r"LTM_CTRL\script.py", "-t"]),
    ("foc-ra6m4",       [r"LTM_FOC\script.py", "-s", "--board", "ra6m4"]),
    ("foc-g474",        [r"LTM_FOC\script.py", "-s", "--board", "stm32g474"]),
    ("hal-ra6m4",       [r"LTM_HAL\script.py", "-s", "--hal", "ra6m4"]),
    ("hal-g474",        [r"LTM_HAL\script.py", "-s", "--hal", "stm32g474_rbt3"]),
    ("hal-ra8t2",       [r"LTM_HAL\script.py", "-s", "--hal", "ra8t2_CPU0"]),
    ("example-basic",   [r"examples\basic\script.py", "-b", "--board", "ra6m4"]),
    ("example-canfd",   [r"examples\canfd\script.py", "-b", "--board", "ra6m4"]),
    ("bootloader",      [r"BootLoader\IAP-UDS\ra8t2_CPU0\script.py", "-b"]),
    ("bootloader-test", [r"BootLoader\IAP-UDS\ra8t2_CPU0\script.py", "-t"]),
]


def fix_tmp():
    """LTO 的 lto-wrapper 处理不了非 ASCII 临时路径，自动指到仓库下的纯 ASCII 目录"""
    tmp = os.environ.get("TMP") or os.environ.get("TEMP") or ""
    if tmp and all(ord(c) < 128 for c in tmp):
        return
    d = os.path.join(ROOT, ".ci_tmp")
    os.makedirs(d, exist_ok=True)
    os.environ["TMP"] = os.environ["TEMP"] = d
    print("[warn] TMP/TEMP 非 ASCII（%s），改用 %s" % (tmp, d))


def step_path(rel):
    """示例脚本里写的是 Windows 分隔符，这里统一换成当前平台的分隔符"""
    return os.path.join(ROOT, *rel.replace("\\", "/").split("/"))


def main():
    ap = argparse.ArgumentParser(description="LtMotorLib CI（构建 + 主机测试）")
    ap.add_argument("--list", action="store_true", help="列出步骤")
    ap.add_argument("--only", nargs="*", default=None, help="只跑名字含这些子串的步骤")
    ap.add_argument("--stop-on-fail", action="store_true", help="失败即停")
    args = ap.parse_args()

    steps = STEPS
    if args.only:
        steps = [s for s in STEPS if any(k in s[0] for k in args.only)]
    if args.list:
        for name, cmd in steps:
            print("%-16s %s" % (name, cmd[0]))
        return

    fix_tmp()
    env = dict(os.environ)
    results = []
    for i, (name, cmd) in enumerate(steps, 1):
        print("\n=== [%d/%d] %s ===" % (i, len(steps), name), flush=True)
        p = subprocess.run([PY, step_path(cmd[0])] + cmd[1:],
                           cwd=ROOT, env=env, capture_output=True)
        out = (p.stdout + p.stderr).decode("utf-8", errors="replace")
        ok = p.returncode == 0
        results.append((name, ok))
        if ok:
            tail = [l for l in out.splitlines() if l.startswith("[OK]")]
            print("    " + (tail[-1] if tail else "OK"), flush=True)
        else:
            print("\n".join(out.splitlines()[-25:]), flush=True)
        if not ok and args.stop_on_fail:
            break

    print("\n================ CI 汇总 ================")
    for name, ok in results:
        print("  %-16s %s" % (name, "PASS" if ok else "FAIL"))
    bad = [n for n, ok in results if not ok]
    if bad:
        sys.exit("[FAIL] %d 个步骤失败：%s" % (len(bad), ", ".join(bad)))
    print("[OK] 全部 %d 个步骤通过" % len(results))


if __name__ == "__main__":
    main()
