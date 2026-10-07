#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""examples/basic 一键构建 + J-Link 烧录

板级映射来自 boards/<board>/board.cmake：HAL SDK、工具链、机型参数、链接脚本。

用法:
  python script.py -b               # 构建 SDK + 例程（Release）
  python script.py -R               # 构建 + 烧录
  python script.py -o               # 仅烧录现有 HEX
  python script.py -c               # 清理 build 目录
  python script.py -b --board xxx   # 指定板级配置（默认 ra6m4）
"""
import os
import sys
import shutil
import subprocess
import argparse

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

ROOT     = os.path.dirname(os.path.abspath(__file__))              # examples/basic
LTM_ROOT = os.path.normpath(os.path.join(ROOT, "..", ".."))        # 仓库根
FOC      = os.path.join(LTM_ROOT, "LTM_FOC")
BUILD    = os.path.join(ROOT, "build")
BIN      = os.path.join(ROOT, "bin")
ELF      = os.path.join(BIN, "ltm_foc_basic.elf")
HEX      = os.path.join(BIN, "ltm_foc_basic.hex")
DEVICE   = "R7FA6M4AF"          # J-Link 器件名（RA6M4 / Cortex-M33）
SPEED    = "1000"               # SWD 时钟 kHz


def run(cmd):
    print(">>> " + " ".join(cmd))
    return subprocess.run(cmd).returncode == 0


def build(board):
    # 1) 构建该板的 LTM_FOC 库：机型参数取自 boards/<board>/user_param_def.h
    if not run([sys.executable, os.path.join(FOC, "script.py"), "-s", "--board", board]):
        return False

    # 2) 构建例程：链接 LTM_FOC/bin + boards/<board> 指定的 HAL SDK
    #    干净重建，避免切换板级/工具链时的旧缓存
    shutil.rmtree(BUILD, ignore_errors=True)
    cfg = ["cmake", "-G", "MinGW Makefiles", "-B", BUILD, "-S", ROOT,
           "-DLTM_FOC_BOARD=" + board,
           "-DCMAKE_BUILD_TYPE=Release"]
    return run(cfg) and run(["cmake", "--build", BUILD])


def find_jlink():
    exe = os.environ.get("JLINK_PATH")
    if exe and os.path.isfile(exe):
        return exe
    exe = shutil.which("JLink.exe") or shutil.which("JLink")
    if exe:
        return exe
    for p in (r"D:\GoodThing\HPM_SDK_Develop\JLink\JLink_V936\JLink.exe",
              r"C:\Program Files\SEGGER\JLink\JLink.exe",
              r"C:\Program Files (x86)\SEGGER\JLink\JLink.exe"):
        if os.path.isfile(p):
            return p
    return None


def flash(jlink):
    img = HEX if os.path.isfile(HEX) else ELF
    if not os.path.isfile(img):
        print("[FAIL] 找不到 %s，请先构建（-b）" % img)
        return False

    with open(ELF, "rb") as f:
        head = f.read(20)
    if len(head) == 20 and (head[18] | (head[19] << 8)) != 40:
        print("[FAIL] %s e_machine != 40，不是 ARM 镜像" % ELF)
        return False

    script = ("si SWD\n"
              "speed %s\n"
              "device %s\n"
              "connect\n"
              'loadfile "%s"\n'
              "r\n"
              "g\n"
              "qc\n") % (SPEED, DEVICE, os.path.abspath(img).replace("\\", "/"))
    sf = os.path.join(ROOT, "flash.jlink")
    with open(sf, "w") as f:
        f.write(script)
    try:
        return run([jlink, "-CommanderScript", sf, "-ExitOnError", "1"])
    finally:
        if os.path.isfile(sf):
            os.remove(sf)


def main():
    ap = argparse.ArgumentParser(description="examples/basic 一键构建 + 烧录")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-R", "--run", action="store_true", help="构建(Release) + 烧录")
    g.add_argument("-b", "--build", action="store_true", help="构建（Release）")
    g.add_argument("-o", "--only-flash", action="store_true", help="仅烧录现有镜像")
    g.add_argument("-c", "--clean", action="store_true", help="清理 build 目录")
    ap.add_argument("--board", default="ra6m4", help="板级配置目录名（boards/<board>）")
    ap.add_argument("--jlink", default=None, help="JLink.exe 路径（默认自动查找）")
    args = ap.parse_args()

    if args.clean:
        if os.path.isdir(BUILD):
            shutil.rmtree(BUILD, ignore_errors=True)
            print("[clean] removed " + BUILD)
        print("[OK] 清理完成")
        return

    jlink = args.jlink or find_jlink()
    if not jlink:
        sys.exit("[FAIL] 找不到 JLink.exe，用 --jlink 指定或设 JLINK_PATH")

    if args.run:
        if not build(args.board):
            sys.exit("[FAIL] 构建失败")
        if not flash(jlink):
            sys.exit("[FAIL] 烧录失败")
        print("[OK] 构建 + 烧录完成")
    elif args.only_flash:
        if not flash(jlink):
            sys.exit("[FAIL] 烧录失败")
        print("[OK] 烧录完成")
    elif args.build:
        if not build(args.board):
            sys.exit("[FAIL] 构建失败")
        print("[OK] 构建完成")
    else:
        ap.print_help()


if __name__ == "__main__":
    main()