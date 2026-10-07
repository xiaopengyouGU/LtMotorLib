#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
examples/basic 开环示例一键脚本
- 构建 ltm_hal_basic（依赖 ../../ltm_hal_ra6m4/bin 的 SDK 产物）
- J-Link 一键烧录

用法:
  python script.py -b           # 构建（Release）
  python script.py -R           # 构建 + 烧录
  python script.py -o           # 仅烧录现有 elf
  python script.py -c           # 清理 build 目录
"""
import os
import sys
import shutil
import subprocess
import argparse

# 统一 UTF-8 输出：无论终端/重定向环境，print 中文不乱码
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

ROOT   = os.path.dirname(os.path.abspath(__file__))
BUILD  = os.path.join(ROOT, "build")
ELF    = os.path.join(ROOT, "bin", "ltm_hal_basic.elf")
DEVICE = "R7FA6M4AF"


def run(cmd):
    print(">>> " + " ".join(cmd))
    return subprocess.run(cmd).returncode == 0


GENERATOR = "MinGW Makefiles"


def ensure_cache(dirpath, source, generator):
    """Remove stale CMake cache (moved/copied project or generator mismatch)."""
    cache = os.path.join(dirpath, "CMakeCache.txt")
    if not os.path.isfile(cache):
        return
    home = None
    gen  = None
    with open(cache, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL"):
                home = line.split("=", 1)[1].strip()
            elif line.startswith("CMAKE_GENERATOR:INTERNAL"):
                gen = line.split("=", 1)[1].strip()
    mismatch = False
    if home and os.path.normcase(os.path.normpath(home)) != os.path.normcase(os.path.normpath(source)):
        mismatch = True
    if gen and gen != generator:
        mismatch = True
    if mismatch:
        print(f"[warn] stale CMake cache, rebuilding dir: {dirpath}")
        shutil.rmtree(dirpath, ignore_errors=True)


def build():
    ensure_cache(BUILD, ROOT, GENERATOR)
    cfg = ["cmake", "-G", GENERATOR, "-B", BUILD, "-S", ROOT,
           "-DCMAKE_BUILD_TYPE=Release"]
    return run(cfg) and run(["cmake", "--build", BUILD])


def find_jlink():
    exe = os.environ.get("JLINK_PATH")
    if exe and os.path.isfile(exe):
        return exe
    exe = shutil.which("JLink.exe") or shutil.which("JLink")
    if exe:
        return exe
    for p in (r"C:\Program Files\SEGGER\JLink\JLink.exe",
              r"C:\Program Files (x86)\SEGGER\JLink\JLink.exe"):
        if os.path.isfile(p):
            return p
    return None


def flash(jlink):
    elf = os.path.abspath(ELF).replace("\\", "/")
    if not os.path.isfile(elf):
        print(f"[FAIL] 找不到 {elf}，请先构建")
        return False
    script = (f"device {DEVICE}\n"
              "si SWD\n"
              "speed 1000\n"
              "r\nh\n"
              f"loadfile {elf}\n"
              "r\ng\nexit\n")
    sf = "flash.jlink"
    with open(sf, "w") as f:
        f.write(script)
    try:
        return run([jlink, sf])
    finally:
        if os.path.isfile(sf):
            os.remove(sf)


def main():
    ap = argparse.ArgumentParser(description="examples/basic 开环示例 一键构建/烧录")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-R", "--run", action="store_true", help="构建(Release) + 烧录")
    g.add_argument("-b", "--build", action="store_true", help="构建（Release）")
    g.add_argument("-o", "--only-flash", action="store_true", help="仅烧录现有 elf")
    g.add_argument("-c", "--clean", action="store_true", help="清理 build 目录")
    ap.add_argument("--jlink", default=None, help="JLink.exe 路径（默认自动查找）")
    args = ap.parse_args()

    if args.clean:
        if os.path.isdir(BUILD):
            shutil.rmtree(BUILD, ignore_errors=True)
            print(f"[clean] removed {BUILD}")
        print("[OK] 清理完成")
        return

    if args.run:
        if not build():
            sys.exit("[FAIL] 构建失败")
        jlink = args.jlink or find_jlink()
        if not jlink:
            sys.exit("[FAIL] 找不到 JLink.exe（--jlink 指定或设 JLINK_PATH）")
        if not flash(jlink):
            sys.exit("[FAIL] 烧录失败")
        print("[OK] 烧录完成")
    elif args.only_flash:
        jlink = args.jlink or find_jlink()
        if not jlink:
            sys.exit("[FAIL] 找不到 JLink.exe（--jlink 指定或设 JLINK_PATH）")
        if not flash(jlink):
            sys.exit("[FAIL] 烧录失败")
        print("[OK] 烧录完成")
    else:
        if not build():
            sys.exit("[FAIL] 构建失败")
        print("[OK] 构建完成")


if __name__ == "__main__":
    main()
