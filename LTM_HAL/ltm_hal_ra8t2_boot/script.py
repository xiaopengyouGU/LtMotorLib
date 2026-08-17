#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LTM_HAL 工程一键脚本
- 构建 LTM_HAL 静态库（支持不同版本生成：-v x.y.z）
- 构建 example 开环例程
- J-Link 一键烧录 example

用法:
  python script.py -s -v 1.1.0     # 仅构建 LTM_HAL（版本 1.1.0，产物到 bin/v1.1.0/）
  python script.py -e              # 仅构建 example（需先构建 SDK）
  python script.py -b              # 构建 SDK + example
  python script.py -R              # 构建 SDK + example 并烧录（Release）
  python script.py -D              # 构建 SDK + example 并烧录（Debug）
  python script.py -o              # 仅烧录现有 example elf
  python script.py -c              # 清理 build 目录
  python script.py --jlink <路径>  # 指定 JLink.exe（默认自动查找）
"""
import os
import sys
import shutil
import subprocess
import argparse

ROOT        = os.path.dirname(os.path.abspath(__file__))
SDK_BUILD   = os.path.join(ROOT, "build")
SDK_BIN     = os.path.join(ROOT, "bin")
EXAMPLE_DIR = os.path.join(ROOT, "example")
EX_BUILD    = os.path.join(EXAMPLE_DIR, "build")
EX_ELF      = os.path.join(EX_BUILD, "ltm_hal_example.elf")
DEVICE      = "R7KA8T2LF_CPU0"
DEFAULT_VER = "1.0.1"


def find_jlink():
    """按优先级查找 JLink.exe：环境变量 JLINK_PATH > PATH > 默认安装目录"""
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


def run(cmd):
    print(">>> " + " ".join(cmd))
    r = subprocess.run(cmd)
    return r.returncode == 0


def ensure_cache(dirpath, source):
    """CMakeCache 来源目录与当前工程不一致时（工程移动/复制后常见），删掉重建"""
    cache = os.path.join(dirpath, "CMakeCache.txt")
    if not os.path.isfile(cache):
        return
    home = None
    with open(cache, "r", errors="ignore") as f:
        for line in f:
            if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL"):
                home = line.split("=", 1)[1].strip()
                break
    if home and os.path.normcase(os.path.normpath(home)) != os.path.normcase(os.path.normpath(source)):
        print(f"[warn] CMake 缓存来源不匹配: {home}")
        print(f"[warn] 删除 {dirpath} 后重建")
        shutil.rmtree(dirpath, ignore_errors=True)


def build_sdk(version, build_type):
    ensure_cache(SDK_BUILD, ROOT)
    cfg = ["cmake", "-G", "MinGW Makefiles", "-B", SDK_BUILD, "-S", ROOT,
           "-DCMAKE_BUILD_TYPE=" + build_type,
           "-DLTM_HAL_VERSION=" + version]
    return run(cfg) and run(["cmake", "--build", SDK_BUILD])


def build_example(build_type):
    ensure_cache(EX_BUILD, EXAMPLE_DIR)
    cfg = ["cmake", "-G", "MinGW Makefiles", "-B", EX_BUILD, "-S", EXAMPLE_DIR,
           "-DCMAKE_BUILD_TYPE=" + build_type]
    return run(cfg) and run(["cmake", "--build", EX_BUILD])


def flash(jlink):
    elf = os.path.abspath(EX_ELF).replace("\\", "/")
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


def clean():
    for d in (SDK_BUILD, EX_BUILD):
        if os.path.isdir(d):
            shutil.rmtree(d, ignore_errors=True)
            print(f"[clean] removed {d}")
    print("[OK] 清理完成")


def main():
    ap = argparse.ArgumentParser(description="LTM_HAL 一键构建/烧录")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-s", "--sdk-only", action="store_true", help="仅构建 LTM_HAL")
    g.add_argument("-e", "--example-only", action="store_true", help="仅构建 example")
    g.add_argument("-b", "--build", action="store_true", help="构建 SDK + example")
    g.add_argument("-R", "--release-run", action="store_true", help="构建(Release) + 烧录")
    g.add_argument("-D", "--debug-run", action="store_true", help="构建(Debug) + 烧录")
    g.add_argument("-o", "--only-flash", action="store_true", help="仅烧录现有 elf")
    g.add_argument("-c", "--clean", action="store_true", help="清理 build 目录")
    ap.add_argument("-v", "--version", default=DEFAULT_VER, help=f"LTM_HAL 版本号（默认 {DEFAULT_VER}）")
    ap.add_argument("--jlink", default=None, help="JLink.exe 路径（默认自动查找）")
    args = ap.parse_args()

    if args.clean:
        clean()
        return

    if not any([args.sdk_only, args.example_only, args.build,
                args.release_run, args.debug_run, args.only_flash]):
        ap.print_help()
        return

    build_type = "Release"
    if args.debug_run:
        build_type = "Debug"

    need_sdk = args.sdk_only or args.build or args.release_run or args.debug_run
    need_ex  = args.example_only or args.build or args.release_run or args.debug_run
    need_flash = args.release_run or args.debug_run or args.only_flash

    if need_sdk:
        print(f"[1/3] 构建 LTM_HAL (version={args.version}, {build_type}) ...")
        if not build_sdk(args.version, build_type):
            sys.exit("[FAIL] LTM_HAL 构建失败")
        print(f"[OK] 产物 -> bin/libLTM_HAL.a + bin/v{args.version}/")

    if need_ex:
        print(f"[2/3] 构建 example ({build_type}) ...")
        if not build_example(build_type):
            sys.exit("[FAIL] example 构建失败")

    if need_flash:
        jlink = args.jlink or find_jlink()
        if not jlink:
            sys.exit("[FAIL] 找不到 JLink.exe（--jlink 指定或设 JLINK_PATH）")
        print(f"[3/3] 烧录 {EX_ELF} (JLink: {jlink}) ...")
        if not flash(jlink):
            sys.exit("[FAIL] 烧录失败")
        print("[OK] 烧录完成")
    else:
        print("[OK] 完成")


if __name__ == "__main__":
    main()