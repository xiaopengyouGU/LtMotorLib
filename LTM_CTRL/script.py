#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LTM_CTRL 构建脚本

  python script.py -s [arm|riscv|pc]     # 不给平台就三个全出 -> bin/<platform>/
  python script.py -e                    # 构建并运行 PC 仿真示例
  python script.py -t                    # 构建并运行 lt_speed 压测
  python script.py -b                    # -s all + -e
  python script.py -c                    # 清理构建目录

交付布局：bin/<platform>/libLTM_CTRL.a + bin/<platform>/ltm_ctrl/lt_*.h
"""
import os, sys, shutil, subprocess, argparse

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

ROOT = os.path.dirname(os.path.abspath(__file__))
GENERATOR = "MinGW Makefiles"
PLATFORMS = ("arm", "riscv", "pc")
TOOLCHAINS = {
    "arm":   os.path.join(ROOT, "cmake", "toolchain_arm.cmake"),
    "riscv": os.path.join(ROOT, "cmake", "toolchain_riscv.cmake"),
    "pc":    os.path.join(ROOT, "cmake", "toolchain.cmake"),
}
EXAMPLE = os.path.join(ROOT, "example")
EX_BUILD = os.path.join(EXAMPLE, "build")
EX_EXE = os.path.join(EX_BUILD, "sim.exe")
STRESS_EXE = os.path.join(EX_BUILD, "speed_stress.exe")


def build_dir(platform):
    return os.path.join(ROOT, "build", platform)


def run(cmd, cwd=None):
    print(">>> " + " ".join(cmd))
    return subprocess.run(cmd, cwd=cwd).returncode == 0


def ensure_cache(dirpath, source):
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
        print(f"[warn] CMake 缓存来源不匹配: {home}，删除重建")
        shutil.rmtree(dirpath, ignore_errors=True)


def build_platform(platform):
    """构建单个平台的库，产物与交付头由 CMake 拷到 bin/<platform>/"""
    d = build_dir(platform)
    ensure_cache(d, ROOT)
    cmd = ["cmake", "-G", GENERATOR, "-B", d, "-S", ROOT,
           "-DCMAKE_BUILD_TYPE=Release", "-DLTM_CTRL_PLATFORM=" + platform]
    if TOOLCHAINS[platform]:
        cmd.append("-DCMAKE_TOOLCHAIN_FILE=" + TOOLCHAINS[platform])
    return run(cmd) and run(["cmake", "--build", d])


def build_example():
    ensure_cache(EX_BUILD, EXAMPLE)
    return (run(["cmake", "-G", GENERATOR, "-B", EX_BUILD, "-S", EXAMPLE, "-DCMAKE_BUILD_TYPE=Release"])
            and run(["cmake", "--build", EX_BUILD]))


def run_example():
    if not os.path.isfile(EX_EXE):
        print("[FAIL] 找不到 sim.exe，请先构建")
        return False
    return run([EX_EXE], cwd=EX_BUILD)


def build_stress():
    """构建压测目标（example 工程）"""
    ensure_cache(EX_BUILD, EXAMPLE)
    return (run(["cmake", "-G", GENERATOR, "-B", EX_BUILD, "-S", EXAMPLE, "-DCMAKE_BUILD_TYPE=Release"])
            and run(["cmake", "--build", EX_BUILD, "--target", "speed_stress"]))


def run_stress():
    if not os.path.isfile(STRESS_EXE):
        print("[FAIL] 找不到 speed_stress.exe，请先构建")
        return False
    return run([STRESS_EXE], cwd=EX_BUILD)


def clean():
    for d in [os.path.join(ROOT, "build"), EX_BUILD]:
        if os.path.isdir(d):
            shutil.rmtree(d, ignore_errors=True)
            print(f"[clean] removed {d}")
    print("[OK] 清理完成")


def main():
    ap = argparse.ArgumentParser(description="LTM_CTRL 构建/仿真")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-s", "--sdk", nargs="*", metavar="PLATFORM", help="构建库；不给平台则 arm / riscv / pc 全出")
    g.add_argument("-e", "--example", action="store_true", help="构建并运行 PC 仿真示例")
    g.add_argument("-t", "--stress", action="store_true", help="构建并运行 lt_speed 压测")
    g.add_argument("-b", "--build", action="store_true", help="-s all + PC 仿真示例")
    g.add_argument("-c", "--clean", action="store_true", help="清理构建目录")
    args = ap.parse_args()

    if not (args.sdk is not None or args.example or args.stress or args.build or args.clean):
        ap.print_help()
        return

    if args.clean:
        clean()
        return

    if args.sdk is not None or args.build:
        plats = list(PLATFORMS) if (args.build or not args.sdk) else args.sdk
        for p in plats:
            if p not in PLATFORMS:
                sys.exit(f"[FAIL] 未知平台 '{p}'，可选：{' / '.join(PLATFORMS)}")
            if not build_platform(p):
                sys.exit(f"[FAIL] {p} 构建失败")
            print(f"[OK] bin/{p}/libLTM_CTRL.a + bin/{p}/ltm_ctrl/")

    if args.stress:
        if not build_stress():
            sys.exit("[FAIL] 压测构建失败")
        if not run_stress():
            sys.exit("[FAIL] 压测未通过")

    if args.example or args.build:
        if not build_example():
            sys.exit("[FAIL] 仿真示例构建失败")
        if not run_example():
            sys.exit("[FAIL] 仿真运行失败")

    print("[OK] 完成")


if __name__ == "__main__":
    main()