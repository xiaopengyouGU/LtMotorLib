#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LTM_FOC RA6M4 SDK 一键构建脚本
用法:
  python script.py -s     # 构建 SDK（产物到 bin/）
  python script.py -c     # 清理 build 目录
"""
import os, sys, shutil, subprocess, argparse

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

ROOT          = os.path.dirname(os.path.abspath(__file__))
LTM_ROOT      = os.path.dirname(ROOT)
DEFAULT_BOARD = "ra6m4"
SDK_BUILD     = os.path.join(ROOT, "build")

def run(cmd):
    print(">>> " + " ".join(cmd))
    return subprocess.run(cmd).returncode == 0

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
        print(f"[warn] CMake 缓存源不匹配，删除重建 {dirpath}")
        shutil.rmtree(dirpath, ignore_errors=True)

def build_sdk(board):
    ensure_cache(SDK_BUILD, ROOT)
    shutil.rmtree(SDK_BUILD, ignore_errors=True)   # 每次干净重建，避免旧缓存里的工具链/生成器
    cfg = ["cmake", "-G", "MinGW Makefiles", "-B", SDK_BUILD, "-S", ROOT,
           "-DCMAKE_TOOLCHAIN_FILE=" + os.path.join(LTM_ROOT, "cmake", "toolchain_arm.cmake"),
           "-DLTM_FOC_BOARD=" + board,
           "-DCMAKE_BUILD_TYPE=Release"]
    return run(cfg) and run(["cmake", "--build", SDK_BUILD])

def sync_example_lib():
    """产物同步到 examples/basic/lib：例程永远链接最新库，避免旧副本。"""
    dst = os.path.join(ROOT, "examples", "basic", "lib")
    if not os.path.isdir(dst):
        return
    shutil.copy2(os.path.join(ROOT, "bin", "libLTM_FOC.a"),
                 os.path.join(dst, "libLTM_FOC.a"))
    shutil.copytree(os.path.join(ROOT, "bin", "ltm_foc"),
                    os.path.join(dst, "ltm_foc"), dirs_exist_ok=True)
    shutil.copy2(os.path.join(ROOT, "src", "common", "lt_motor_types.h"),
                 os.path.join(dst, "ltm_foc", "lt_motor_types.h"))
    print("[OK] 已同步 -> examples/basic/lib/")


def main():
    ap = argparse.ArgumentParser(description="LTM_FOC SDK 一键构建")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-s", "--sdk", action="store_true", help="构建 LTM_FOC 静态库")
    ap.add_argument("--board", default=DEFAULT_BOARD, help="板级配置目录名（boards/<board>），默认 ra6m4")
    g.add_argument("-c", "--clean", action="store_true", help="清理 build 目录")
    args = ap.parse_args()

    if args.clean:
        if os.path.isdir(SDK_BUILD):
            shutil.rmtree(SDK_BUILD, ignore_errors=True)
            print(f"[clean] removed {SDK_BUILD}")
        print("[OK] 清理完成")
        return
    if not args.sdk:
        ap.print_help(); return

    print(f"[1/1] 构建 LTM_FOC({args.board}) Release ...")
    if not build_sdk(args.board):
        sys.exit("[FAIL] LTM_FOC 构建失败")
    print("[OK] 产物 -> bin/libLTM_FOC.a + bin/ltm_foc/")
    sync_example_lib()

if __name__ == "__main__":
    main()
