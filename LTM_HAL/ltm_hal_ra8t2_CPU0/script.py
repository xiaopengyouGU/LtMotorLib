#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LTM_HAL SDK 一键构建脚本（ltm_hal_ra8t2_CPU0）
- 仅构建 LTM_HAL 静态库：libLTM_HAL.a + ltm_hal/ltm_hal.h（bin/ 留档）
- 示例工程在 examples 下，由那边的 script.py 负责

用法:
  python script.py -s -v 1.1.0     # 构建 SDK（版本 1.1.0，产物到 bin/v1.1.0/）
  python script.py -c              # 清理 build 目录
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

ROOT        = os.path.dirname(os.path.abspath(__file__))
SDK_BUILD   = os.path.join(ROOT, "build")
DEFAULT_VER = "1.0.2"


def run(cmd):
    print(">>> " + " ".join(cmd))
    return subprocess.run(cmd).returncode == 0


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
        print(f"[warn] CMake 缓存来源不匹配，删除后重建: {dirpath}")
        shutil.rmtree(dirpath, ignore_errors=True)


def build_sdk(version):
    ensure_cache(SDK_BUILD, ROOT)
    cfg = ["cmake", "-G", "MinGW Makefiles", "-B", SDK_BUILD, "-S", ROOT,
           "-DCMAKE_BUILD_TYPE=Release",
           "-DLTM_HAL_VERSION=" + version]
    return run(cfg) and run(["cmake", "--build", SDK_BUILD])


def main():
    ap = argparse.ArgumentParser(description="LTM_HAL SDK 一键构建")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-s", "--sdk", action="store_true", help="构建 LTM_HAL 静态库")
    g.add_argument("-c", "--clean", action="store_true", help="清理 build 目录")
    ap.add_argument("-v", "--version", default=DEFAULT_VER, help=f"LTM_HAL 版本号（默认 {DEFAULT_VER}）")
    args = ap.parse_args()

    if args.clean:
        if os.path.isdir(SDK_BUILD):
            shutil.rmtree(SDK_BUILD, ignore_errors=True)
            print(f"[clean] removed {SDK_BUILD}")
        print("[OK] 清理完成")
        return

    if not args.sdk:
        ap.print_help()
        return

    print(f"[1/1] 构建 LTM_HAL (version={args.version}, Release) ...")
    if not build_sdk(args.version):
        sys.exit("[FAIL] LTM_HAL 构建失败")
    print(f"[OK] 产物 -> bin/libLTM_HAL.a + bin/v{args.version}/")


if __name__ == "__main__":
    main()
