#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LTM_HAL SDK 一键构建脚本（ltm_hal_ra6m4）
- 仅构建 LTM_HAL 静态库：libLTM_HAL.a + ltm_hal/ltm_hal.h（bin/ 留档）
- 构建成功后自动同步 examples/*/lib/，并依次执行各例程的 python script.py -b（--no-examples 可跳过）

用法:
  python script.py -s -v 1.0.2     # 构建 SDK（产物到 bin/v1.0.2/）+ 同步 lib/ + 构建全部例程
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
GENERATOR   = "MinGW Makefiles"   # keep consistent with examples; auto-clean on mismatch
EXAMPLES_DIR = os.path.join(ROOT, "examples")
BIN_LIB      = os.path.join(ROOT, "bin", "libLTM_HAL.a")
BIN_HDR      = os.path.join(ROOT, "bin", "ltm_hal", "ltm_hal.h")


def run(cmd):
    print(">>> " + " ".join(cmd))
    return subprocess.run(cmd).returncode == 0


def ensure_cache(dirpath, source, generator):
    """CMakeCache 来源目录与当前工程不一致时（工程移动/复制后常见），删掉重建"""
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
        print(f"[warn] CMake 缓存来源不匹配，删除后重建: {dirpath}")
        shutil.rmtree(dirpath, ignore_errors=True)


def build_sdk(version):
    ensure_cache(SDK_BUILD, ROOT, GENERATOR)
    cfg = ["cmake", "-G", GENERATOR, "-B", SDK_BUILD, "-S", ROOT,
           "-DCMAKE_BUILD_TYPE=Release",
           "-DLTM_HAL_VERSION=" + version]
    return run(cfg) and run(["cmake", "--build", SDK_BUILD])


def iter_examples():
    """examples/ 下所有含 script.py 的子目录（按名字排序）"""
    if not os.path.isdir(EXAMPLES_DIR):
        return []
    return [os.path.join(EXAMPLES_DIR, n) for n in sorted(os.listdir(EXAMPLES_DIR))
            if os.path.isfile(os.path.join(EXAMPLES_DIR, n, "script.py"))]


def sync_lib(example_dir):
    """把 SDK 产物拷进例程 lib/：libLTM_HAL.a + ltm_hal/ltm_hal.h"""
    lib = os.path.join(example_dir, "lib")
    os.makedirs(os.path.join(lib, "ltm_hal"), exist_ok=True)
    shutil.copy2(BIN_LIB, os.path.join(lib, "libLTM_HAL.a"))
    shutil.copy2(BIN_HDR, os.path.join(lib, "ltm_hal", "ltm_hal.h"))


def build_example(example_dir):
    """在例程目录里执行 python script.py -b"""
    name = os.path.basename(example_dir)
    print(f">>> [{name}] {sys.executable} script.py -b   (cwd={example_dir})")
    return subprocess.run([sys.executable, "script.py", "-b"], cwd=example_dir).returncode == 0


def build_all_examples():
    exs = iter_examples()
    if not exs:
        print("[skip] examples/ 下没有可构建的例程")
        return True
    print(f"[2/2] 同步 lib/ 并构建 {len(exs)} 个例程 ...")
    for i, ex in enumerate(exs, 1):
        name = os.path.basename(ex)
        sync_lib(ex)
        print(f"[{i}/{len(exs)}] {name}: lib/ <- bin/ 已同步")
        if not build_example(ex):
            print(f"[FAIL] 例程 {name} 构建失败")
            return False
    print("[OK] 全部例程构建完成")
    return True


def main():
    ap = argparse.ArgumentParser(description="LTM_HAL SDK 一键构建")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-s", "--sdk", action="store_true", help="构建 LTM_HAL 静态库")
    g.add_argument("-c", "--clean", action="store_true", help="清理 build 目录")
    ap.add_argument("-v", "--version", default=DEFAULT_VER, help=f"LTM_HAL 版本号（默认 {DEFAULT_VER}）")
    ap.add_argument("--no-examples", action="store_true", help="只构建 SDK，不同步/不构建 examples")
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

    print(f"[1/2] 构建 LTM_HAL (version={args.version}, Release) ...")
    if not build_sdk(args.version):
        sys.exit("[FAIL] LTM_HAL 构建失败")
    print(f"[OK] 产物 -> bin/libLTM_HAL.a + bin/v{args.version}/")

    if args.no_examples:
        print("[skip] --no-examples：跳过 examples 的同步与构建")
        return
    if not build_all_examples():
        sys.exit("[FAIL] 例程构建失败")


if __name__ == "__main__":
    main()
