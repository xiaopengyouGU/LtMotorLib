#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LTM_HAL SDK 一键构建脚本（LTM_HAL 下所有平台共用这一份）

- 构建指定平台的静态库：bin/libLTM_HAL.a + bin/ltm_hal/ltm_hal.h（bin/v<版本>/ 留档）
- 版本号唯一出处：该平台 CMakeLists.txt 的 project(LTM_HAL VERSION x.y.z)
- 构建成功后依次构建共享例程 LTM_HAL/examples/*（板级由 --board 指定，默认按平台映射）
- 平台目录名 ltm_hal_<平台>；--hal 收短名（ra6m4）也收全名（ltm_hal_ra6m4）

用法:
  python LTM_HAL/script.py -s --hal ra6m4                # 构建 SDK + 共享例程（板级 ra6m4）
  python LTM_HAL/script.py -s --hal ra6m4 --board ra6m4  # 显式指定例程板级
  python LTM_HAL/script.py -s --hal ra6m4 --no-examples   # 只构建 SDK
  python LTM_HAL/script.py -s --hal ra6m4 -v 1.0.3        # 指定版本号
  python LTM_HAL/script.py -c --hal ra6m4                 # 清理该平台 build 目录
  python LTM_HAL/script.py --list                         # 列出可用平台

各平台目录下的 script.py 只是转发入口（`cd ltm_hal_ra6m4 && python script.py -s` 照旧可用），
逻辑只有本文件一份。
"""
import argparse
import os
import shutil
import subprocess
import sys

# 统一 UTF-8 输出：无论终端/重定向环境，print 中文不乱码
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

HAL_ROOT  = os.path.dirname(os.path.abspath(__file__))
EXAMPLES  = os.path.join(HAL_ROOT, "examples")   # 例程统一放 LTM_HAL/examples（全平台共用）
GENERATOR = "MinGW Makefiles"   # keep consistent with examples; auto-clean on mismatch

# 平台目录 -> 例程默认板级（boards/<board>）
HAL_BOARD = {
    "ltm_hal_ra6m4":          "ra6m4",
    "ltm_hal_stm32g474_rbt3": "stm32g474",
    "ltm_hal_ra8t2_CPU0":     "ra8t2",
}

# 共享例程按 Q15 接口写；仍是浮点接口的平台暂时不做例程
HAL_NO_EXAMPLES = {
    "ltm_hal_ra8t2_CPU0": "HAL 仍是浮点接口（未对齐 Q15），共享例程暂不支持",
}


def run(cmd):
    print(">>> " + " ".join(cmd))
    return subprocess.run(cmd).returncode == 0


def hal_list():
    """LTM_HAL 下所有平台目录（含 CMakeLists.txt），按名字排序"""
    return sorted(n for n in os.listdir(HAL_ROOT)
                  if n.startswith("ltm_hal_") and
                  os.path.isfile(os.path.join(HAL_ROOT, n, "CMakeLists.txt")))


def resolve_hal(name):
    """--hal 接受短名（ra6m4）或目录名（ltm_hal_ra6m4）"""
    for cand in (name, "ltm_hal_" + name):
        p = os.path.join(HAL_ROOT, cand)
        if os.path.isfile(os.path.join(p, "CMakeLists.txt")):
            return p
    sys.exit("[FAIL] 找不到平台 %s（可用：%s）" % (name, "、".join(hal_list())))


def default_version(hal_dir):
    """版本号唯一出处：该平台 CMakeLists.txt 的 project(LTM_HAL VERSION x.y.z)"""
    with open(os.path.join(hal_dir, "CMakeLists.txt"), "r", errors="ignore") as f:
        for line in f:
            if line.startswith("project(LTM_HAL"):
                item = line.split()
                if "VERSION" in item:
                    return item[item.index("VERSION") + 1]
    sys.exit("[FAIL] %s 的 CMakeLists.txt 缺少 project(LTM_HAL VERSION x.y.z)"
             % os.path.basename(hal_dir))


def ensure_cache(dirpath, source, generator):
    """CMakeCache 来源目录/生成器与当前工程不一致时（工程移动/复制后常见），删掉重建"""
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
        print(f"[warn] CMake 缓存来源/生成器不匹配，删除重建 {dirpath}")
        shutil.rmtree(dirpath, ignore_errors=True)


def build_sdk(hal_dir, build_dir, version):
    ensure_cache(build_dir, hal_dir, GENERATOR)
    cfg = ["cmake", "-G", GENERATOR, "-B", build_dir, "-S", hal_dir,
           "-DCMAKE_BUILD_TYPE=Release",
           "-DLTM_HAL_VERSION=" + version]
    return run(cfg) and run(["cmake", "--build", build_dir])


def iter_examples():
    """LTM_HAL/examples 下所有含 script.py 的例程（按名字排序）"""
    if not os.path.isdir(EXAMPLES):
        return []
    return [os.path.join(EXAMPLES, n) for n in sorted(os.listdir(EXAMPLES))
            if os.path.isfile(os.path.join(EXAMPLES, n, "script.py"))]


def build_example(example_dir, board):
    """在例程目录里执行 python script.py -b --board <board>"""
    name = os.path.basename(example_dir)
    print(f">>> [{name}] {sys.executable} script.py -b --board {board}   (cwd={example_dir})")
    return subprocess.run([sys.executable, "script.py", "-b", "--board", board],
                          cwd=example_dir).returncode == 0


def build_all_examples(board):
    exs = iter_examples()
    if not exs:
        print("[skip] LTM_HAL/examples 下没有可构建的例程")
        return True
    print(f"[2/2] 构建 {len(exs)} 个共享例程（板级 {board}） ...")
    for i, ex in enumerate(exs, 1):
        name = os.path.basename(ex)
        print(f"[{i}/{len(exs)}] {name}")
        if not build_example(ex, board):
            print(f"[FAIL] 例程 {name} 构建失败")
            return False
    print("[OK] 全部例程构建完成")
    return True


def main():
    ap = argparse.ArgumentParser(description="LTM_HAL SDK 一键构建（LTM_HAL 下所有平台共用）")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-s", "--sdk", action="store_true", help="构建 LTM_HAL 静态库")
    g.add_argument("-c", "--clean", action="store_true", help="清理该平台的 build 目录")
    ap.add_argument("--hal", help="平台名：短名 ra6m4 或目录名 ltm_hal_ra6m4")
    ap.add_argument("-v", "--version", default=None,
                    help="LTM_HAL 版本号（默认取该平台 CMakeLists 的 project 版本）")
    ap.add_argument("--board", default=None,
                    help="例程板级（boards/<board>），默认按平台映射（如 ra6m4 -> ra6m4）")
    ap.add_argument("--no-examples", action="store_true", help="只构建 SDK，不构建 examples")
    ap.add_argument("--list", action="store_true", help="列出可用平台")
    args = ap.parse_args()

    if args.list:
        for n in hal_list():
            print(n)
        return

    if not args.hal:
        ap.print_help()
        return

    hal_dir   = resolve_hal(args.hal)
    name      = os.path.basename(hal_dir)
    build_dir = os.path.join(hal_dir, "build")
    board     = args.board or HAL_BOARD.get(name, name)

    if args.clean:
        if os.path.isdir(build_dir):
            shutil.rmtree(build_dir, ignore_errors=True)
            print(f"[clean] removed {build_dir}")
        print("[OK] 清理完成")
        return

    if not args.sdk:
        ap.print_help()
        return

    version = args.version or default_version(hal_dir)
    print(f"[1/2] 构建 {name} (version={version}, Release) ...")
    if not build_sdk(hal_dir, build_dir, version):
        sys.exit("[FAIL] LTM_HAL 构建失败")
    print(f"[OK] 产物 -> bin/libLTM_HAL.a + bin/v{version}/")

    if args.no_examples:
        print("[skip] --no-examples：跳过 examples 的构建")
        return
    if name in HAL_NO_EXAMPLES:
        print("[skip] %s：%s" % (name, HAL_NO_EXAMPLES[name]))
        return
    if not build_all_examples(board):
        sys.exit("[FAIL] 例程构建失败")


if __name__ == "__main__":
    main()
