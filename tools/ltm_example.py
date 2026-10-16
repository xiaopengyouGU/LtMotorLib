#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""例程一键构建 / J-Link 烧录的公共实现（LTM_HAL 例程、LTM_FOC 例程共用）。

各例程目录下的 script.py 只声明差异（镜像名、默认板级、要不要先编依赖 SDK），
参数解析、构建、烧录逻辑只有本文件一份。

板级信息全部取自 boards/<board>/board.cmake：
  LTM_BOARD_JLINK_DEVICE / LTM_BOARD_JLINK_SPEED / LTM_HAL_SDK_DIR / LTM_BOARD_LINKER

用法（在各例程目录下）:
  python script.py -b             # 构建（Release）
  python script.py -R             # 构建 + J-Link 烧录
  python script.py -o             # 仅烧录现有镜像
  python script.py -c             # 清理 build 目录
  python script.py -b --board xx  # 指定板级（默认见各例程说明）
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

GENERATOR       = "MinGW Makefiles"
RE_JLINK_DEVICE = re.compile(r'set\(\s*LTM_BOARD_JLINK_DEVICE\s+"([^"]+)"')
RE_JLINK_SPEED  = re.compile(r'set\(\s*LTM_BOARD_JLINK_SPEED\s+([0-9]+)')


def init_stdout():
    """统一 UTF-8 输出：无论终端/重定向环境，print 中文不乱码"""
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")


def run(cmd):
    print(">>> " + " ".join(cmd))
    return subprocess.run(cmd).returncode == 0


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


def board_info(ltm_root, board):
    """J-Link 器件名与 SWD 时钟（kHz）都取自板级配置"""
    p = os.path.join(ltm_root, "boards", board, "board.cmake")
    if not os.path.isfile(p):
        sys.exit("[FAIL] 找不到板级配置 %s" % p)
    text = open(p, encoding="utf-8", errors="ignore").read()
    m = RE_JLINK_DEVICE.search(text)
    if not m:
        sys.exit("[FAIL] %s 未定义 LTM_BOARD_JLINK_DEVICE（J-Link 器件名）" % p)
    s = RE_JLINK_SPEED.search(text)
    return m.group(1), (s.group(1) if s else "1000")


def ensure_cache(build_dir, source):
    """CMakeCache 来源目录/生成器与当前工程不一致时（工程移动/复制后常见），删掉重建"""
    cache = os.path.join(build_dir, "CMakeCache.txt")
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
    if gen and gen != GENERATOR:
        mismatch = True
    if mismatch:
        print("[warn] CMake 缓存来源/生成器不匹配，删除重建 %s" % build_dir)
        shutil.rmtree(build_dir, ignore_errors=True)


def build(project_dir, board, defines=None, prebuild=None):
    """构建例程：可选先跑 prebuild（依赖的 SDK），再 cmake 配置 + 编译"""
    if prebuild is not None and not prebuild(board):
        return False
    build_dir = os.path.join(project_dir, "build")
    ensure_cache(build_dir, project_dir)
    shutil.rmtree(build_dir, ignore_errors=True)   # 干净重建：换板级后不留旧缓存
    cfg = ["cmake", "-G", GENERATOR, "-B", build_dir, "-S", project_dir,
           "-DLTM_FOC_BOARD=" + board,
           "-DCMAKE_BUILD_TYPE=Release"]
    for d in (defines or []):
        cfg.append("-D" + d)
    return run(cfg) and run(["cmake", "--build", build_dir])


def flash(jlink, project_dir, image, device, speed):
    """J-Link 烧录：优先 HEX（没有则 ELF），生成 Commander 脚本后执行"""
    bin_dir = os.path.join(project_dir, "bin")
    hexf = os.path.join(bin_dir, image + ".hex")
    elf  = os.path.join(bin_dir, image + ".elf")
    img  = hexf if os.path.isfile(hexf) else elf
    if not os.path.isfile(img):
        print("[FAIL] 找不到 %s，请先构建（-b）" % img)
        return False
    if os.path.isfile(elf):
        with open(elf, "rb") as f:
            head = f.read(20)
        if len(head) == 20 and (head[18] | (head[19] << 8)) != 40:
            print("[FAIL] %s e_machine != 40，不是 ARM 镜像" % elf)
            return False
    script = ("si SWD\n"
              "speed %s\n"
              "device %s\n"
              "connect\n"
              'loadfile "%s"\n'
              "r\n"
              "g\n"
              "qc\n") % (speed, device, os.path.abspath(img).replace("\\", "/"))
    sf = os.path.join(project_dir, "flash.jlink")
    with open(sf, "w", encoding="utf-8", newline="\n") as f:
        f.write(script)
    try:
        return run([jlink, "-CommanderScript", sf, "-ExitOnError", "1"])
    finally:
        if os.path.isfile(sf):
            os.remove(sf)


def main(project_dir, ltm_root, image, default_board, description,
         prebuild=None, defines=None):
    """例程脚本统一入口：解析参数 → 清理 / 仅烧录 / 构建(/烧录)"""
    init_stdout()
    ap = argparse.ArgumentParser(description=description)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-R", "--run", action="store_true", help="构建(Release) + 烧录")
    g.add_argument("-b", "--build", action="store_true", help="构建（Release，默认动作）")
    g.add_argument("-o", "--only-flash", action="store_true", help="仅烧录现有镜像")
    g.add_argument("-c", "--clean", action="store_true", help="清理 build 目录")
    ap.add_argument("--board", default=default_board,
                    help="板级配置目录名（boards/<board>），默认 %s" % default_board)
    ap.add_argument("--jlink", default=None, help="JLink.exe 路径（默认自动查找）")
    ap.add_argument("--speed", default=None,
                    help="SWD 时钟 kHz（默认取板级 LTM_BOARD_JLINK_SPEED）")
    args = ap.parse_args()

    build_dir = os.path.join(project_dir, "build")
    if args.clean:
        if os.path.isdir(build_dir):
            shutil.rmtree(build_dir, ignore_errors=True)
            print("[clean] removed " + build_dir)
        print("[OK] 清理完成")
        return

    device, speed = board_info(ltm_root, args.board)
    if args.speed:
        speed = args.speed

    jlink = None
    if args.run or args.only_flash:
        jlink = args.jlink or find_jlink()
        if not jlink:
            sys.exit("[FAIL] 找不到 JLink.exe，用 --jlink 指定或设 JLINK_PATH")

    if args.only_flash:
        if not flash(jlink, project_dir, image, device, speed):
            sys.exit("[FAIL] 烧录失败")
        print("[OK] 烧录完成")
        return

    if not build(project_dir, args.board, defines=defines, prebuild=prebuild):
        sys.exit("[FAIL] 构建失败")
    if args.run:
        if not flash(jlink, project_dir, image, device, speed):
            sys.exit("[FAIL] 烧录失败")
        print("[OK] 构建 + 烧录完成")
    else:
        print("[OK] 构建完成")
