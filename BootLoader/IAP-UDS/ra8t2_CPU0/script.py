#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一 BootLoader 一键脚本

构建 / 烧录复用 tools/ltm_example.py；板级（工具链、HAL 包、运行区链接脚本、
J-Link 器件名与时钟、App 起始地址）全部来自 boards/<board>/board.cmake。

用法:
  python script.py -b              # 仅构建（Release）
  python script.py -R              # 构建(Release) + 烧录
  python script.py -D              # 构建(Debug) + 烧录
  python script.py -o              # 仅烧录现有镜像
  python script.py -t              # 平台无关测试（功能 + 压测）
  python script.py -u --port COM3 --firmware app.bin   # UART IAP 升级
  python script.py -c              # 清理 build 目录
"""
import argparse
import os
import subprocess
import sys


def _ltm_root(start):
    """向上找到含 boards/ 与 LTM_FOC/ 的仓库根"""
    d = start
    while not (os.path.isdir(os.path.join(d, "boards")) and
               os.path.isdir(os.path.join(d, "LTM_FOC"))):
        p = os.path.dirname(d)
        if p == d:
            sys.exit("[FAIL] 找不到仓库根（需含 boards/ 与 LTM_FOC/）")
        d = p
    return d


ROOT     = os.path.dirname(os.path.abspath(__file__))
LTM_ROOT = _ltm_root(ROOT)
sys.path.insert(0, os.path.join(LTM_ROOT, "tools"))

from ltm_example import (board_field, board_info, build, clean, find_jlink,
                         flash, init_stdout, run)

IMAGE    = "BootLoader"
TEST_DIR = os.path.join(ROOT, "host", "test")


def run_tests():
    """平台无关测试：core + mock，编译成 PC 可执行文件直接跑"""
    if not os.path.isdir(TEST_DIR):
        print("[FAIL] 找不到测试目录: %s" % TEST_DIR)
        return False
    core = os.path.join(ROOT, "core")
    ok = True
    for src, exe in (("test_main.c", "test_runner.exe"),
                     ("stress_test.c", "stress_runner.exe")):
        cmd = ["gcc", "-std=c11", "-Wall", "-Wextra",
               "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast",
               "-I" + core, "-I" + os.path.join(LTM_ROOT, "components"),
               "-I" + os.path.join(ROOT, "port"),
               "-I" + TEST_DIR, "-I" + os.path.join(ROOT, "app"),
               os.path.join(core, "bootloader.c"),
               os.path.join(core, "uds_server.c"),
               os.path.join(core, "iso15765.c"),
               os.path.join(LTM_ROOT, "components", "protocol", "protocol.c"),
               os.path.join(LTM_ROOT, "components", "protocol", "ltm_commut.c"),
               os.path.join(TEST_DIR, "mock.c"),
               os.path.join(TEST_DIR, "port_mock.c"),
               os.path.join(TEST_DIR, src),
               "-o", os.path.join(TEST_DIR, exe)]
        if not run(cmd):
            ok = False
            continue
        if subprocess.run([os.path.join(TEST_DIR, exe)]).returncode != 0:
            ok = False
    return ok


def uart_upgrade(port, firmware, baud, app_base):
    """UART IAP 升级：调用 host/upgrade_client.py"""
    client = os.path.join(ROOT, "host", "upgrade_client.py")
    return run([sys.executable, client, "--port", port, "--firmware", firmware,
                "--baud", str(baud), "--app-base", hex(app_base)])


def main():
    init_stdout()
    ap = argparse.ArgumentParser(description="统一 BootLoader 构建/烧录/测试/升级")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-b", "--build", action="store_true", help="仅构建（Release）")
    g.add_argument("-R", "--release-run", action="store_true", help="构建(Release) + 烧录")
    g.add_argument("-D", "--debug-run", action="store_true", help="构建(Debug) + 烧录")
    g.add_argument("-o", "--only-flash", action="store_true", help="仅烧录现有镜像")
    g.add_argument("-t", "--test", action="store_true", help="平台无关测试（功能 + 压测）")
    g.add_argument("-u", "--uart-upgrade", action="store_true", help="UART IAP 升级")
    g.add_argument("-c", "--clean", action="store_true", help="清理 build 目录")
    ap.add_argument("--board", default="ra8t2", help="板级配置目录名（boards/<board>），默认 ra8t2")
    ap.add_argument("--jlink", default=None, help="JLink.exe 路径（默认自动查找）")
    ap.add_argument("--keep-app", action="store_true",
                    help="保留运行区旧 App（默认烧 BootLoader 时清掉运行区向量表）")
    ap.add_argument("--port", default=None, help="UART 升级串口（如 COM3）")
    ap.add_argument("--baud", type=int, default=115200, help="UART 波特率")
    ap.add_argument("--firmware", default=None, help="固件 bin 文件")
    ap.add_argument("--app-base", type=lambda x: int(x, 0), default=None,
                    help="App 起始地址（默认取板级 LTM_BOARD_APP_ADDR）")
    args = ap.parse_args()

    app_base = args.app_base or int(board_field(LTM_ROOT, args.board, 
                                     "APP_ADDR", "0x02008000"), 0)

    if args.clean:
        clean(ROOT)
        return

    if args.uart_upgrade:
        if not args.port or not args.firmware:
            ap.print_help()
            return
        if not uart_upgrade(args.port, args.firmware, args.baud, app_base):
            sys.exit("[FAIL] UART 升级失败")
        print("[OK] UART 升级完成")
        return

    if args.test:
        print("[1/1] 平台无关测试 ...")
        if not run_tests():
            sys.exit("[FAIL] 测试未通过")
        print("[OK] 测试全部通过")
        return

    if not any([args.build, args.release_run, args.debug_run, args.only_flash]):
        ap.print_help()
        return

    build_type = "Debug" if args.debug_run else "Release"
    device, speed = board_info(LTM_ROOT, args.board)

    if args.build or args.release_run or args.debug_run:
        print("[1/2] 构建 %s（%s，板级 %s）..." % (IMAGE, build_type, args.board))
        if not build(ROOT, args.board, build_type=build_type):
            sys.exit("[FAIL] 构建失败")
        print('[OK] 产物 -> bin/%s.elf/.hex/.bin' % IMAGE)

    if args.release_run or args.debug_run or args.only_flash:
        jlink = args.jlink or find_jlink()
        if not jlink:
            sys.exit("[FAIL] 找不到 JLink.exe，用 --jlink 指定或设 JLINK_PATH")
        wipe = None if args.keep_app else [(app_base, b"\x00" * 8)]
        print("[2/2] 烧录 %s（JLink: %s）%s"
              % (IMAGE, jlink, "，保留运行区 App" if args.keep_app
                 else "，清掉运行区向量表（0x%08X）" % app_base))
        if not flash(jlink, ROOT, IMAGE, device, speed, downloads=wipe):
            sys.exit("[FAIL] 烧录失败")
        print("[OK] 完成")


if __name__ == "__main__":
    main()
