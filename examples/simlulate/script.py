import subprocess
import sys
import os
import shutil
import argparse
import json

PROJECT_NAME = "test0"
BUILD_DIR = "build"
ELF_PATH = os.path.join(BUILD_DIR, f"{PROJECT_NAME}.elf")

def run_cmake_configure(build_type="Debug"):
    compile_cmd = [
        "cmake",
        "-G", "MinGW Makefiles",
        "-B", BUILD_DIR,
        "-S", ".",
        f"-DCMAKE_BUILD_TYPE={build_type}",
    ]
    print(f">>> 配置 CMake (Build type: {build_type}) ...")
    result = subprocess.run(compile_cmd, check=False)
    return result.returncode == 0

def run_cmake_build():
    build_cmd = ["cmake", "--build", BUILD_DIR]
    print(">>> 构建项目 ...")
    result = subprocess.run(build_cmd, check=False)
    return result.returncode == 0

def clean_build():
    if os.path.exists(BUILD_DIR):
        print(f">>> 删除构建目录 {BUILD_DIR} ...")
        shutil.rmtree(BUILD_DIR, ignore_errors=True)
    else:
        print(">>> 构建目录不存在，无需清理。")

def main():
    parser = argparse.ArgumentParser(description="STM32 项目构建与烧录脚本")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-b", "--build-only", action="store_true", help="仅构建 Debug 版本（不烧录）")
    group.add_argument("-D", "--debug-run", action="store_true", help="构建并烧录 Debug 版本")
    group.add_argument("-R", "--release-run", action="store_true", help="构建并烧录 Release 版本")
    group.add_argument("-o", "--only-flash", action="store_true", help="仅烧录（使用已构建的 .elf 文件）")
    group.add_argument("-c", "--clean", action="store_true", help="仅清理构建目录")
    args = parser.parse_args()

    # 无参数：显示帮助
    if not any(vars(args).values()):
        parser.print_help()
        sys.exit(0)

    if args.clean:
        clean_build()
        return

    # 确定构建类型（仅当需要构建时）
    build_type = "Release" if args.release_run else "Debug"
    need_build = args.build_only or args.debug_run or args.release_run
    need_run   = args.debug_run or args.release_run
    if need_build:
        if not run_cmake_configure(build_type):
            print("❌ CMake 配置失败")
            sys.exit(1)
        if not run_cmake_build():
            print("❌ 构建失败")
            sys.exit(1)
    
    if need_run:
        # 根据构建类型选择输出目录
        exe_dir = os.path.join('bin', 'release' if build_type == "Release" else 'debug')
        executable = os.path.join(exe_dir, f'{PROJECT_NAME}.exe' if sys.platform == 'win32' else PROJECT_NAME)
        print("开始运行程序！！！")

        # VSCode终端中文支持
        if sys.platform == 'win32':
            subprocess.run('chcp 65001', shell=True)  # 切换控制台代码页
            subprocess.run(executable, shell=True)
        else:
            subprocess.run(executable, shell=True)

    if args.build_only:
        print("✅ 构建完成 (Debug)")

if __name__ == "__main__":
    main()