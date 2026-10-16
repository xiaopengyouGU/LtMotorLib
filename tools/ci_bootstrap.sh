#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# LTM CI 引导脚本（Linux / CI runner 共用）
#
# 作用：装好 CMake + Make + 宿主 GCC + ARM 交叉工具链，检查工具链可用，然后
#       调 tools/ci.py 跑完整构建与主机侧测试。
#
# 用法（GitHub Actions / Gitee 流水线 / 本地 Linux 都一样）：
#   bash tools/ci_bootstrap.sh
#
# ARM 工具链优先用发行版包（快、走内网镜像）；先用 probe 检查头文件与
# nano/nosys specs 是否齐全，不齐再下载官方 GNU Arm Embedded（xpack 版）。
# ---------------------------------------------------------------------------
set -e

SUDO=""
if command -v sudo >/dev/null 2>&1; then
    SUDO="sudo"
fi

probe_toolchain() {
    printf '#include <string.h>\n#include <assert.h>\nint main(void){return 0;}\n' > /tmp/ltm_probe.c
    arm-none-eabi-gcc -c /tmp/ltm_probe.c -o /tmp/ltm_probe.o 2>/dev/null &&
        arm-none-eabi-gcc /tmp/ltm_probe.c -specs=nano.specs -specs=nosys.specs \
            -o /tmp/ltm_probe.elf 2>/dev/null
}

echo "==== [1/3] 安装基础工具（cmake / make / gcc / curl）"
$SUDO apt-get update -qq || true
$SUDO apt-get install -y -qq --no-install-recommends cmake make gcc curl || true

echo "==== [2/3] 准备 ARM 交叉工具链"
$SUDO apt-get install -y -qq --no-install-recommends \
    gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi || true

if probe_toolchain; then
    echo "[ok] 发行版 ARM 工具链可用：$(arm-none-eabi-gcc --version | head -1)"
else
    echo "[info] 发行版工具链不完整（缺 newlib 头或 nano/nosys specs），下载官方 GNU Arm Embedded"
    ARM_VER="13.3.1-1.1"
    curl -fL --retry 3 --retry-delay 2 -o /tmp/ltm_armgcc.tar.gz \
        "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v${ARM_VER}/xpack-arm-none-eabi-gcc-${ARM_VER}-linux-x64.tar.gz"
    $SUDO mkdir -p /opt/armgcc
    $SUDO tar -xf /tmp/ltm_armgcc.tar.gz -C /opt/armgcc --strip-components=1
    export PATH="/opt/armgcc/bin:$PATH"
    probe_toolchain
    echo "[ok] 工具链：$(arm-none-eabi-gcc --version | head -1)"
fi

echo "==== [3/3] 构建 + 主机测试（tools/ci.py）"
python3 "$(dirname "$0")/ci.py" "$@"
