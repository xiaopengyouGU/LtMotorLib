# tools/ —— 构建与 CI 辅助

## ci.py —— 一键 CI（本地与 CI runner 共用）

只构建 + 跑主机侧测试，不烧录：

```bash
python tools/ci.py                 # 全部 11 步
python tools/ci.py --list          # 列出步骤
python tools/ci.py --only hal foc  # 只跑名字含 hal / foc 的步骤
python tools/ci.py --stop-on-fail
```

覆盖：`LTM_CTRL`（arm/pc 库 + 主机压测）、`LTM_FOC`（ra6m4 / stm32g474）、
`LTM_HAL`（三平台 SDK + 共享例程）、`examples`（basic / canfd）、`BootLoader`（固件 + 主机测试）。

环境变量：

- `LTM_CMAKE_GENERATOR`：覆盖 CMake 生成器（Linux 用 `Unix Makefiles`，默认 `MinGW Makefiles`）
- 临时目录非 ASCII 时脚本会自动改用 `<仓库>/.ci_tmp`（LTO 的 `lto-wrapper` 吃不了中文路径）

## ci_bootstrap.sh —— Linux / CI 引导（装工具链 + 跑 ci.py）

```bash
bash tools/ci_bootstrap.sh          # 后面可跟 ci.py 的参数，如 --only hal
```

做三件事：装 `cmake/make/gcc/curl`；准备 ARM 工具链（先用发行版包，探针不过就下载官方
GNU Arm Embedded / xpack 版）；调 `tools/ci.py`。

## 怎么接到 CI 上

| 平台 | 做法 |
|---|---|
| GitHub Actions | `.github/workflows/ci.yml` 已配好，push / PR 自动跑 `bash tools/ci_bootstrap.sh` |
| Gitee 流水线 | 仓库页 → 服务 → 流水线 → 新建流水线：拉本仓库源码，加一个「执行 Shell / 命令」任务，命令填 `bash tools/ci_bootstrap.sh`；触发方式选「代码提交触发」即可 |
| 本地 Linux | 直接 `bash tools/ci_bootstrap.sh`（脚本自己装依赖，可加 `--only` 只跑部分步骤） |

## ltm_example.py —— 例程构建/烧录公共库

`examples/*`、`LTM_HAL/examples/*`、`BootLoader` 的 `script.py` 都调用它；
板级信息（HAL 包、工具链、链接脚本、J-Link 器件名与时钟、例程机型参数）来自
`boards/<board>/board.cmake`。
