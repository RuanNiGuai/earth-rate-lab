# 构建、签名与发布

当前支持本项目已使用的 Styria Mini Tracker 和 Styria R1 Receiver；不要将应用镜像当作任意同芯片设备的通用固件。当前retained内存布局依赖已审查的两级启动加载器布局，不能直接推广到其他启动加载器。

## 依赖

使用Python 3.12、Git、Zephyr SDK 0.17.4，安装CMake／Ninja及对应系统依赖。

```sh
python -m pip install -r requirements-build-lock.txt
python tools/setup_sdk.py tracker --workspace .local/sdk-tracker
python tools/setup_sdk.py receiver --workspace .local/sdk-receiver
```

`config/sdk/`冻结当前使用的全部已安装模块提交。Tracker为13个项目，Receiver为54个项目；Receiver原清单另有19个未安装项目，未纳入本板型的最小构建环境。接收器的8处Zephyr兼容改动保存在 `zephyr-compat.patch`，setup会验证后应用。此处使用的0.17.4组合与上游Receiver的1.0.1构建线不同，必须连同该补丁使用，不能只照抄上游SDK说明。

setup只接受新目录，不更新或重置已有SDK。build会核对各模块提交与已记录补丁；新源码版本构建必须使用新构建目录。安装的是构建所需base依赖；两条SDK的完整开发／测试依赖包含互相冲突的ruff固定版本，不应装入同一个环境。系统级驱动、USB权限和具体OS的SDK安装请按Zephyr SDK官方说明完成。

## 密钥

私钥通过 `--signing-key` 路径传入，不能放进源码包。`config/signing-public.pem`仅是原试用设备的验签公钥，不能用于签名。维护者原密钥已移至私有 `.local/keys/`。

自己配置启动加载器时，可用SDK中的imgtool生成ECDSA-P256密钥并导出公钥：

```sh
python .local/sdk-tracker/bootloader/mcuboot/scripts/imgtool.py keygen -k /PRIVATE/PATH/signing.pem -t ecdsa-p256
python .local/sdk-tracker/bootloader/mcuboot/scripts/imgtool.py getpub -k /PRIVATE/PATH/signing.pem -e pem > /PRIVATE/PATH/signing-public.pem
```

自己的新密钥必须与设备MCUboot所信任的公钥配套；现有设备不会因为重新签名就信任新密钥。本仓库的常规构建只生成应用，不自动更换启动加载器。历史签名路径配置不作为新构建入口。

## 构建候选

```sh
python tools/build_firmware.py tracker --workspace .local/sdk-tracker --toolchain /PATH/zephyr-sdk-0.17.4 --signing-key /PRIVATE/PATH/signing.pem --build-dir .local/build-tracker
python tools/build_firmware.py receiver --workspace .local/sdk-receiver --toolchain /PATH/zephyr-sdk-0.17.4 --build-dir .local/build-receiver
```

Windows支持绝对盘符路径；有空格时加引号。建议源码、SDK及构建目录使用纯英文路径；构建脚本将编译临时文件放在构建目录内，避免工具链使用带非ASCII字符的用户临时目录。`--git`可指定Git可执行文件。Tracker默认包含F04、CALWIRE、15–43°C温补和保留区配置；自动深睡仍关闭。候选Build ID由源码／配置哈希生成，`source-identity.json`记录完整SHA256，避免沿用CW2身份冒充原部署版本。自动生成的Tracker `VERSION`不进入源码清单及源码哈希；本机路径、含凭据或查询参数的Git远程URL不会进入固件显示信息。

## 打包

```sh
python tools/package_firmware.py --build-dir .local/build-tracker --imgtool .local/sdk-tracker/bootloader/mcuboot/scripts/imgtool.py --public-key /PRIVATE/PATH/signing-public.pem --out .local/package-tracker
python tools/package_firmware.py --build-dir .local/build-receiver --out .local/package-receiver
```

打包验证签名或HEX／UF2映射、应用地址范围，并生成文件哈希。它不依赖某次历史 `.tmp` 回滚镜像，不包含私钥，也不自动刷机。正常回退需保留自己设备原来的已知可用镜像和每台参数快照；源码包本身不承诺提供适合所有设备的回滚二进制。

固定版本的 imgtool 漏列了 ECDSA-P256 公钥对应的 SHA-256 类型，直接运行其 `verify -k public.pem` 会误报类型不匹配。打包器仅在进程内补齐这一映射，随后仍执行上游完整哈希与签名验证；不修改 SDK、不使用私钥验签。安装 SDK 后可运行额外的真实密码学回归（有效签名、错误公钥、篡改负载／签名、拒绝私钥）：

```sh
python tests/verify_signature_sdk.py --imgtool .local/sdk-tracker/bootloader/mcuboot/scripts/imgtool.py
```

公开源码目录由 `python tools/release_export.py --out release/earth-rate-lab` 生成。先在该目录执行离线测试与导出检查，再建立Git仓库。只发布明确验收的固件附件；未刷机候选不能标为已实机通过。

## Helper 连续校准运行组件

Helper 源码位于本仓库 `helper/`。默认向导从主 EXE 旁的 `calibration/earth-rate-calibration.exe` 启动本机组件，无需最终用户选择 Python 或源码目录；普通功能独立运行。开发者高级路径仍支持本源码的 Python 桥接。

Windows 主程序需要 Visual Studio C++ 工具链和 `helper/rust-toolchain.toml` 固定的 Rust。进入 Helper 目录构建，不改变全局工具链；公开仓库没有内嵌固件载荷，因此明确使用 `-NoFirmware`：

```powershell
cd helper
./build.ps1 -Release -NoFirmware
# 使用已有项目环境运行完整工作区检查；test 需要 cargo-nextest。
./build.ps1 -Action fmt -CargoArguments @('--all', '--', '--check')
./build.ps1 -Action clippy -NoFirmware -CargoArguments @('--workspace', '--all-targets', '--', '-D', 'warnings')
./build.ps1 -Action test -NoFirmware -CargoArguments @('--workspace')
./build.ps1 -Action test -NoFirmware -CargoArguments @('--workspace', '--doc')
./build.ps1 -Action test -CargoArguments @('-p', 'slimenrf-tools', '--locked')
cd ..
```

完整平台依赖与入口见 [Helper 开发文档](../helper/docs/DEVELOPMENT.md)。主程序通常输出到 `helper/target/release/slimenrf-helper.exe`；设置 `CARGO_TARGET_DIR` 时以构建实际输出为准。以下 Python 组件打包命令均从本仓库根目录执行。

使用标准 Windows Python 3.12 的独立构建环境，安装固定构建依赖后打包：

```sh
python -m pip install -r requirements-helper-runtime.txt
python tools/package_helper_runtime.py --out artifacts/helper-package --work .local/helper-runtime-build
```

输出须不存在已有 calibration 子目录。组件携带 Python、数值库、设备访问库、对应许可证和逐文件哈希清单；主程序仍按 Helper 自己的 Rust 指南编译。把两者置于同一发布目录，保留 calibration 文件夹。不要把开发机虚拟环境路径写入分发配置。若构建环境没有 LICENSE.txt，可用 `--python-license` 显式指定该版本的完整许可证。

新工作流先测试 `tools/check.py`，再做[四份完整合成录制](JOINT_SOLVER_zh.md)的准备与求解回放，以及打包组件在无 Python PATH、无开发环境路径下的离线执行检查。合成数据保持 synthetic 标记，不能写入设备。电脑端通过不代替原生界面点击和实机精度验收；本轮不刷固件。
