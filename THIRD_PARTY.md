# 来源与许可证

本仓库新增宿主工具、文档和整理代码采用根目录MIT许可。第三方代码的既有版权、SPDX标记和许可证继续有效；根MIT不覆盖或撤销它们。

| 内容 | 来源／基线 | 许可文件 |
|---|---|---|
| Tracker | SlimeVR／SlimeNRF派生固件，本地整合基线 `06ce018fa6d17a3ffa8b7826b9928470881bce87` | [MIT](integration/tracker/LICENSE-MIT)、[Apache-2.0](integration/tracker/LICENSE-APACHE)，个别文件另有SPDX |
| Receiver | SlimeVR／SlimeNRF派生固件，基线 `46f0f6d6a2156204dbcc5e6fa8a5b52ce89ccd6d` | [MIT](integration/receiver/LICENSE-MIT)、[Apache-2.0](integration/receiver/LICENSE-APACHE)，个别文件另有SPDX |
| VQF C库 | https://github.com/jitingcn/vqf-c | [原许可](integration/tracker/vqf-c/LICENSE) |
| Fusion | https://github.com/xioTechnologies/Fusion | [原许可](integration/tracker/Fusion/LICENSE.md) |
| Helper | [jitingcn/SlimeNRF-Helper](https://github.com/jitingcn/SlimeNRF-Helper)，基于 `3d43dd780b997f6263762c34e996c612781b5b43` 的本地扩展 | [MIT](helper/LICENSE-MIT)、[Apache-2.0](helper/LICENSE-APACHE)，Rust 依赖另有各自声明 |
| NCS／Zephyr及其模块 | `config/sdk/`固定各仓库URL与提交，未将SDK安装目录打包 | 下载模块内各自许可证；部分Nordic文件有专属许可 |

固件源文件中的Nordic五条款SPDX等声明保持原样；保留所有随源码提供的许可证。VQF／Fusion以源码副本随包提供，固定来源记录见 `config/upstream.json`。源码清单记录本次实际内容，包含基线之后的本地改动；不能只检出上表基线就期待得到本版本。

Helper 源码已整理至 `helper/` 并纳入发布白名单；原始现场录制、设备参数和历史开发副本不公开。合成样例由项目代码生成，不含真实设备标识或个人位置。实验摘要只公开匿名统计。Python依赖由pip安装并沿用各包许可。

## Helper 校准运行组件

完整 Helper 包附带独立 `calibration/` 组件，构建依赖固定在 `requirements-helper-runtime.txt`。组件的 `licenses/` 保留 Python、NumPy、SciPy、pyserial、hidapi 及打包工具的许可和上游声明，NumPy／SciPy 轮子内的 OpenBLAS 等依赖许可也一并保留；`RUNTIME_MANIFEST.json` 记录实际文件、源码哈希和依赖版本。Helper 本体的 Rust 依赖声明仍由其原有 notices 工具生成。固件不随本次 Helper 包附送。
