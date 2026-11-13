# Earth Rate Lab

面向 **Styria Mini（nRF52840 + ICM-45686）与 Styria R1 接收器**的 SlimeNRF 实验分支：设备固件、可靠无线采集、十台刚体联合校准、参数回放比较及桌面 Helper。

基于 [jitingcn](https://github.com/jitingcn) 的 Tracker、Receiver、VQF 与 Helper，保留原许可证。本仓库与上游独立，**不是上游官方发行版**。当前为实验版，支持的板型、启动加载器与协议均有限定。

## 相对上游改了什么

本次对照固定为本项目已发布提交 `6f8f6b7` 与2026-11-13核实的 jitingcn Tracker `a9b441c`、Receiver `109a4d2`、VQF `6d68ad6`、Helper `2872946`（0.4.6），不使用本地中间版本充当上游。

| 范围 | 上游已经提供 | 本项目的新增与修改 |
|---|---|---|
| 校准与温补 | 逐轴灵敏度、基础校准、温度表、MLS/LUT及采集开关 | 十台刚体联合求解；修改温区／桶密度、温度确认与点保存，增加地球项和g-sens实验链 |
| 采集与无线 | 原始采集、单目标ARQ重传、批量入口、TDMA | 增加F04积分窗、接收器入队确认和结束审计；修改采集时隙、心跳及恢复逻辑 |
| 参数与Helper | 参数命令、快照、前后回读、录制／回放和校准引导 | CALWIRE整组计划、逐字段写入与核验；扩展五步向导、温表覆盖及新旧参数比较 |
| 融合、OTA和休眠 | VQF、ESB OTA、WoM及状态保留 | 修改Earth pair／观测门控／状态恢复；Styria ARC8 v2报文与Helper适配；选择性移植上游修复 |

已有功能也有实质修改，具体行为、源码和代价见[改动总览](docs/CHANGES_zh.md)及[上游文件对照](docs/UPSTREAM_DIFF_zh.md)。本项目没有完整继承当前上游的新SDK、板型和Helper依赖，也没有完整上游实机A/B精度结果。

## 当前可以做什么

- Helper 内完成三轴正反各 20 圈连续录制、十姿态与可选末尾返回；十台一起动作，文件和流程自动管理。
- F04 采集以设备积分窗、序号、入队确认、重传和结束清单核对完整性；正常 SlimeVR 姿态链路仍是另一套传输策略。
- 联合求解共享角运动、安装关系及地球方向，同时保留各设备位置不同带来的加速度差异。
- 比较新旧参数后，只写入验证较好的字段；当前推荐更新比例，保留原 Earth/g-sens。重复写入只核验已完成设备。
- 温表覆盖检查、自动采集开关、无线备份与回读、ARC8 OTA v2；自动温度采集每次开机默认关闭。

## 最重要的结果

以下均为本项目内部参数／算法对照或单次采集验收，**不是相对 jitingcn 上游固件的性能提升**。

当前十台 Tracker 已部署 `e144bbfe183d_cw6_stable`，并完成镜像确认、二次重启和校准参数保持检查。新腕部独立录制的逐项离线对照中，全关返回差 54.82°、仅启用比例校准为 1.53°、全开为 1.56°；十台独立长静置未显示明显净收益。操作者确认的地理方向只用于这份新腕部录制，完整边界见[校准消融报告](docs/CALIBRATION_ABLATION_20261113_zh.md)。

| 比较 | 已有结果 | 不能据此声称 |
|---|---|---|
| 本轮新比例与旧参数，Helper 同批录制回放 | 回位 RMS **5.68879° → 0.73644°，降低 87.05%**；静置 0.09195 → 0.09212°/分钟，基本持平 | 独立实机角度精度提高 87% |
| 历史补偿全关与全开，双方均起始归零 | 整体静置角速度 RMS **0.00516473 → 0.00482453°/s，降低 6.59%** | 整夜姿态漂移降低同样比例 |
| 稳定低通，旧单台高频静置 17 段的主机对照 | 漂移指标中位数 **0.24278 → 0.19404°/分钟，降低 20.08%** | 十台运动或所有设备都有 20% 收益 |
| 现用通信固件，约 40.5 分钟十台录制 | **1,216,480** 个窗口，结束清单与落盘审计零缺口；含全电池段 | 所有无线环境永不丢包或整夜续航已验收 |

温补在历史窄温区对照中反而变差 6.70%；g-sens 收益不稳定。地球项的总残差门槛未通过不等于共享方向不可解。完整解释、各项优先级、逐台结果与证据边界见[校准效果](docs/RESULTS_zh.md)和[实验总结](docs/EXPERIMENTS_zh.md)。

## 从哪里开始

| 目的 | 入口 |
|---|---|
| 使用桌面程序 | [本分支 Releases](https://github.com/RuanNiGuai/earth-rate-lab/releases)、[Helper 五步向导](docs/HELPER_FLEET_zh.md) |
| 首次使用及异常处理 | [快速开始](QUICKSTART_zh.md)、[操作指南](USER_GUIDE_zh.md) |
| 看改了什么、优势及代价 | [改动总览](docs/CHANGES_zh.md)、[上游文件对照](docs/UPSTREAM_DIFF_zh.md) |
| 看当前实际部署与尚未完成项 | [状态](docs/STATUS_zh.md)、[待办及验收标准](docs/ROADMAP_zh.md) |
| 理解模型及回放 | [算法](ALGORITHM_zh.md)、[联合求解](docs/JOINT_SOLVER_zh.md)、[融合回放](docs/FUSION_REPLAY_zh.md) |
| 修改代码、构建、复现 | [目录与模块](docs/PROJECT_MAP_zh.md)、[构建](docs/BUILD_zh.md)、[证据索引](docs/EVIDENCE_zh.md) |

完整 Helper 包须连同 `calibration/` 使用，不需要用户另装 Python。旧程序关闭后再启动本分支；上游 Helper 的自动更新渠道不能替代本分支的校准组件。源码不包含个人录制、设备身份、实际校准系数、私钥、SDK 或缓存。

## 离线复现

Python 3.12；完整测试另需 GCC/Clang。Windows 可通过 `CC` 指定编译器。

```sh
python -m venv .venv
# Windows: .venv/Scripts/activate；Linux/macOS: source .venv/bin/activate
python -m pip install -r requirements.txt
python tools/check.py
python tools/release_export.py --check-only
python tools/jointcal/example.py --out .local/example
python tools/jointcal/joint_rigid_v3.py --config .local/example/input.json --out .local/example-result
```

数学样例不访问硬件，不代表实际精度。私人实测数据仅公开脱敏统计和原报告摘要哈希，不能仅凭公开样例复现私人实测数值。Helper 源码在 [`helper/`](helper/README.md)，固件在 [`integration/`](integration/README.md)。

## 发布与许可

`config/publish.json` 和 `config/helper-source-files.json` 明确列出发布内容。导出到新目录：

```sh
python tools/release_export.py --out release/new-source
```

导出器检查私钥、令牌、本机路径、越界路径及维护文档链接，并生成逐文件 SHA-256 清单。历史开发副本和现场数据留在私人工作区，未被整理操作删除。

新增项目代码使用 [MIT](LICENSE)；第三方许可证继续有效，见[来源说明](THIRD_PARTY.md)。

本次快照的整理范围、测试与附件说明见[发布记录](docs/PUBLICATION_zh.md)。
