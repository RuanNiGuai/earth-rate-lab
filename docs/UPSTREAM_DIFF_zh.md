# 已发布项目与 jitingcn 上游文件对照

核查日期：**2026-11-13（Asia/Shanghai）**。本次重新读取GitHub公开分支提交，并从Git对象提取双方文件。项目侧固定为已上传GitHub的 `lab-2026.11.12` 源码；上游侧固定为下表公开提交。**不以本地整合快照、Helper旧版或本项目中间固件作为“上游”。** 面向使用者的功能对照见[改动总览](CHANGES_zh.md)。

后续范围说明：本页统计和功能证据仍对应下表固定提交。其后的稳定低通、状态版本、加速度快照和恢复边界修正见[精度版本](ACCURACY_CANDIDATE_zh.md)，十台 CW6 实刷、二次重启和参数保持见[部署记录](DEPLOYMENT_20261113_zh.md)。本页所述“主机原型”“只读与模拟验收”是被审计提交的状态，不能用来覆盖后续部署记录。

## 1. 对比对象与方法

| 对象 | 仓库／分支 | 本次固定提交 |
|---|---|---|
| 已发布项目 | [RuanNiGuai/earth-rate-lab](https://github.com/RuanNiGuai/earth-rate-lab) `main` | `6f8f6b77cf60bad262f155e3dee9ff552995d715` |
| 上游Tracker | [jitingcn/SlimeVR-Tracker-nRF](https://github.com/jitingcn/SlimeVR-Tracker-nRF) `dev` | `a9b441ccec8a31f5f2903b17663e939a41c265ab` |
| 上游Receiver | [jitingcn/SlimeVR-Tracker-nRF-Receiver](https://github.com/jitingcn/SlimeVR-Tracker-nRF-Receiver) `dev` | `109a4d26ac93e69bc326865fc741960ae13e828d` |
| 上游VQF C | [jitingcn/vqf-c](https://github.com/jitingcn/vqf-c) `cmsis-dsp-optimization` | `6d68ad6905bf278ae1b528736f9d3ffccf235e20` |
| 上游Helper | [jitingcn/SlimeNRF-Helper](https://github.com/jitingcn/SlimeNRF-Helper) `main`，源码版本0.4.6 | `2872946e04c8d625e3eb7f74e8bd7e7954ac9c6d` |

这些上游提交与11月12日记录一致；需要修正的是功能对照口径及项目侧身份，而不是假设上游刚刚更新。旧审计的上游HEAD原本已指向公开源码，但项目侧读取可变工作目录，还额外混列Helper 0.4.3基线；旧“改动总览”则以本地迭代问题为主。本次将双方都固定到公开Git提交，删除中间基线比较列，重写既有功能的具体修改说明。

`config/upstream.json` 中的 `06ce018…`、`46f0f6d…` 是本地历史整合ID，Helper `3d43dd7…` 是导入来源记录；它们只说明来源，不能充当本次比较基准，也不据此计算上游ahead/behind。[审计配置](../config/upstream-audit.json)单独记录本次双方身份。

按相同相对路径比较文件，统一CRLF/LF，保留其他字节差异；这不是Git重命名检测或功能等价证明。Tracker的 `vqf-c/` 单独比较，其上游gitlink恰好也是上表VQF提交；本地附带的 `Fusion/` 是第三方库，不算自行新增功能；生成文件 `VERSION` 排除。其余配置、文档、测试、板型均计入文件统计。

## 2. 文件统计及其含义

| 范围 | 上游普通文件 | 项目文件 | 内容相同 | 项目独有路径 | 同路径内容不同 | 仅上游有该路径 |
|---|---:|---:|---:|---:|---:|---:|
| Tracker（排除上述项） | 738 | 379 | 70 | 157 | 152 | 516 |
| Receiver | 256 | 262 | 228 | 9 | 25 | 3 |
| VQF | 4 | 4 | 2 | 0 | 2 | 0 |
| Helper | 433 | 467 | 375 | 35 | 57 | 1 |

[逐文件JSON](data/upstream-diff.json)保存状态、归一化内容SHA-256及增减行数。`added`／`removed`仅表示双方路径集合不同：Tracker的157个独有路径中有86个在 `boards/`，516个仅上游路径中有271个在 `tests/`、108个在 `boards/`、93个在 `src/`。版本年代、目录拆分、SDK适配和发布筛选都会造成这些差异，不能宣称新增157个功能或删除516项能力。相同功能换了文件名也可能同时计入两边。

## 3. 已有功能的具体修改与文件证据

下列上游链接全部固定到本次提交；本地链接指向本仓库文件，其被审计内容取自上表项目提交。“已有”仅说明功能来源，不表示两边实现相同。

### 3.1 灵敏度、基础校准与联合求解

上游 [cal_sens.c][u-sens] 的 `sensor_calibrate_sens` 已按实际样本间隔测量零偏、积分旋转并更新 `gyroSensScale`；[Kconfig][u-kconfig]已有可配置圈数，上游默认5圈。上游Helper [calibration/engine.rs][u-helper-cal]还有逐轴任务与前后值比较，接收器可把命令发给目标或全部设备。因此“自动比例校准”“批量发命令”和“参数回读”不是本项目新增概念。

项目在 [calibration.c](../integration/tracker/src/sensor/calibration.c)、[cal_archive_core.c](../integration/tracker/src/sensor/cal_archive_core.c) 中扩展比例／多姿态观测和归档，在 [continuous.py](../tools/jointcal/continuous.py)、[joint_rigid_v3.py](../tools/jointcal/joint_rigid_v3.py)中实现十台正反20圈与多姿态的连续联合求解。共享角运动，分别估计安装、杆臂及相对时钟，并用信息量与收敛条件约束候选。它改变了现有比例校准的输入、求解及审阅流程；固件运行时仍是每轴单系数。上游已有加速度校准，本项目的新加速度候选尚未与设备已装矩阵完成对照，不能列为已替换或更准的功能。

### 3.2 温度读数、采样桶和点保存

上游 [sensor.c][u-sensor] 已有温度滤波／有效性处理；[tcal_runtime.c][u-tcal] 已存实测均温、按升降温方向EMA混合零偏，提供自动采集和备份接口；[tcal_mls_lut.c][u-lut] 实现MLS/LUT曲线。**“实测温度取代桶中心”和“自动温采开关”不能再用本地旧版行为证明优于当前上游。**

| 细节 | 上游固定源码 | 本项目固定源码 |
|---|---|---|
| 温区／桶／LUT | Kconfig默认10–45°C、每度2桶；LUT每度2格 | [prj.conf](../integration/tracker/prj.conf)设15–43°C、每度5桶；[calibration.c](../integration/tracker/src/sensor/calibration.c)中LUT每度4格；采样上界43°C不含 |
| 累计窗口门槛 | `TCAL_ACCUM_MIN_SAMPLES=2000`、温跨0.53°C、25秒刷新；另有运动范围／稳健块筛选 | `1250`、0.11°C、25秒；另加固定参考面、新鲜加速度和CALWIRE冻结互斥；这些是不同策略，不是单靠样本数判断优劣 |
| 同桶重新保存 | 保存本次实测温度，零偏按方向EMA混合 | 常规EMA时实测温度也用相同权重混合；大幅零偏不一致经多个窗口确认后替换，避免把旧零偏配到新温度 |
| 孤立跳温 | 已有输入有效性与滤波，不使用本项目的确认状态机 | [temperature_guard.h](../integration/tracker/src/sensor/temperature_guard.h)：启动／断流／跳变>1°C时至少3次、跨度≥100ms、范围≤0.2°C才接纳；采集与短时补偿缓存分别处理 |
| 数据与模型管理 | 已有校准锁、代际校验、曲线统一及加热校准相关逻辑 | 保留本地较早的LUT→MLS回退，扩展CALWIRE快照／覆盖工具；没有整体移植上游并发模型或加热功能 |

温表更密、窗口更短及参考姿态约束是可核实的实现差异；既有数据没有证明全温区精度优于上游。细节、测试和门槛见[温度说明](FUSION_UPSTREAM_zh.md)。

### 3.3 采集、重传与普通追踪调度

上游 [connection.c][u-connection] 已有 `connection_request_raw_retransmit`、原始包历史与校准快照；[Receiver esb.c][u-rx-esb] 已追踪原始序号缺口并通过ACK载荷请求补发。单目标可靠采集保留ARQ历史，批量路径不保留该历史。上游也有批量采集接口，不能写成只能单台使用或完全没有可靠传输。

项目增加 [f04_summary.c](../integration/tracker/src/sensor/f04_summary.c)、[f04_capture_ack.h](../integration/tracker/src/sensor/f04_capture_ack.h) 和 [Receiver f04_capture_ack.h](../integration/receiver/src/connection/f04_capture_ack.h)：先把约20ms传感器数据积分成48B窗口，约50Hz发送；接收器成功入队后确认设备、boot、session、seq和CRC，Tracker保留未确认队首。主机由 [f04_summary.py](../tools/f04_summary.py)及结束清单审计首部、中间和尾部完整性。

既有 [Tracker tdma.c](../integration/tracker/src/connection/tdma.c)、[Tracker esb.c](../integration/tracker/src/connection/esb.c)和 [Receiver esb.c](../integration/receiver/src/connection/esb.c)也被修改：F04期间改用较宽时隙、暂停常规姿态、保留控制与心跳，增加采集重同步和会话恢复处理。它不是只加了几个独立文件，也不是全链路统一重传。64窗缓冲约1.28秒；接收器ACK不能证明电脑已落盘，断电与长遮挡仍可能丢失。详见[F04协议](../integration/F04_PROTOCOL.md)。

### 3.4 参数读写、快照和Helper工作流

上游已有`sens`／`tcal`控制、温表备份、采集校准快照，Helper校准引导会检查非目标轴比例和应用零偏是否变化；不能将这些基础检查写成本项目首次提供。

项目新增 [calwire.c](../integration/tracker/src/sensor/calwire.c) 与 [calwire_apply.py](../tools/calwire_apply.py)，将冻结快照、完整设备身份、CRC、证据哈希和逐字段写入关联。只允许规定的S/M/E等操作，保留温表和归档；整组先预检，已是目标状态的设备只核验，失败按本次已知提交尝试回退。这扩大了已有读写功能的范围和状态约束，不提供整组原子提交或密码学设备认证。

上游 [Helper README][u-helper-readme]已经包含终端、采集、VQF回放、温度图、校准引导、OTA和更新检查。项目基于0.4.3，新增 [fleet_calibration](../helper/crates/app/src/features/fleet_calibration/)、[fleet_workflow.py](../tools/fleet_workflow.py)和独立校准运行组件；同时修改已有设备预约、采集入口、导航、温度命令回读、参数审阅及固件更新交互。五步流程管理四份录制与后续分析，回放后可只采用新S并保留旧M/E。上游0.4.6的GUI依赖、更新通知和本地未知板型确认等后续改动没有整体合入，不能用本地版本号冒称已升级。

### 3.5 VQF、地球项与休眠

上游 [vqf.c][u-vqf]及 [Tracker融合适配][u-tracker-vqf] 已有姿态／零偏估计、数值保护和observer／bias policy接口。项目的 [vqf-c/src/vqf.c](../integration/tracker/vqf-c/src/vqf.c)与头文件均不同：加入 `updateAccGated` 接口，选择性纳入倒置修正、协方差对称／非负保护和IIR系数切换公式；没有复制最新上游全部状态字段与策略。

外围 [fusion/vqf/vqf.c](../integration/tracker/src/sensor/fusion/vqf/vqf.c)、[earth_pair.c](../integration/tracker/src/sensor/fusion/earth_pair.c)、[field_calibration.c](../integration/tracker/src/sensor/field_calibration.c)及校准模块增加地球参考、g-sens相关补偿接口、加速度接纳门控和输出参考恢复。新增的是这条实验补偿链及其与已有融合的连接，不能把VQF算法本身算成本项目成果。

上游 [power.c][u-power]已有WoM／唤醒和电源生命周期。项目在 [retained.h](../integration/tracker/src/retained.h)、[power.c](../integration/tracker/src/system/power.c)及 [Styria MCUboot板型](../integration/tracker/boards/styria/styria_mini_mcuboot/)中适配本地启动链和融合状态恢复。历史retained冲突是本项目特定构建的排查结论，未证明当前上游有同样故障。自动深睡仍关闭，没有上游整夜或电流A/B。稳定低通、动态零偏候选仍只在电脑端实验。

### 3.6 OTA协议和启动验证

上游 [esb_ota.h][u-ota]及 [Helper protocol.rs][u-helper-ota]已经支持ESB v1（60B数据载荷、66B tracker FW_INFO），并有板型与MCUboot更新路径。项目 [arc8_esb_ota.c](../integration/tracker/src/system/arc8_esb_ota.c)针对Styria分区使用v2、44B数据及48B tracker INFO；[Receiver esb_ota.c](../integration/receiver/src/esb_ota.c)桥接信息格式，Helper [protocol.rs](../helper/crates/app/src/features/firmware/esb_ota/protocol.rs)增加解析／发送适配。接收器仍保留66B兼容信息结构，不能把tracker空口48B写成所有接口都变48B。

这是已有OTA栈的实质修改，包含匹配版本、板型、分区和激活状态的要求。CRC验证与签名验证不同，MCUboot负责最终签名和启动；PC包检查不能保证实机已经运行新固件。最新Helper交互做过只读查询和模拟传输，不能替代整条OTA实刷验收。

## 4. 移植来源、未合入项与结果边界

| 上游改动 | 本项目处理 | 归属 |
|---|---|---|
| Tracker `a3f9544` 无效FIFO温度、`75fff95` 合法包头 | 选择性移植；另加寄存器读取失败处理与温度确认 | 上游修复加本地扩展，不能整体记为原创 |
| VQF `1d1d867` 数值保护、`6d68ad6` 系数公式 | 提取部分实现，保留本地门控和状态布局 | 部分移植，不等于两版VQF相同 |
| Receiver `109a4d2` 配对满提示 | 移植限频拒绝提示 | 与上游共享修复，不是超越上游的新能力 |
| Tracker NCS v3.4、新板型、校准模块拆分、加热、完整事件生命周期 | 未整体合入；本项目Tracker仍用固定v3.1，Receiver用v3.4 | 版本分叉与功能取舍；项目不是上游超集 |
| Helper 0.4.4–0.4.6的GUI依赖、升级提示、发布和兼容性工作 | 本项目保留0.4.3基础上的已交付扩展 | 不能把本地新增工作流当成已合并当前上游全部功能 |

目前**没有在同一硬件、同一动作与参数条件下，运行这组完整上游源码和本项目的实机A/B基准**。87.05%的回位改善、20.08%的低通实验改善及F04零缺口记录都各有本地对照，详见[结果](RESULTS_zh.md)和[实验](EXPERIMENTS_zh.md)；不能拿它们量化“相对上游提高多少”。文件不同只证明实现不同，收益须由相应测试支持。

## 5. 重现本次审计

取得包含上表提交的五个Git仓库。在含新版审计工具的项目目录运行（最后四项是独立上游仓库，不是本项目的源码副本）：

```sh
python tools/audit_upstream.py --project-repo /path/to/earth-rate-lab-git --tracker-repo /path/to/upstream-tracker --receiver-repo /path/to/upstream-receiver --vqf-repo /path/to/upstream-vqf --helper-repo /path/to/upstream-helper --out .local/upstream-diff.json
```

工具使用[固定配置](../config/upstream-audit.json)中**双方提交**的Git归档，不读取工作树改动，不联网、不改分支。输出schema为2，仅保留 `versus_upstream`；旧版 `versus_base`／`versus_head` 双列已取消。结果应与[公开清单](data/upstream-diff.json)一致；以后项目或上游更新，应同时记录新身份和核查日期再生成新报告。SDK和独立Fusion最新版本不在这四组文件审计范围内，其来源见[许可证与来源](../THIRD_PARTY.md)。

[u-sens]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/src/sensor/calibration/cal_sens.c
[u-kconfig]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/Kconfig
[u-sensor]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/src/sensor/sensor.c
[u-tcal]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/src/sensor/calibration/tcal_runtime.c
[u-lut]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/src/sensor/calibration/tcal_mls_lut.c
[u-connection]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/src/connection/connection.c
[u-rx-esb]: https://github.com/jitingcn/SlimeVR-Tracker-nRF-Receiver/blob/109a4d26ac93e69bc326865fc741960ae13e828d/src/connection/esb.c
[u-helper-cal]: https://github.com/jitingcn/SlimeNRF-Helper/blob/2872946e04c8d625e3eb7f74e8bd7e7954ac9c6d/crates/app/src/features/calibration/engine.rs
[u-helper-readme]: https://github.com/jitingcn/SlimeNRF-Helper/blob/2872946e04c8d625e3eb7f74e8bd7e7954ac9c6d/README.zh-CN.md
[u-vqf]: https://github.com/jitingcn/vqf-c/blob/6d68ad6905bf278ae1b528736f9d3ffccf235e20/src/vqf.c
[u-tracker-vqf]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/src/sensor/fusion/vqf/vqf.c
[u-power]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/src/system/power.c
[u-ota]: https://github.com/jitingcn/SlimeVR-Tracker-nRF/blob/a9b441ccec8a31f5f2903b17663e939a41c265ab/src/system/esb_ota.h
[u-helper-ota]: https://github.com/jitingcn/SlimeNRF-Helper/blob/2872946e04c8d625e3eb7f74e8bd7e7954ac9c6d/crates/app/src/features/firmware/esb_ota/protocol.rs
