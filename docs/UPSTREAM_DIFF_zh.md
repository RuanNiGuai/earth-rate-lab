# 与 jitingcn 上游的差异审查

核查日期：2026-11-12。使用 GitHub 公开 API 读取默认分支，随后抓取 Git 对象比较；不是只根据发布说明猜测。精确提交固定在 [upstream-audit.json](../config/upstream-audit.json)，逐文件状态、行数和 SHA-256 在[差异清单](data/upstream-diff.json)。后续上游移动分支不会改变本次结论。

## 一、核对了哪些仓库

| 仓库 | 本次默认分支 HEAD | 相对上次审查 |
|---|---|---|
| [Tracker](https://github.com/jitingcn/SlimeVR-Tracker-nRF) `dev` | `a9b441ccec8a31f5f2903b17663e939a41c265ab` | 未变化 |
| [Receiver](https://github.com/jitingcn/SlimeVR-Tracker-nRF-Receiver) `dev` | `109a4d26ac93e69bc326865fc741960ae13e828d` | 未变化 |
| [VQF C](https://github.com/jitingcn/vqf-c) `cmsis-dsp-optimization` | `6d68ad6905bf278ae1b528736f9d3ffccf235e20` | 未变化 |
| [Helper](https://github.com/jitingcn/SlimeNRF-Helper) `main` | `2872946e04c8d625e3eb7f74e8bd7e7954ac9c6d` | 在 `7c3bca9` 后新增界面依赖升级和稳定版发布两次提交 |

Tracker 与 Receiver 的本地导入 ID `06ce018…`、`46f0f6d…` 是私人历史整合快照，**不能当成 jitingcn 公开仓库的共同祖先**。所以本次对它们做“当前公开源码快照与本地发布文件”的内容比较，而不伪造 Git ahead/behind。VQF 亦保留历史基线树 ID，当前比较使用上游可获取提交。Helper 额外对记录的源基线 `3d43dd7…` 做比较。

## 二、差异如何解释

文件比较将 CRLF/LF 统一；Tracker 的 `vqf-c`、`Fusion` 子目录与生成 `VERSION` 单独处理，不把平铺子模块误计成新增功能。差异清单里的 `removed` 表示本地发布文件没有上游对应文件，可能是历史导入范围、其他板型／SDK支持或发布筛选差异，**不等于本轮删除了这些功能**。文件行数也不是功能收益评分。

| 本地发布内容与对应上游快照 | 新增文件 | 内容修改 | 上游有／本地未纳入 |
|---|---:|---:|---:|
| Tracker 对当前 HEAD | 157 | 152 | 516 |
| Receiver 对当前 HEAD | 9 | 25 | 3 |
| VQF 对当前 HEAD | 0 | 2 | 0 |
| Helper 对记录基线 `3d43dd7` | 34 | 41 | 0 |
| Helper 对当前 HEAD | 35 | 57 | 1 |

统计包含本分支说明文档。Tracker／Receiver 的基线列采用上次审查快照，与本次 HEAD 相同；不能由此推导最初导入以来的提交数量。

## 三、已选择性移植的内容

| 上游内容 | 本地处理 | 收益与限制 |
|---|---|---|
| Tracker `a3f9544`：温度 `INT16_MIN` | 已移植，并补充寄存器失败返回无效值 | 避免污染均温；不代替实际温度精度校准 |
| Tracker `75fff95`：ICM 高精度 FIFO 包头 | 接纳合法 ODR 标志变化 | 不因低两位变化丢掉合法传感器样本；需要区分设备漏样与无线缺包 |
| VQF `1d1d867`：倒置与协方差数值保护 | 选入接近180°轴方向、协方差对称／非负保护 | 改善特定数值边界；不是完整正半定或任意异常输入保证 |
| VQF `6d68ad6`：系数变换公式 | 仅提取所需公式 | 保持恒定输入下状态连续；未整套引入观测器、自适应 tau 或全部新状态 |
| Receiver `109a4d2`：配对存储满提示 | 已移植，限频提示 | 满容量时明确拒绝，新／旧身份行为分开 |

具体机制和测试在[融合与温度移植说明](FUSION_UPSTREAM_zh.md)。本项目叠加的 CALWIRE、F04 可靠采集、共享地球方向、刚体联合求解与 Helper 工作流不能通过直接检出上述提交得到。

## 四、此次发现的 Helper 新改动

本地保留基于0.4.3的已验证扩展；上游0.4.4/0.4.5预发布及后续稳定发布没有整体覆盖本地源码。

| 提交／功能 | 值得之处 | 本次决定与原因 |
|---|---|---|
| `abb1c57`：本地未知 bootloader 镜像的显式风险确认与 Dawn Mini 元数据 | 对更多本地镜像更友好 | 当前 Styria 校准流程不依赖；仍保留现有严格兼容检查，单独合入需对授权、板型和回退流程回归 |
| `4db16ec`：自动更新通知 | 用户可见性更好 | 本分支还需自己的更新渠道与校准组件绑定，不能沿用官方升级覆盖定制程序 |
| `bccda33`、`22a73a5`：GPUI Kit / Fast | 继承框架维护修正 | 有价值但不是本次两个已修复问题的根因；需随锁文件、notices及Windows/Linux原生界面一起验证，暂不整包升级 |
| `72215cd`、`7c3bca9`、`2872946`：预发布／稳定发布工具与CI | 更完整的发布流程 | 使用本仓库独立实验版标签；不能冒称本地已经是上游新版，或直接发布上游官方产物 |

这轮是整理、审查与发布已验证功能，不以文档维护名义换掉融合算法、SDK或界面依赖。暂缓项目已进入[待办](ROADMAP_zh.md)，不是遗漏。

## 五、其他未整体合并的能力

上游新增板型、加热器、外接磁力计、不同 SDK 的收益依赖具体硬件；当前设备为无磁 Styria Mini，不能宣称已支持所有上游板型。本地 retained 内存与两级启动加载器有关，盲目合并复位／睡眠路径可能破坏姿态连续性。

统一温补网格缓存、自适应零偏／输出观测器、完整电源事件无线生命周期值得继续评估，但分别涉及采样线程读写归属、滤波状态兼容与重启会话边界。当前参数默认仍关闭自适应 tau 和软 MBE 实验开关。

## 六、复现差异审计

先自行 clone/fetch 上表四个仓库；然后在本项目根目录运行：

```sh
python tools/audit_upstream.py --tracker-repo /path/to/tracker-git --receiver-repo /path/to/receiver-git --vqf-repo /path/to/vqf-git --helper-repo /path/to/helper-git --out .local/upstream-diff.json
```

工具只读本地 Git 对象及发布文件，不联网、不改分支、不读取凭据。输出的提交固定，不是自动跟随当前 HEAD；要做新审查，先更新固定提交与日期，重新核验后再提交清单。
