# ARC8-H2 电脑端联合校准与安全写入设计（实验）

## 确认的现状

1. ARC8-B Tracker `sensor/cal_archive_core.c` 中已经有完整的 `cal_arc_solve()` 求解器。
   `cal_archive_stage2.inc` 当前在 Tracker 工作线程上组织 Earth 5 停点、
   GSENS 6 姿态、比例 X/Y/Z 20 圈及 TCal 观测，只输出候选值；**不会**自动保存生产校准。
2. 归档原始观测现保存在 Tracker NVS 的多条记录中。`cal archive solve` 只打印
   候选结果，**现有 HID 事件与 Info ACK 不能充当完整原始观测导出**。
3. 已有 `sensor_gyro_scale_set`、`sensor_gyro_gsens_set`、`sensor_earth_reference_set`
   分别更新不同生产配置，但并无一次性全部生效、失败回滚、重启后整体生效的事务。
4. 新增的 `tools/arc8_cal_pc` 是 PC **离线求解与候选校验**，不含写入指令。

## 目标流程

```text
Tracker: 原始传感器采集 + 质量筛选 + 完整归档
      ↓ 受校验的只读导出（待实现）
Helper / PC: 同源 ARC7 C 求解 + 拟合质量诊断
      ↓ candidate.cal.json（目前已实现）
Helper: 人工审阅、绑定设备/板型/参考姿态、冻结会话（待实现）
      ↓ 分块写入临时 NVS 槽（待实现）
Tracker: 完整性/范围/硬件身份校验 + 读回哈希（待实现）
      ↓ 显式提交新一代校准，不修改旧代（待实现）
Tracker: 重启 + 回报当前已加载 generation / hash（待实现）
      ↓ PC 二次读取对照（待实现）
完成；失败则读取旧一代参数
```

**“刷入校准参数”与“刷 MCUboot 固件镜像”必须严格分离。** 不使用 ESB OTA
镜像 `BEGIN/DATA/END` 写生产校准，也不绕过 MCUboot 签名链。

## 建议的 ARC8 CAL-IMPORT v1（尚未实现的协议草案）

- `CAPABILITIES`：设备明确返回协议版本、固件构建/板型、硬件 UID、支持的数据族、
  当前生效 generation/hash、参考姿态与允许的写入大小。`0x1209:7690` 本身不能证明 ARC8 身份。
- `EXPORT_BEGIN / READ / END`：冻结本次归档，输出实际 NVS 记录、单条 CRC、
  `reference_id`、采集 epoch、传感器单位、样本完整性及来源版本；PC 先验证每条记录。
- `STAGE_BEGIN / CHUNK / END`：仅针对**单一 Tracker**，一次一会话；数据块<=44 B
  （不能和 OTA 的 44B 协议直接共用操作码）；每块序号、运行校验与最终 SHA256。
  传输中断不能污染当前配置。
- `VALIDATE`：Tracker 从 staging 读回并检查长度/哈希、数据是否有限、尺度
  0.9–1.1、g-sens 的范围、Earth 向量/纬度、配置版本、板型及参考身份；无效即拒绝。
- `COMMIT`：仅在使用者确认后更新**独立的 generation 指针/记录**；旧 generation
  保留，支持断电恢复和主动回退。真实设备会话需要不可重放的绑定/授权，不可仅依靠
  易变化的 receiver-local tracker slot 或未经认证的文件哈希。
- `REBOOT_VERIFY`：设备实际重新加载并回报 generation、校准包 digest、各模块有效状态；
  PC 端查询一致才显示「NVS 已持久化并重启验证」。发送成功、ACK、END SUCCESS 都不够。
- `ROLLBACK`：人工或启动健康检查失败恢复上一个有效 generation。

对生产固件要求：要么统一从单个 versioned 校准快照加载全部生产参数，
要么在启动期间先加载整体 snapshot 再**原子地**改变生效 generation；
不应把三个旧 NVS ID 的分别写入误当成原子操作。

## 验证清单（不能把未运行项目标记通过）

| 工作项 | 状态 | 验证依据/剩余任务 |
|---|---|---|
| PC 端复用原 C `cal_arc_solve` | 已实现，本机编译 | GCC + ctypes 尺寸/偏移审计 |
| 输入集合与 `reference_id` 拒绝规则 | 已实现，单元测试 | 严格 schema 检查 |
| 拟合拒绝/禁止产生伪造系数 | 已测试 | 占位数据被原 C 求解器拒绝 |
| 本地候选 SHA256 完整性校验 | 已实现，单元测试 | 文件防误修改，非数字签名 |
| Helper H2 操作证据只读台账 | Rust 源码已接入 | 尚未 Cargo 编译/GUI 运行 |
| Tracker 归档完整只读导出 | 未实现 | 需建立独立数据通道及 NVS 版本解析 |
| 使用真实采集结果在 PC 成功拟合 | **未检验** | 无完整真实归档样本 |
| PC 与 Tracker 算法结果逐数比对 | **未检验** | 需要同一份真实观测/不同 ABI 交叉验证 |
| 可靠写入、原子提交、断电回退 | **未实现/未检验** | 需要改 Tracker 固件及接收器协议 |
| Rust 编译、Windows EXE、USB 通信 | **未检验** | 本环境无 Rust/Cargo 及设备 |
| Zephyr/NCS 构建和 MCUboot 签名 | **未检验** | 需目标工具链与真实固件产物 |
| NVS 重启后读回确认 | **未检验** | 需引入新协议及真机验收 |

## 下一轮明确任务

优先做 Tracker 的只读、带 CRC 的归档导出与 Helper 的接收/组包模块；
以一台 Styria 的真实完整数据验证 PC 同源求解和 Tracker 候选比对。
只有验证数据及设备身份后，再增设独立的 staged-import/commit 子系统。
