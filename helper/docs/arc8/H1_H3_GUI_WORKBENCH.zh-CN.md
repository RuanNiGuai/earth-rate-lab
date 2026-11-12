# ARC8 H1–H3 原生图形工作台：功能核查与使用说明

基线：用户交付的 `SlimeNRF_Helper_v043_ARC8_H3_完整源码.zip`。本次只修改 Helper，不改 Tracker/Receiver，不刷写设备。

## 已有与新增功能对应

| 功能 | 源码证据 | 现在的 GUI 状态 | 验证性质 |
|---|---|---|---|
| 接收器监视、H1 Tracker Events | `features/arc8_h1/{mod.rs,application.rs,evidence.rs}` | ARC8 工作台原有事件订阅/事件日志 | 原有行为；新版本未跑真机 |
| Raw Capture | `features/capture` | 工作台打开原生采集页；不是自建高频流 | 原有 Helper 功能，不等于 10 颗高频无损 |
| VQF Replay | `features/replay` | 工作台打开原生回放页 | 原有 Helper 功能，未加入 ARC8 MBE 算法 |
| H3 四组归档导出 | `tools/arc8_cal_pc/archive_transport.py` | 新增选日志、填身份、解析并保存结构化 JSON | H3 严格 CRC/序号/身份检查；非无线归档 |
| H2 ARC7 同源 PC 求解 | `tools/arc8_cal_pc/calibration.py` + vendored C | 新增选择 JSON、求解并保存候选包 | 原始同源 C 求解，需真实完整归档 |
| 候选参数包验核 | `calibration.py verify` | 新增选候选包、检查结构和 SHA-256 | 不验证追踪器加载/NVS |
| 十设备刚体同步采集 | Foundation v2 设计 | 本版无操作按钮 | Tracker 批量联动未实现/验收 |
| 高精度全速采样时间戳 | Foundation v2 设计 | 本版无操作按钮 | 固件需新增硬件采样时标 |
| 校准包写回/NVS 回读/回滚 | Foundation v2 设计 | **没有写入按钮** | 协议未实现，不能用旧控制命令伪装事务 |
| OTA | 原 Helper OTA 界面 | 原功能不变，工作台不触发 OTA | ARC8 实机 BEGIN 仍存在下行阻断 |

## Windows 使用

1. 用本版完整源码重新编译 Helper。原版 EXE 不会自动出现新菜单：

   ```powershell
   .\build.ps1 -Action build -Release -NoFirmware
   ```

2. 左侧打开 **ARC8 联合校准工作台**，也可以跳转到 Raw Capture/VQF Replay/HID Log。
3. 在 Tracker H3 控制台运行 `cal archive export`，完整保存 `ARC8X1 BEGIN/DATA/END/DONE` 输出。不能用普通 20 分钟 IMU 波形冒充这份四组 ARC7 校准归档。
4. 选择原始控制台日志，填写**人工核实**的设备 UID、接收器标识和动作来源，选择不存在的输出 JSON 路径。通过校验时生成 H2 归档。
5. 选择解析后的归档 JSON，选择未存在的候选包路径，点击联合求解。C 求解器质量或可辨识度不通过会拒绝生成结果。
6. 选择候选包 JSON，点击校验，仅验证离线完整性。NVS 始终标记 **未验证**。

运行时安装 **Python 3**；求解器需 GCC/Clang。默认 Windows 使用 `py -3`，可设置 `ARC8_PYTHON` 指定解释器。编译 Helper 本身不需要 Python。对于中文 Windows 用户目录造成的 MinGW 临时文件问题，先为 `TMP`/`TEMP` 设置 ASCII 目录，并确保 `gcc.exe` 在 PATH 或通过 `CC` 指定。

文件处理有大小上限，不覆盖已有文件，失败不生成伪造的校准包；Python 工作子进程限时 90 秒。设备身份由人工提供，尚未获得设备侧签名或固定 UID 的自动验证回执。

## 尚未检验/验收

- **尚未执行 Rust 1.99.1 编译或 Windows GUI 原生运行**（本交付环境无 Rust/Cargo 和 Windows）。新代码和现有 GPUI 组件的类型与布局需在 Windows 构建中检查。
- Windows Python 3 + MinGW GCC 端到端通过 **GUI 点击** 调用尚未检验；旧的 H2/H3 CLI 已在用户本机完成 26/26 回归。
- 实际 H3 控制台日志导出、真实数据成功求解和跨设备回放尚未检验。
- 10 设备批量无线操作、可靠时间同步、参数安全写回、重启读回和回退尚无生产端后端。
- EXE 嵌入的 C 求解器是否与目标 Windows 运行时的 GCC 匹配，待用户本机验证。

## 本次安全边界

- 不使用额外的 USB/HID reader；H1 复用已有接收器管理及事件订阅。
- H3 只接受完整、经 CRC 验证的四组记录，身份采用用户显式输入。
- H2 只能生成 `OFFLINE_CANDIDATE_UNAPPLIED` 状态参数包。
- 所有执行通过后台任务，不能从此页触发 MCUboot、校准执行、OTA、NVS 写入或参数安装。
