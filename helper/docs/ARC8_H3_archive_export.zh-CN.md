# ARC8-H3：Tracker 归档只读导出 → PC 联合校准（实验）

2026-10-31。基线：ARC8-B H1 Tracker/Receiver + SlimeNRF-Helper v0.4.3 ARC8-H2。

## 能力边界

实现了 **Tracker 控制台 `cal archive export` → `ARC8X1` 分块文本 → PC `archive_transport.py` 严格重组及两层 CRC32 校验 → H2 归档 JSON → 同源 C 求解器** 的源码链路。导出代码不写生产 NVS、不安装校准参数、不触发 OTA。没有加入 Receiver ESB/HID 无线归档转发。本轮本机只执行离线 C/Python 双端测试，未构建 Zephyr 固件，也未对真实追踪器/USB/RTT 实测。

输入必须是 ARC7 **完整归档**，不等于 20 分钟原始 IMU 录制、普通 `hidlog`、标准 Tracker Events 或此前生成的结构模板。只有曾经在 Tracker 中使用 ARC7 的归档命令采集并成功持久化 T-Cal、scale x/y/z、Earth 五停点和六面 g-sens 后才能导出。

## Tracker 端改动

- `src/sensor/cal_archive_safe.c`：新增 `export` 命令；仅在归档采集及保存空闲时接受；重新从 NVS 读取并验证已持久化的数据。
- `src/sensor/cal_archive_stage2.inc`：验证 14 个阶段记录及原 T-Cal 记录，从结构化的 NVS 归档组成四组 `CAL_ARC_VERSION=2` 二进制结构，再输出 `ARC8X1` 文本包。
- `src/sensor/cal_archive_wire.[ch]`：纯 C、无 Zephyr 依赖的分包器，24 bytes/行，包含长度、CRC32、引用编号、版本代数和顺序偏移；静态断言四个 ABI 大小。
- `src/console.c`：`cal archive export` 菜单帮助。工程原有 CMake `src/*.c` glob 包含新 C 文件，不用额外改动构建脚本。

实际载体是 Tracker 自己的控制台（可在配有 USB CDC 控制台时使用 COM；或者经 RTT 手动操作并保存完整日志）。它不是 Receiver 的 HID 通道。日志有 `ARC8X1 BEGIN/DATA/END/DONE`，没有 `DONE` 不得当作成功。打印失败以 `ARC8X1 ERROR ...` 表示。

## PC 端命令（Windows PowerShell）

请先**编译并刷入包含 H3 修改的 Tracker 实验固件**。本包没有可直接刷写的成品镜像，本机也没有 NCS/Zephyr 交叉编译环境；只有源码不可能在旧版设备上执行 `cal archive export`。

MinGW GCC 16.2.0、Python 3.10+ 已在用户 Windows 机器上验证可用于 H2。中文路径可能影响 gcc 临时文件，建议配置 ASCII 临时目录；不要在 PowerShell 中无意覆盖其他环境变量：

```powershell
New-Item -ItemType Directory -Force C:\arc8-tmp | Out-Null
$env:TMP = 'C:\arc8-tmp'
$env:TEMP = 'C:\arc8-tmp'
$env:CC = 'C:\your\toolchain\msys64\msys64\mingw64\bin\gcc.exe'
py -m pip install pyserial
```

进入 `helper` 根目录后，通过 Tracker 直连 USB CDC COM 口（**不要选 Receiver HID**）：

```powershell
py tools\arc8_cal_pc\archive_transport.py capture-serial --port COM7 --baud 115200 --timeout 120 --tracker-uid STYRIA-TRACKER-OPERATOR-ID --receiver-key USB-DIRECT-NO-RX --note '真实 ARC7 完整归档，实际采集日期和实验条件' --out observation.arc8.json
```

COM7 仅为示例。`tracker-uid` 与 `receiver-key` 都是**操作员自行声明的字符串，尚非设备证明**，必须按真实情况填写。对于 RTT（没有 Tracker COM 口），手动在 Tracker 控制台输入 `cal archive export`，将整个串口/RTT 会话保存为 `tracker-export.log`，然后：

```powershell
py tools\arc8_cal_pc\archive_transport.py parse-log tracker-export.log --tracker-uid STYRIA-TRACKER-OPERATOR-ID --receiver-key RTT-DIRECT-NO-RX --note '真实 ARC7 归档及原始 RTT 日志留存' --out observation.arc8.json
```

只在完整校验通过后创建 JSON（拒绝覆盖已有路径）。若失败则不会生成伪造的归档输出。

随后调用 **ARC7 同源求解**：

```powershell
py tools\arc8_cal_pc\calibration.py solve observation.arc8.json candidate.cal.json
py tools\arc8_cal_pc\calibration.py verify candidate.cal.json
```

求解器可能因缺少可辨识性或质量不足拒绝归档；这是质量门限而不是传输错误。`candidate.cal.json` 仍是 `OFFLINE_CANDIDATE_UNAPPLIED`，禁止解释为生产校准已更新。

## Windows 修复

按用户反馈，H2 的 `ctypes.CDLL` 在 Windows 上可能在离开 `TemporaryDirectory` 作用域后继续占用 `arc8_cal.dll`，导致清理 `WinError 5`。本阶段在求解器和测试中的临时目录改用 `ignore_cleanup_errors=True`，**只忽略清理异常，绝不跳过 GCC/ABI/求解失败**。Windows 可能暂留临时 DLL，可在所有进程退出后清理。本包未声称在此 Linux 环境中复现 Windows 文件锁；该修复由用户在 MinGW Windows 上确认可行。

## 测试和未检验清单

| 测试 | 状态与证据 |
|---|---|
| ARC8X1 C 生产者→Python 消费者二进制对接 | 本机 GCC/Python **通过**，使用专用合成测试数据，非真实观测 |
| 二层 CRC32、长度、代数、字段 ABI | 本机自动测试通过 |
| 丢包、重复、乱序、两次传输、身份混用等失败关闭 | 本机自动测试通过 |
| H2 本机原生求解和旧测试 | 14/14 通过（本阶段回归） |
| 新 H3 测试 | 12/12 通过（共 26/26） |
| H1 Rust 静态集成门禁 | 11/11 通过；**非 Rust 编译** |
| Rust/Cargo 编译和 Helper 原生界面 | **未检验** |
| NCS/Zephyr Tracker 真正构建和镜像大小 | **未检验** |
| 用户 Windows 上的 H3 pyserial/COM 或 RTT 真实导出 | **未检验** |
| 同一归档 Tracker 端与 PC 端求解数值比较 | **未检验**，缺完整真实归档与设备日志 |
| Receiver ESB → USB HID 无线导出 | **未实现** |
| 设备身份认证、防回放、签名参数包 | **未实现**，只有操作员填入的标签及 CRC |
| 参数事务性 staging、原子 NVS commit、重启读回和回退 | **未实现**，严禁误报已刷入 |
| 10 台并发、掉电/断链/性能压力测试 | **未检验** |

### 本机重现测试

解压完整实验包，保持 `helper` 与 `tracker_receiver` 在同一父级目录；运行：

```shell
cd helper
python -m unittest discover -s tools/arc8_cal_pc/tests -v
```

跨语言测试会直接编译配套 Tracker 原生 C 编码函数；若只取出 Helper 而没有 Tracker 源码，该组测试会明确 **SKIP**，不能算通过。若另放 Tracker，可设置 `ARC8_TRACKER_ROOT` 为 `tracker/ncs/tracker` 完整目录。

## 后续工作

先获得用户真实的**完整 ARC7 NVS 归档**及一份完整 `ARC8X1` 控制台日志；重放通过后检查同一设备 PC 计算的质量指标、数值，再设计 Receiver 分块应答/重传/状态查询路径。原始实时 IMU 录制可独立保留用于 VQF 分析，但不能替代归档。
