# ARC8-H1 原生集成 · SlimeNRF-Helper v0.4.3

此分支为用户提供的 Helper v0.4.3 源码增加一个 ARC8 H1 诊断页面（导航栏“ARC8 H1 只读诊断”）。

## 范围与设计

- **无额外 HID reader / 句柄**：直接复用原生 `discovery::ReceiverMonitor`、`receiver_events::Watch` 与 `hid_log` TraceHub。
- VID/PID `1209:7690` + usage page 1 仅能识别兼容 HID 接收器，**不能认证 ARC8 固件身份、版本或设备配对完整性**。
- 订阅按钮调用现有 `monitor.set_events_enabled`，**只会由已有 Watch 生成 224 号事件查询/订阅/续租/停止报文**；不发送 210 INFO、206 LIST、校准执行、OTA 或 NVS 写入报文。事件租约会影响接收器 RAM 中的订阅状态；因此属于只读诊断而非完全零控制命令。
- 设备列表只显示接收器实际观察到的无线注册/传感器数据，不声称是完整配对列表或固件版本。
- 事件显示来源（Tracker / Receiver）、tracker id、operation id、event seq、phase、outcome；过滤不符合 ARC8-B 事件校验基本规则的报文，对重复包去重（按接收器实例/boot nonce/seq/operation 等字段），保留最多 256 条，页面显示最近 80 条。
- 接收器合成 UNKNOWN 只能显示 UNKNOWN。**Tracker END SUCCESS 和 STORAGE phase 均不能证明 NVS 持久化成功**。H2 需增加明确的持久化操作号/状态查询机制。
- 打开页面时订阅 TraceHub，离开页面停止本页面 Trace 监听；先前在接收器启用的事件租约不会随页面切换自动关闭，可返回页面手工停止，或在原 HID 日志页停用。程序退出时沿用原有接收器清理逻辑。

## Windows 构建

在已安装 Rust >=1.99、Windows 编译环境且 Cargo 依赖可解析的机器中：

```powershell
cargo test -p slimenrf-helper --bin slimenrf-helper arc8_h1
cargo check -p slimenrf-helper
cargo build -p slimenrf-helper --release
```

实际 package 名称以 `crates/app/Cargo.toml` 为准；上述命令需要自行检查后使用。生成的可执行程序不得等同于已通过真实 USB/事件功能验收。

## 实测清单

1. 先以已知旧 Helper 对相同接收器确认 USB 枚举及 HID 日志仍正常。
2. 启动定制版，打开 ARC8 H1 页面：显示候选接收器和已观察到的 Tracker，而非完整配对表。
3. 单台追踪器执行非破坏性校准/静止状态观察；开启事件订阅，核对 `224` 的 QUERY/SUBSCRIBE/RENEW/STOP ACK；观察同一操作的 `operation_id`、`event_seq`。
4. 断开重连接收器，核对暂停/重新订阅、去重、不把接收器 UNKNOWN 当成功。
5. 检查原 Helper 的 OTA 页面仍原样存在，但**不要在本阶段刷写**；验证多接收器绑定身份、权限错误、停止订阅和退出行为。

## 未完成

- Rust/Cargo 编译、Windows GUI / 真机 HID 及设备身份认证（本交付环境未安装 Rust 工具链）。
- ARC7 联合归档执行回执、NVS 持久化证据及主动状态查询（H2）。
- ARC8 MCUboot 签名 OTA 图形化控制、升级确认与回退（H4，仍不得启用）。
