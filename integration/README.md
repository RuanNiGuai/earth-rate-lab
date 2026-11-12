# 固件源码入口

`tracker/` 与 `receiver/` 是当前维护的源码；发布时包含完整源码副本及第三方库，不依赖本机嵌套 Git 历史。

- [当前版本和剩余验收](../docs/STATUS_zh.md)
- [固定 SDK、构建和签名](../docs/BUILD_zh.md)
- [F04 摘要协议](F04_PROTOCOL.md)
- [CALWIRE 无线协议与安装](../docs/CALWIRE_zh.md)
- [离线验证入口](../tools/check.py)

截至2026-11-12，实验设备已部署 Tracker `e66e938449ed_cw5_f04ack` 与 Receiver `1719ef40fccb_cw4_f04ack`，并完成重启与长录制检查。不同构建、历史CW2记录与主机算法候选不能混为同一部署证明；最新Helper OTA修复仅做实际只读查询及模拟传输，本轮没有重新刷机。历史现场记录保留在维护者工作区，公开仓库提供脱敏摘要。
