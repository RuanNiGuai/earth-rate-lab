# 项目目录、模块与维护方式

公开仓库是整理后的可构建源码集合。私人开发工作区中的历史版本、SDK、录制、密钥和构建缓存不进入仓库；没有为了整理而删除原始证据。旧的发布副本也不被强制重置。

```text
earth-rate-lab/
  README.md                   项目入口与关键结论
  QUICKSTART_zh.md             用户首次使用
  USER_GUIDE_zh.md             操作索引
  ALGORITHM_zh.md              补偿链与约束
  docs/                       当前状态、效果、实验、差异、待办和构建
    data/                     脱敏数值及固定提交差异
    HISTORY_zh.md              只作历史证据，不代替当前状态
  helper/                     本分支桌面Helper完整Rust源码及自身许可证
    crates/app/               GUI、设备所有权、OTA和联合校准入口
    crates/recording/         原始记录／回放及旧单台工具
    crates/firmware-image/    镜像解析与验证
    crates/terminal-core/     终端核心
    crates/tools/             许可清单与维护工具
  integration/
    tracker/                  Styria追踪器固件，含VQF/Fusion依赖源码
    receiver/                 配套接收器固件
    tests/                    协议主机回归
  tools/
    jointcal/                 连续录制分析、刚体求解、比较和计划
    helper_fleet.py           Helper独立组件的任务入口
    calwire.py                无线快照与参数协议
    calwire_apply.py          全组预检、逐字段事务、回读及回退
    fusion_replay.py/.c       实际融合核心的主机回放
    validate_fusion.py        RAWFIFO验证
    release_export.py        发布白名单与清单生成
    audit_upstream.py         固定Git对象的差异审计
  config/                     SDK锁定、公开验签公钥、发布白名单与上游提交
  examples/                   合成数据配置，不含真实设备数据
  tests/                      宿主工具、模型和导出的回归
  SOURCE_MANIFEST.json        此源码快照的逐文件哈希（导出时生成）
```

## 一、谁负责什么

Helper拥有界面、设备选择与独占预约；后台Python组件处理十台流程和数值求解；固件采样、预积分及补偿；接收器负责无线转发。三个层次通过明确协议交互，不能让界面文本替代设备确认。

Helper源码来自已验证交付包的源清单，整理到 `helper/` 后逐文件校验。`config/helper-source-files.json`明确包含必要的隐藏构建文件和合法测试夹具，避免递归打包缓存或漏掉构建输入。上游框架许可证和维护指引继续保留。

## 二、会话生命周期

准备时冻结接收器和十台身份；录制保存原始窗口与结束清单；分析生成分段与时钟映射；求解绑定输入哈希；比较选择字段；审阅计划绑定快照、证据和哈希；安装再次全组预检。已知目标状态只核验，已知中间状态需恢复，未知变化拒绝覆盖。

重录、重算或重新比较会使旧计划失效。失败记录和旧输出仍保留，不在同一目录覆盖原始数据。读回验证与重启验证是两件事。数据层面不为补齐缺口伪造传感器样本。

## 三、开发与测试

根目录执行Python/C离线检查；Helper在自身目录执行Rust工作区检查，具体命令见[构建](BUILD_zh.md)。整理本身不调整已安装固件、数值参数或用户偏好；界面行为变化需原生窗口检查，协议写入使用模拟器与独立授权的硬件验收。

根CI只运行离线检查，不接触硬件。`helper/.github/`是保留的独立Helper工作流来源，嵌套在本仓库下不会自动成为根Actions；不能把它写成已在本仓库完成跨平台构建。

## 四、发布操作

先运行白名单检查，再导出到全新目录。根仓库只放源码和脱敏摘要，大二进制放GitHub Release附件；不要直接把私人工作区 `git add .`。Release里的Helper程序须连同 `calibration/` 和许可证分发；程序、原始源码与测试清单分别记录哈希。

初始公开发布保留上游源码版权，不携带现场参数和签名私钥。公开公钥用于说明原实验固件的验证关系，不能让其他人构建出的固件被现有启动加载器自动信任。不能对不同板型分发同一个应用镜像冒充通用固件。
