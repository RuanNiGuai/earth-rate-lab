# 无线校准资料与参数传输

CALWIRE v1 通过接收器 USB 控制台与已配对追踪器通信。追踪器使用电池即可读取三轴归档、现行比例/地球自转/g-sens 参数、完整温补点和运行 LUT。它不要求重新做校准动作，也不改变已有温度范围与休眠修复。

每次传输绑定追踪器编号、RF 地址和随机会话号。请求和响应均有 CRC32，响应还回显请求摘要；完整导出另有整体 CRC32，并在电脑保存 SHA256。超时重发不能满足其他请求。导出在设备上冻结为只读快照，三轴归档包含双份存储记录，由电脑按参考编号、版本、CRC、代次选取。

设备拒绝在原始采集、固件升级、校准动作或自动温补采集中开启会话。会话期间暂缓进入休眠和开启自动温补，最后一个请求后 60 秒自动释放。原有参数、温补开关和自动温补状态不由导出命令修改。USB 或无线的其他归档操作在会话期间被拒绝。

写入流程为：保存快照，暂存待写系数，检查完整性和数值范围，核对备份参数与当前参数一致、温补表未变，再由传感器线程在静止状态提交。每次提交仅修改一个生产存储记录，并由固件读回核验。三个记录不是一个原子事务，电脑必须保存写入日志和恢复资料；不能把接收器“已排队”当作存储成功。重启后必须重新导出以核对持久性。地球参考仍需要与校准定义的参考姿态对应，不能在任意放置方向重启后宣称方向已经正确。

`tools/calwire.py` 提供读取备份；`tools/calwire_apply.py` 提供受审查计划的安装、验证和回退。传输协议通过校验不代表校准解可靠。CW2 是固件迭代名，线帧中的协议版本仍为 1。

协议包含 BEGIN、READ、PUT、CHECK、APPLY、END 六种固定操作，不提供任意命令、任意 NVS 写入或固件区域访问。线帧定义见追踪器的 `calwire_protocol.h`；接收器采用相同头文件。收到的射频地址和 CRC 用于防止错发及数据损坏，不是密码学身份认证。

## 加速度坐标与恢复边界（CW6）

线协议仍为 v1，新增可选只读 TLV 4 v1（52 字节）：版本、坐标编号、六面校准开关、活动值与持久化一致标志，以及 3 个 bias 和行优先 3×3 矩阵。坐标为驱动轴、单位 g；`a_cal=A(a_raw-b)`，g-sens 项是 `M a_cal`，随后乘 S。旧快照可读，但加速度变换明确标为未知。

新原始 M 必须连同 B 做仿射变换；当前不提供 M/B/加速度原子事务，所以计划生成只允许经比较的比例更新、保留 M/E。`--trial` 不能绕过坐标不兼容。完整拟合仍可生成诊断审阅。

整组显式回滚也先核对全部设备。备份与 journal 先同步落盘；主机锁由内核租约保护，进程退出后仅回收已确认死亡的持有者，活动或未知持有者拒绝抢占。固件出现 NVS 结果不确定时，直到重启都拒绝再次读取／写入本次会话的参数，避免将旧 RAM 当作持久化结果。

## 命令行闭环

先按[批量采集](BATCH_CALIBRATION_zh.md)创建包含自己配对身份的会话。备份命令中的build必须与目标设备实际版本完全一致；以下是原试用组合的示例，源码候选具有自己的build ID。

```sh
python tools/calwire.py --session .local/run --out .local/backups --slots 0 1 2 3 4 5 6 7 8 9 --build 06ce018fa6d1_wom3_t02_on_cw2
```

每台得到snapshot.bin、backup.json和三轴记录。备份是冻结的校准快照，不是整个Flash镜像；自动温补采集进行中时会话会拒绝，需先关闭采集并确认保存。已有输出不会覆盖。

通过[联合求解](JOINT_SOLVER_zh.md)取得结果后，审查逐台质量、参考姿态和数据来源，再生成安装计划。计划生成器只写本机文件，不接触硬件。它绑定快照、结果与会话身份；合成数据不能生成安装计划。

```sh
python tools/jointcal/make_plan.py --config .local/inputs/input.json --result .local/result/result.json --session .local/run --out .local/reviewed-plan --selection .local/comparison/selection.json --reference-confirmed --authorization "已审查该结果并同意安装"
```

如果所选字段门槛没有通过，该命令拒绝。先在 Helper 或比较工具生成当前源码和已安装加速度矩阵绑定的 `selection.json`；旧格式快照需重新只读备份。完整原始 M/E/B 候选即使拟合门槛通过也不生成安装计划。`--reference-confirmed`仅在设备确实处于原+X起始面和原航向时使用。自动门槛不能替代独立精度审查。

```sh
python tools/calwire_apply.py --plan .local/reviewed-plan/plan.json --session .local/run --out .local/install --mode apply
python tools/calwire_apply.py --plan .local/reviewed-plan/plan.json --session .local/run --out .local/readback --mode verify
python tools/calwire_apply.py --plan .local/reviewed-plan/plan.json --session .local/run --out .local/rollback --mode rollback
```

三条命令分别执行安装、只读核对和恢复原参数，不应把它们当作每次都依次执行的固定流程。回退恢复本计划中的S/M/E，不改温补和原始归档。每次使用新的输出目录，保留plan、evidence、before.bin、expected-after.bin和journal.jsonl。

正常结束为退出码0；无效输入、身份／哈希不匹配、超时、未知设备状态或回读失败为非零。安装异常会尝试按已知提交前缀回退，回退失败也记录日志；未知并发变更拒绝覆盖。主机断电后不会自动恢复流程，应保留日志、重新检查现场和快照，再显式rollback或处理未知状态。

**verify不重启设备，也不证明设备已经重启。** 在原参考面与航向下完成实际重启，确认启动标识／运行时长已变化后，再执行verify，才能把两份证据合称“重启后回读”。普通Earth有效标志不等于Earth pair当前正在运行。
