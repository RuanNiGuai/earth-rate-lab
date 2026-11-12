# 示例

`fleet.example.json`使用虚构设备身份，复制到私有目录后替换为自己的接收器序列号和配对RF信息。不要把真实设备配置提交进仓库。

完整十台合成求解数据通过 `python tools/jointcal/example.py --out .local/example` 生成。用 `python tools/jointcal/joint_rigid_v3.py --config .local/example/input.json --out .local/example-result` 求解。它是数学演示，不是实际温度／运动精度验收。
