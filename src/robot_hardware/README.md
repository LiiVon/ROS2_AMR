# robot_hardware

## 职责

文本/Mick 二进制协议编解码、Mock/Serial/UDP 后端、差速与麦轮运动学、系统适配器、ros2_control 插件、驱动/模拟/里程计路径节点。

## 主要产物

`chassis_driver_node`、`chassis_simulator_node`、`odom_path_recorder_node`、`robot_hardware_backends`、`robot_chassis_hardware`

## 依赖关系

上游：接口包；下游：teleop/description/仿真

## 使用

```bash
colcon build --base-paths src --packages-select robot_hardware
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
