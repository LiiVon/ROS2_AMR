# robot_hardware

底盘硬件抽象层：统一「协议 → 传输 → 运动学 → 系统适配 → ROS 接口」五级分层，同时提供 ros2_control 插件与独立驱动节点。

## 1. 分层结构

```text
robot_hardware/
├── include/robot_hardware/
│   ├── protocol/        # 报文字段与编解码
│   ├── transport/       # 通信后端抽象与实现
│   ├── kinematics/      # 运动学模型
│   ├── system/          # 底盘系统适配（状态机/看门狗/指令换算）
│   └── ros/             # ros2_control SystemInterface 实现
├── src/
│   ├── protocol/  transport/  kinematics/  system/  ros/
│   └── nodes/           # 可执行节点入口
├── config/              # 标定 / EKF / 控制器参数
├── launch/              # 硬件、仿真器、ros2_control 启动
└── test/                # 协议/运动学/适配器单元测试
```

| 目录 | 层次 | 职责 |
|---|---|---|
| include/.../protocol/ | 数据层 | `packet.hpp`：文本协议与二进制帧的结构定义、编解码、越界饱和 |
| include/.../transport/ | 传输层 | `backend.hpp` 抽象 + `mock`/`serial`/`udp` 三种实现 + 工厂 |
| include/.../kinematics/ | 算法层 | `kinematics.hpp`：差速/麦轮等模型的轮速与机身速度互算 |
| include/.../system/ | 业务层 | `adapter.hpp`：连接状态机、看门狗、指令限幅与里程计积分 |
| include/.../ros/ | 适配层 | `hardware_interface.hpp`：`hardware_interface::SystemInterface` 实现 |
| src/nodes/ | 入口层 | `chassis_driver_node`、`chassis_simulator_node`、`odom_path_recorder_node` |

## 2. 接口与节点

| 类型 | 名称 | 说明 |
|---|---|---|
| 话题 | `/chassis/state` | 连接状态、后端名称、错误计数 |
| 话题 | `/odom`、`/wheel/odom`、`/chassis/odom_path` | 里程计与轨迹 |
| 服务 | `/chassis/set_mode`、`/chassis/clear_odometry` | 控制模式切换、里程计清零 |
| 参数 | `backend`（`mock`/`serial`/`udp`）、`port`、`bind_host` | 后端选择与链路配置 |
| 插件 | `robot_hardware::ChassisHardwareInterface` | ros2_control 系统插件（`robot_hardware_plugins.xml`） |
| 执行 | `chassis_driver_node` | 独立驱动节点（非 ros2_control 链路） |
| 执行 | `chassis_simulator_node` | UDP 底盘模拟器，用于无硬件联调 |
| 执行 | `odom_path_recorder_node` | 里程计轨迹录制 |

## 3. 启动与使用

```bash
ros2 launch robot_hardware chassis_simulator.launch.py bind_host:=127.0.0.1 port:=19000
ros2 launch robot_hardware hardware.launch.py backend:=udp
ros2 launch robot_hardware ros2_control_hardware.launch.py backend:=mock
```

## 4. 关键约束

- `/cmd_vel` 不直接下发：驱动只接受仲裁后的最终指令，安全闸门是唯一发布者。
- 协议解码必须容忍历史帧长（超限饱和、非法长度丢弃），不得抛异常退出。
- `mock` 后端用于无硬件验证；`serial` 需要真实串口设备权限。

## 5. 测试

`chassis_protocol_test`、`chassis_kinematics_test`、`chassis_adapter_test`（`colcon test --packages-select robot_hardware`）。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
