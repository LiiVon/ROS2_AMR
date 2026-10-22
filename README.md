# ROS2_AMR

基于 ROS 2 Jazzy 的室内自主移动机器人（AMR）系统：底盘通信 → 传感器标准化 → 安全仲裁 → 诊断闭环 → Nav2/SLAM → 路径跟踪 → 任务调度（站点/设施/充电/车队/订单）→ Gazebo 仿真与可视化演示。

> 本仓库为新版实现（从 0 设计），旧版 `robot/` 仅作功能参考，不参与构建。

## 1. 环境

- Ubuntu 24.04 LTS、ROS 2 Jazzy
- C++17、CMake ≥ 3.20、colcon、ament_cmake
- 依赖：`nav2_bringup`、`slam_toolbox`、`robot_localization`、`twist_mux`、`behaviortree_cpp`、`ros_gz_sim`、`ros_gz_bridge`、`laser_filters`、`xacro`、`controller_manager`、`diff_drive_controller`、`diagnostic_aggregator`

## 2. 构建

```bash
cd /home/lz/robot_ws
source /opt/ros/jazzy/setup.bash
# 若构建时提示 No module named 'em'，把 Debian 的 dist-packages 加入 PYTHONPATH
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
colcon build --base-paths src --symlink-install
source install/setup.bash
```

> `--base-paths src` 必须带：工作区根目录下还有只读参考项目 `robot/`，不加会把两套同名包一起扫描。

只构建部分包：

```bash
colcon build --base-paths src --symlink-install --packages-select robot_hardware robot_tasks
```

## 3. 运行

```bash
source install/setup.bash

# 1) mock 底盘 + 传感器 + 安全链 + 诊断（最小闭环）
ros2 launch robot_bringup bringup.launch.py backend:=mock

# 2) 仅模型与 TF
ros2 launch robot_bringup display.launch.py

# 3) 路径跟踪对比（pure_pursuit / stanley）
ros2 launch robot_bringup tracking.launch.py controller:=pure_pursuit

# 4) 一键自动仿真演示（Gazebo + RViz + 自动任务链）
ros2 launch robot_bringup amr_demo.launch.py gui:=true use_rviz:=true
ros2 launch robot_bringup amr_demo.launch.py gui:=false use_rviz:=false   # headless
```

mock 闭环快速验证：

```bash
ros2 topic echo /chassis/state --once          # backend: mock, connected: true
ros2 topic pub -r 20 /cmd_vel/teleop geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic echo /odom --once                    # 位置持续推进
```

### 无显示器 / 无 GPU 环境

Gazebo 需要可用的 3D 渲染上下文。在无 GPU 的服务器上请按旧版方式启动 VNC 虚拟桌面并 `export DISPLAY=:1`，或使用 `gui:=false`（服务端仍需要渲染传感器，本仓库已为 headless 增加 `--headless-rendering`）。
若机器完全无 3D（如本 VMware 虚拟机），仿真运行时测试会自动跳过；其余功能（mock 链路、导航配置、任务逻辑）不受影响。

### ros2_control 链路

```bash
ros2 launch robot_hardware ros2_control_hardware.launch.py backend:=mock
ros2 control list_controllers
ros2 launch robot_hardware hardware_ekf.launch.py backend:=mock
ros2 launch robot_hardware chassis_simulator.launch.py bind_host:=127.0.0.1 port:=19000
```

## 4. 测试

```bash
colcon test --base-paths src
colcon test-result --verbose
python3 tools/check_conventions.py            # 命名/行数规范
python3 tools/check_conventions.py --strict   # 发布门禁（行数超限也算失败）
```

当前基线：**533 个测试、0 失败、1 跳过**（跳过的为 Gazebo 运行时用例，需要 3D 显示）。

## 5. 接口约定

| 类别 | 示例 |
|---|---|
| 命令链 | `/cmd_vel/nav2`、`/cmd_vel/tracking`、`/cmd_vel/teleop` → `/cmd_vel/muxed` → `/cmd_vel` |
| 底盘 | `/chassis/state`、`/chassis/odom_path`、`/odom`、`/wheel/odom` |
| 安全/系统 | `/system/emergency_stop`、`/system/health`、`/system/fault_state`、`/teleop/active_source` |
| 导航 | `/navigation/map_state`、`/navigation/safety_state`、`/navigation/localization_health`、`/navigation/current_goal` |
| 任务/业务 | `/mission/state`、`/facility/payload_state`、Action `/mission/navigate_sequence` |
| 服务 | `/chassis/set_mode`、`/chassis/clear_odometry`、`/mission/enqueue`、`/facility/reserve_resource`、`/business/submit_order`、`/navigation/list_maps`、`/site/activate` |

完整清单见 `需求文档.md` 附录 B/C。

## 6. 目录

```text
src/
├── robot_interfaces_core           # 核心状态/底盘/统计
├── robot_interfaces_navigation     # 导航/地图/安全 + NavigateSequence
├── robot_interfaces_mission        # 任务/站点/停靠/充电/交通
├── robot_interfaces_facility       # 设施/电梯/门/确认/载具
├── robot_interfaces_fleet          # 车队
├── robot_interfaces_business       # 订单/履约/场景/工作流
├── robot_interfaces_site           # 站点配置
├── robot_hardware                  # 协议/传输/运动学/适配器/ros2_control 插件
├── robot_description               # URDF / 几何 / RViz
├── robot_sensors                   # 激光 / IMU 标准化
├── robot_teleop                    # cmd_vel 仲裁 + 安全闸门
├── robot_utils                     # 系统监控 / 故障监督
├── robot_navigation                # Nav2 / SLAM / 地图 / 语义区
├── robot_path_tracking             # Pure Pursuit / Stanley
├── robot_tasks                     # 任务中枢 / 订单 / 设施 / 车队
├── robot_simulation                # Gazebo / 桥接 / 可视化 / 演示编排
├── robot_experiments               # 实验指标
└── robot_bringup                   # 顶层 launch
```

## 7. 命名规范

- topic/service/action：`/<域>/<名称>`，**不使用 `v2` 等版本前缀**
- 类型/函数 `PascalCase`；变量/成员 `snake_case`（成员尾下划线）
- ROS 句柄：`*_publisher`、`*_subscription`、`*_service`、`*_client`、`*_timer`
- 每个消息类型只有一个定义包（无兼容副本）

详见 `需求文档.md` §4 与 `docs/差异说明.md`。
