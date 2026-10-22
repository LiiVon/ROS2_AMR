# robot_simulation

仿真与演示：Gazebo 场景、桥接与实体控制、平面里程计真值、可视化与演示编排。

## 1. 分层结构

```text
robot_simulation/
├── include/robot_simulation/
│   └── core/        # planar_odometry.hpp
├── src/
│   ├── core/        # planar_odometry.cpp
│   ├── gazebo/      # 实体桥接、TF、取消暂停
│   └── demo/        # 可视化、演示导演、双车演示
├── worlds/  media/  rviz/   # 场景、材质、可视化配置
├── config/          # 物理参数、仿真区域、站点
├── launch/          # sim.launch.py
└── test/            # 里程计与启动参数测试
```

| 目录 | 层次 | 职责 |
|---|---|---|
| include/.../core/ + src/core/ | 算法层 | `planar_odometry`：平面运动积分（可单测） |
| src/gazebo/ | 桥接层 | 实体位姿/速度注入、里程计 TF 发布、仿真时钟控制 |
| src/demo/ | 展示层 | RViz 可视化、演示流程编排、双车协同演示 |
| worlds/ + config/ | 资源层 | 世界文件、物理与站点配置 |

## 2. 节点与启动

| 类型 | 名称 | 说明 |
|---|---|---|
| 执行 | `amr_sim_gazebo_entity_bridge_node` | ROS 指令 → Gazebo 实体运动 |
| 执行 | `gazebo_odom_tf_node` | 仿真里程计与 TF |
| 执行 | `gazebo_unpause_node` | 场景就绪后自动取消暂停 |
| 执行 | `amr_sim_visualizer_node` | RViz 可视化（目标、路径、状态） |
| 执行 | `amr_sim_demo_director_node` | 演示流程编排 |
| 执行 | `robot2_fleet_demo_driver_node` | 双车协同演示驱动 |
| 启动 | `sim.launch.py` | 场景 + 桥接 + 可视化一键启动 |

## 3. 启动与使用

```bash
ros2 launch robot_simulation sim.launch.py
ros2 launch robot_bringup amr_demo.launch.py gui:=true
```

## 4. 关键约束

- Gazebo 需要可用的 3D 渲染上下文（GPU 或虚拟桌面）；headless 模式已启用 `--headless-rendering`。
- 仿真里程计仅用于展示与对比，控制回路以硬件/模拟后端里程计为准。

## 5. 测试

`planar_odometry_test` 与仿真启动参数检查。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
