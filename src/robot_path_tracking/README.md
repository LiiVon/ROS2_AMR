# robot_path_tracking

路径跟踪：几何跟踪算法（Pure Pursuit / Stanley）与路径发布、基准评测入口。

## 1. 分层结构

```text
robot_path_tracking/
├── include/robot_path_tracking/
│   └── core/        # tracking_math.hpp：预瞄/横向误差/曲率与角速度求解
├── src/nodes/       # path_publisher / pure_pursuit / stanley
└── test/            # 算法单元测试
```

| 目录 | 层次 | 职责 |
|---|---|---|
| include/.../core/ | 算法层 | 纯函数几何计算，不依赖 ROS，可在单测中直接验证 |
| src/nodes/ | 入口层 | 路径发布节点与两个控制器节点（输出 `/cmd_vel/tracking`） |

## 2. 节点与话题

| 类型 | 名称 | 说明 |
|---|---|---|
| 执行 | `path_publisher_node` | 读取路径文件并按固定频率发布参考点 |
| 执行 | `pure_pursuit_controller_node` | 预瞄距离 + 曲率控制 |
| 执行 | `stanley_controller_node` | 前视点 + 航向/横向误差控制 |
| 话题 | `/cmd_vel/tracking` | 跟踪控制器输出（经仲裁与安全闸门） |
| 参数 | `controller`（`pure_pursuit`/`stanley`）、`path_file` | 启动时选择控制器与路径 |

## 3. 启动与使用

```bash
ros2 launch robot_bringup tracking.launch.py controller:=pure_pursuit \
  path_file:=$(ros2 pkg prefix robot_experiments)/share/robot_experiments/paths/loop.txt
```

## 4. 关键约束

- 控制器只发布 `/cmd_vel/tracking`，不直接写 `/cmd_vel`。
- 算法层保持纯函数，节点层只做订阅/参数/定时器，便于单测覆盖。

## 5. 测试

`tracking_math_test`（预瞄、曲率、角速度、边界情况）。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
