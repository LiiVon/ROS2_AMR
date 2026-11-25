# robot_utils

通用基础设施：时间/几何工具、话题名常量、系统监控与故障监督。

## 1. 分层结构

```text
robot_utils/
├── include/robot_utils/
│   ├── math/        # geometry_utils.hpp、time_utils.hpp
│   └── config/      # topic_names.hpp（全项目话题名唯一来源）
├── src/nodes/       # system_monitor_node、fault_supervisor_node
├── config/          # diagnostic_aggregator.yaml
├── launch/          # diagnostics.launch.py
└── test/            # 工具函数测试
```

| 目录 | 层次 | 职责 |
|---|---|---|
| include/.../math/ | 工具层 | 角度归一化、位姿插值、时间换算（header-only） |
| include/.../config/ | 常量层 | 全项目话题名常量（`robot_utils::topics`），避免字符串散落 |
| src/nodes/ | 监督层 | 系统资源与健康监控、故障分级与降级决策 |

## 2. 节点与话题

| 类型 | 名称 | 说明 |
|---|---|---|
| 执行 | `system_monitor_node` | 底盘/电池/急停/传感器话题新鲜度 → `/system/health` + 6 个诊断项 |
| 执行 | `fault_supervisor_node` | 把 `/system/health` 转成急停动作，并输出 `/system/fault_state`；健康断流超过 `health_timeout_ms`（默认 2000 ms）按 FAULT 处理 |
| 启动 | `diagnostics.launch.py` | 只启动诊断聚合器（配置见 `config/README.md`） |

## 3. 启动与使用

```bash
ros2 launch robot_utils diagnostics.launch.py
```

## 4. 关键约束

- `robot_utils` 是 header-only 接口库，不引入运行时状态。
- **话题名必须从 `topic_names.hpp` 的 `robot_utils::topics::k*` 引用**，禁止再写 `"/odom"` 这类字面量：
  各包在源文件顶部加 `#include "robot_utils/config/topic_names.hpp"` 与
  `namespace topics = robot_utils::topics;`，然后写 `topics::kOdom`。
- 服务名（`create_service` / `create_client`）不在该头文件内，仍由各域包自行定义。
- 依赖方向：其它包依赖 `robot_utils`；`robot_utils` **不依赖任何机器人类包**
  （其启动测试用内置桩，不启动 `robot_teleop`，避免循环依赖）。

## 5. 测试

通过消费方单测与启动测试覆盖。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
