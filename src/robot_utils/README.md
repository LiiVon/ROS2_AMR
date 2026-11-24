# robot_utils

通用基础设施：时间/几何工具、话题名常量、系统监控与故障监督。

## 1. 分层结构

```text
robot_utils/
├── include/robot_utils/
│   ├── math/        # geometry_utils.hpp、time_utils.hpp
│   └── config/      # topic_names.hpp（话题/服务名唯一来源）
├── src/nodes/       # system_monitor_node、fault_supervisor_node
├── config/          # diagnostic_aggregator.yaml
├── launch/          # diagnostics.launch.py
└── test/            # 工具函数测试
```

| 目录 | 层次 | 职责 |
|---|---|---|
| include/.../math/ | 工具层 | 角度归一化、位姿插值、时间换算（header-only） |
| include/.../config/ | 常量层 | 话题/服务名常量，避免字符串散落 |
| src/nodes/ | 监督层 | 系统资源与健康监控、故障分级与降级决策 |

## 2. 节点与话题

| 类型 | 名称 | 说明 |
|---|---|---|
| 执行 | `system_monitor_node` | CPU/内存/磁盘/节点心跳 → `/system/health` |
| 执行 | `fault_supervisor_node` | 汇总故障并输出 `/system/fault_state` |
| 启动 | `diagnostics.launch.py` | 监控 + 诊断聚合 |

## 3. 启动与使用

```bash
ros2 launch robot_utils diagnostics.launch.py
```

## 4. 关键约束

- `robot_utils` 是 header-only 接口库，不引入运行时状态。
- 话题名必须从 `topic_names.hpp` 引用，禁止硬编码字符串。

## 5. 测试

通过消费方单测与启动测试覆盖。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
