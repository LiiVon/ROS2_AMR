# robot_tasks

任务中枢：把上层订单/站点任务分解为导航目标、设施动作与恢复策略，以行为树编排、以工作流落地、以目录（catalog）承载静态配置。

## 1. 分层结构

```text
robot_tasks/
├── include/robot_tasks/
│   ├── model/       # 数据模型：目录、任务队列、载荷、事件日志、代价估计
│   ├── workflow/    # 业务流程：任务生命周期、站点序列、充电、恢复、载荷
│   ├── behavior/    # 行为树节点：把流程编排成可组合的动作
│   ├── policy/      # 策略：预检与恢复决策
│   ├── routing/     # 入口路由：订单 → 任务类型
│   └── report/      # 对外快照：操作员视图
├── src/
│   ├── model/  workflow/  behavior/  policy/  routing/  report/
│   ├── nodes/       # mission_runner / navigate_sequence_server / site_config_manager
│   └── impl/        # mission_runner 内部实现分段
├── config/          # 业务订单、站点、设施、车队、场景等 YAML
├── launch/          # mission_runner.launch.py
└── test/            # 39 个单元测试（模型/流程/行为树/路由）
```

| 目录 | 层次 | 职责 |
|---|---|---|
| include/.../model/ | 数据层 | （15 个头文件）站点/设施/码头/车队目录、任务队列、载荷、事件日志、代价估计 |
| include/.../workflow/ | 流程层 | （14 个）任务运行时、站点序列、站点运输、确认、充电、恢复、载荷等流程 |
| include/.../behavior/ | 编排层 | （10 个）行为树节点，组合工作流与导航动作 |
| include/.../policy/ | 决策层 | `mission_preflight`、`mission_recovery_policy` |
| include/.../routing/ | 路由层 | `submit_order_router`：订单类型到处理链的映射 |
| include/.../report/ | 视图层 | `operator_snapshot`：汇总任务/队列/资源状态 |
| src/nodes/ | 入口层 | `mission_runner_node`（服务与定时器注册）、导航 action 服务端、站点配置管理 |
| src/impl/ | 实现细节 | `part_1..5.inc`：mission_runner 服务处理器分段实现 |

## 2. 核心服务与节点

| 类型 | 名称 | 说明 |
|---|---|---|
| 服务 | `/business/submit_order` | **唯一**订单入口，内部路由到运输/拣选/补货等类型 |
| 服务 | `/mission/enqueue`、`/mission/cancel`、`/mission/preempt` | 任务队列管控 |
| 服务 | `/mission/pause`、`/mission/resume` | 运行中任务暂停/恢复 |
| 服务 | `/station/submit_sequence`、`/station/submit_transport` | 站点序列与站点运输任务 |
| 服务 | `/facility/*`、`/fleet/*`、`/site/*` | 设施动作、车队任务、站点配置 |
| Action | `/mission/navigate_sequence` | 由 `navigate_sequence_server_node` 提供多目标导航 |
| 话题 | `/mission/state`、`/operator/snapshot` | 任务状态与操作员视图 |
| 执行 | `mission_runner_node` | 任务中枢主节点 |
| 执行 | `navigate_sequence_server_node` | 多目标导航 action 服务端 |
| 执行 | `site_config_manager_node` | 站点配置校验与激活 |

## 3. 启动与使用

```bash
ros2 launch robot_tasks mission_runner.launch.py
ros2 service call /business/submit_order robot_interfaces_business/srv/SubmitOrder \
  "{order_id: 'order_0', pickup_station: 'S1', dropoff_station: 'S2'}" 
```

## 4. 关键约束

- 订单只能通过 `/business/submit_order` 进入；其它服务是内部编排接口。
- 资源（站点/电梯/设施）必须先预约再使用，流程退出时统一释放。
- 新增任务类型：在 `routing/` 增加映射 → 在 `workflow/` 增加流程 → 在 `behavior/` 组合动作。

## 5. 测试

39 个单元测试覆盖模型、工作流、行为树与路由（`colcon test --packages-select robot_tasks`）。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
