# robot_interfaces_business

业务域接口：订单提交、履约状态、工作流控制与场景任务。共 21 个接口定义，纯 IDL 包，不包含可执行代码。

## 1. 分层结构

```text
robot_interfaces_business/
├── srv/
├── CMakeLists.txt
└── package.xml
```

| 目录 | 层次 | 职责 |
|---|---|---|
| srv/ | 接口层 | 21 个 srv 定义 |

## 2. 接口清单

| 目录 | 数量 | 名称 |
|---|---|---|
| srv/ | 接口定义（21 个） | `CancelWorkflow`、`ConfirmDropoff`、`ConfirmPick`、`GetOperatorSnapshot`、`GetOrderFulfillmentStatus`、`GetWorkflowStatus`、`ListBusinessOrderTypes`、`ListScenarioTasks`、`ListScenarioWorkflows`、`PauseWorkflow`、`ReportShortPick`、`ResumeWorkflow`、`SubmitBusinessOrder`、`SubmitMilkRun`、`SubmitOrder`、`SubmitPickingWave`、`SubmitReplenishmentOrder`、`SubmitScenarioTask`、`SubmitScenarioWorkflow`、`SubmitTransportOrder`、`SubmitVda5050Order` |

## 3. 启动与使用

```bash
ros2 interface show robot_interfaces_business/srv/CancelWorkflow
```

## 4. 关键约束

字段是跨包契约：修改需同步所有使用方，并保持向后可读（新增字段给默认值）。

## 5. 测试

构建期生成的 IDL 由消费方单测与启动测试间接覆盖。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
