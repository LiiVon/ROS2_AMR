# robot_interfaces_mission

任务域接口：队列、站点、停靠、充电、交通与代价估计。共 27 个接口定义，纯 IDL 包，不包含可执行代码。

## 1. 分层结构

```text
robot_interfaces_mission/
├── srv/
├── CMakeLists.txt
└── package.xml
```

| 目录 | 层次 | 职责 |
|---|---|---|
| srv/ | 接口层 | 27 个 srv 定义 |

## 2. 接口清单

| 目录 | 数量 | 名称 |
|---|---|---|
| srv/ | 接口定义（27 个） | `BlockStationRoute`、`CancelQueuedMission`、`CancelStationOrderBatch`、`ClearTrafficReservation`、`DetectTrafficDeadlock`、`EnqueueMission`、`EstimateMissionCost`、`EstimateStationSequence`、`EstimateStationTransport`、`GetDockState`、`GetMissionRecoveryState`、`ListDocks`、`ListMissionEvents`、`ListStationRoutes`、`ListStations`、`ListTrafficReservations`、`PreemptMission`、`ReprioritizeBatch`、`ReprioritizeQueuedMission`、`RequestCharging`、`RequestDocking`、`RequestOpportunityCharging`、`RequestUndocking`、`SubmitStationOrderBatch`、`SubmitStationSequenceTask`、`SubmitStationTransportOrder`、`UnblockStationRoute` |

## 3. 启动与使用

```bash
ros2 interface show robot_interfaces_mission/srv/BlockStationRoute
```

## 4. 关键约束

字段是跨包契约：修改需同步所有使用方，并保持向后可读（新增字段给默认值）。

## 5. 测试

构建期生成的 IDL 由消费方单测与启动测试间接覆盖。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
