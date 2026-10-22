# robot_interfaces_navigation

导航域接口：地图/区域/安全/定位状态与多目标导航 action。共 13 个接口定义，纯 IDL 包，不包含可执行代码。

## 1. 分层结构

```text
robot_interfaces_navigation/
├── msg/
├── srv/
├── action/
├── CMakeLists.txt
└── package.xml
```

| 目录 | 层次 | 职责 |
|---|---|---|
| msg/ | 接口层 | 3 个 msg 定义 |
| srv/ | 接口层 | 9 个 srv 定义 |
| action/ | 接口层 | 1 个 action 定义 |

## 2. 接口清单

| 目录 | 数量 | 名称 |
|---|---|---|
| msg/ | 接口定义（3 个） | `LocalizationHealth`、`MapZone`、`SafetyState` |
| srv/ | 接口定义（9 个） | `ClearObstacleBlockage`、`EvaluateMapPosition`、`ListMaps`、`ListMapZones`、`ReportObstacleBlockage`、`RequestRelocalization`、`SetActiveMap`、`SetDynamicSpeedLimit`、`SetInitialPoseFromStation` |
| action/ | 接口定义（1 个） | `NavigateSequence` |

## 3. 启动与使用

```bash
ros2 interface show robot_interfaces_navigation/srv/NavigateSequence
```

## 4. 关键约束

字段是跨包契约：修改需同步所有使用方，并保持向后可读（新增字段给默认值）。

## 5. 测试

构建期生成的 IDL 由消费方单测与启动测试间接覆盖。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
