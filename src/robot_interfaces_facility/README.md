# robot_interfaces_facility

设施域接口：载荷状态、电梯/门/资源、站点确认。共 17 个接口定义，纯 IDL 包，不包含可执行代码。

## 1. 分层结构

```text
robot_interfaces_facility/
├── msg/
├── srv/
├── CMakeLists.txt
└── package.xml
```

| 目录 | 层次 | 职责 |
|---|---|---|
| msg/ | 接口层 | 1 个 msg 定义 |
| srv/ | 接口层 | 16 个 srv 定义 |

## 2. 接口清单

| 目录 | 数量 | 名称 |
|---|---|---|
| msg/ | 接口定义（1 个） | `PayloadState` |
| srv/ | 接口定义（16 个） | `ConfirmLoad`、`ConfirmStationAction`、`ConfirmUnload`、`ExecuteFacilityAction`、`ExecuteTopModuleAction`、`GetDoorState`、`GetLiftState`、`GetPendingConfirmations`、`ListFacilityResources`、`RejectStationAction`、`ReleaseFacilityResource`、`ReleaseLiftSession`、`RequestLiftSession`、`RequestStationConfirmation`、`ReserveFacilityResource`、`SetPayloadLoaded` |

## 3. 启动与使用

```bash
ros2 interface show robot_interfaces_facility/srv/ConfirmLoad
```

## 4. 关键约束

字段是跨包契约：修改需同步所有使用方，并保持向后可读（新增字段给默认值）。

## 5. 测试

构建期生成的 IDL 由消费方单测与启动测试间接覆盖。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
