# robot_tasks

## 职责

任务队列/优先级/抢占/恢复、订单路由、站点运输、设施联动、充电策略、车队、站点配置。

## 主要产物

`mission_runner_node`、`navigate_sequence_server_node`、`site_config_manager_node` + 2 库

## 依赖关系

bringup / simulation

## 使用

```bash
colcon build --base-paths src --packages-select robot_tasks
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
