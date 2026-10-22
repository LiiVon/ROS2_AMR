# robot_utils

## 职责

系统健康监控、诊断聚合配置、故障监督到急停闭环、topic/service 常量。

## 主要产物

`system_monitor_node`、`fault_supervisor_node`、`robot_utils`（INTERFACE）

## 依赖关系

bringup / 全部节点

## 使用

```bash
colcon build --base-paths src --packages-select robot_utils
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
