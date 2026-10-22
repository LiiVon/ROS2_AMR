# robot_interfaces_core

## 职责

机器人状态、任务状态、底盘状态、导航统计、跟踪误差；控制模式/底盘模式/统计服务。

## 主要产物

5 msg + 3 srv

## 依赖关系

所有节点

## 使用

```bash
colcon build --base-paths src --packages-select robot_interfaces_core
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
