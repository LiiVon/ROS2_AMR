# robot_interfaces_mission

## 职责

任务队列、站点运输、停靠、充电、交通预留与死锁检测。

## 主要产物

27 srv

## 依赖关系

tasks / simulation

## 使用

```bash
colcon build --base-paths src --packages-select robot_interfaces_mission
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
