# robot_interfaces_fleet

## 职责

多机列表、机器人状态、远程任务、车队站点任务。

## 主要产物

5 srv

## 依赖关系

tasks / simulation

## 使用

```bash
colcon build --base-paths src --packages-select robot_interfaces_fleet
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
