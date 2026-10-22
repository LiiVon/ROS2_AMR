# robot_interfaces_navigation

## 职责

地图与语义区、安全状态、定位健康、导航序列 Action。

## 主要产物

3 msg + 9 srv + 1 action

## 依赖关系

navigation / teleop / tasks / simulation

## 使用

```bash
colcon build --base-paths src --packages-select robot_interfaces_navigation
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
