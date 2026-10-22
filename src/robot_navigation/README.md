# robot_navigation

## 职责

Nav2/SLAM/EKF 配置、地图目录、语义区、keepout/speed filter mask。

## 主要产物

`map_manager_node`、`export_zone_filter_masks` + 3 库

## 依赖关系

tasks / bringup / simulation

## 使用

```bash
colcon build --base-paths src --packages-select robot_navigation
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
