# robot_path_tracking

## 职责

参考路径发布、Pure Pursuit、Stanley、跟踪误差与数学库。

## 主要产物

`path_publisher_node`、`pure_pursuit_controller_node`、`stanley_controller_node`

## 依赖关系

bringup / experiments

## 使用

```bash
colcon build --base-paths src --packages-select robot_path_tracking
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
