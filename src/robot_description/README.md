# robot_description

## 职责

URDF/xacro（含 ros2_control 硬件块）、几何参数、RViz 配置。

## 主要产物

`robot.urdf.xacro`、`robot_geometry.yaml`、`display.rviz`

## 依赖关系

bringup / hardware / simulation

## 使用

```bash
colcon build --base-paths src --packages-select robot_description
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
