# robot_teleop

## 职责

cmd_vel 多路仲裁、安全闸门（急停/接管/看门狗）、键盘与虚拟遥控。

## 主要产物

`cmd_vel_mux_node`、`cmd_vel_safety_gate_node`、`keyboard_teleop_node`、`virtual_rc_node`

## 依赖关系

bringup / utils

## 使用

```bash
colcon build --base-paths src --packages-select robot_teleop
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
