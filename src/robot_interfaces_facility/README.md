# robot_interfaces_facility

## 职责

设施资源、电梯/门、装卸确认、站点确认、载具状态。

## 主要产物

1 msg + 16 srv

## 依赖关系

tasks / simulation

## 使用

```bash
colcon build --base-paths src --packages-select robot_interfaces_facility
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
