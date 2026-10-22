# robot_sensors

## 职责

激光与 IMU 过滤/解析，假数据源，诊断输出。

## 主要产物

`laser_filter_node`、`imu_filter_node`、`fake_scan_node`、`imu_100d4_parser_node`

## 依赖关系

navigation / utils / bringup

## 使用

```bash
colcon build --base-paths src --packages-select robot_sensors
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
