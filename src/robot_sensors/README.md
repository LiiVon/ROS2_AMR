# robot_sensors

传感器数据标准化：激光过滤链、IMU 解析与滤波、无硬件时的扫描数据模拟。

## 1. 分层结构

```text
robot_sensors/
├── src/
│   ├── laser/       # laser_filter_node
│   ├── imu/         # imu_filter_node、imu_100d4_parser_node
│   └── sim/         # fake_scan_node
├── config/          # laser_filter_chain.yaml
└── launch/          # sensors.launch.py
```

| 目录 | 层次 | 职责 |
|---|---|---|
| src/laser/ | 数据层 | 激光点云/扫描过滤（角度、距离、区域掩膜） |
| src/imu/ | 数据层 | 厂商帧解析（100D4）与姿态滤波 |
| src/sim/ | 替代层 | 无硬件时按场景发布扫描数据，供导航联调 |
| config/ | 参数层 | 滤波链参数集中管理 |

## 2. 节点与话题

| 类型 | 名称 | 说明 |
|---|---|---|
| 执行 | `laser_filter_node` | 扫描过滤，输出 `/scan_filtered` |
| 执行 | `imu_filter_node` | IMU 平滑，输出 `/imu/data` |
| 执行 | `imu_100d4_parser_node` | 串口帧解析为 `sensor_msgs/Imu` |
| 执行 | `fake_scan_node` | 模拟扫描数据（无硬件联调） |

## 3. 启动与使用

```bash
ros2 launch robot_sensors sensors.launch.py
```

## 4. 关键约束

- 过滤后的数据是导航唯一输入；原始数据保留用于诊断。
- 所有节点只做数据转换，不参与控制决策。

## 5. 测试

构建期自检；数据转换逻辑通过导航/仿真联调验证。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
