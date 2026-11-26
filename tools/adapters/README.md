# tools/adapters

从参考项目 `~/lz_ws/robot/scripts/` 迁移的北向/设施适配器，服务名已改为当前工程
的域前缀命名，不含业务逻辑。

| 文件 | 作用 | 依赖服务 |
|---|---|---|
| `vda5050_file_bridge.py` | VDA5050 订单文件 → ROS 订单入口 | `/business/submit_order`、`/mission/list_events` |
| `vda5050_mqtt_mock_bridge.py` | 文件目录模拟 MQTT broker → VDA5050 订单 | 同上 |
| `vda5050_instant_action_bridge.py` | VDA 即时动作（pause/resume/cancel/factsheet） | `/business/*_workflow` |
| `facility_plc_file_adapter.py` | 文件型 PLC → 设施动作 | `/facility/execute_action`、`/mission/list_events` |
| `external_callback_outbox.py` | 任务事件 → 外部 HTTP 回调（outbox） | `/mission/list_events` |

使用前提：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py backend:=mock
ros2 launch robot_tasks mission_runner.launch.py autostart:=false preflight_enabled:=false
```

示例：

```bash
# VDA 文件桥
python3 tools/adapters/vda5050_file_bridge.py /tmp/order.json /tmp/order_state.json 45

# 即时动作（factsheet 不需要运行中的订单）
python3 tools/adapters/vda5050_instant_action_bridge.py /tmp/action.json /tmp/action_state.json

# 外部回调
python3 tools/adapters/external_callback_outbox.py \
  --outbox /tmp/callbacks.json --endpoint http://127.0.0.1:19090/hook
```

这些脚本只调用已有 ROS 服务；不新增业务判断，也不直接访问数据库/PLC 硬件。
