# robot_teleop

速度指令链路：多路输入仲裁（twist_mux）与安全闸门，保证 `/cmd_vel` 只有一个发布者。

## 1. 分层结构

```text
robot_teleop/
├── include/robot_teleop/
│   └── safety/      # cmd_vel_safety.hpp：限幅、超时、急停判定
├── src/
│   ├── mux/         # cmd_vel_mux_node
│   ├── safety/      # cmd_vel_safety_gate_node
│   └── input/       # keyboard_teleop_node、virtual_rc_node
├── config/          # twist_mux.yaml
├── launch/          # cmd_vel_stack.launch.py
└── test/            # 安全逻辑单元测试
```

| 目录 | 层次 | 职责 |
|---|---|---|
| include/.../safety/ | 算法层 | 纯逻辑：限幅、加速度约束、超时与急停判定（可单测） |
| src/mux/ | 仲裁层 | 多优先级输入仲裁，输出 `/cmd_vel/muxed` |
| src/safety/ | 安全层 | 订阅 `/cmd_vel/muxed` + 急停/接管状态，输出 `/cmd_vel` |
| src/input/ | 输入层 | 键盘遥控与虚拟遥控器 |

## 2. 链路与话题

| 类型 | 名称 | 说明 |
|---|---|---|
| 话题 | `/cmd_vel/nav2`、`/cmd_vel/tracking`、`/cmd_vel/teleop` | 三路输入 |
| 话题 | `/cmd_vel/muxed` | 仲裁输出 |
| 话题 | `/cmd_vel` | **安全闸门唯一发布** |
| 话题 | `/teleop/active_source` | 当前生效输入源 |
| 执行 | `cmd_vel_mux_node`、`cmd_vel_safety_gate_node`、`virtual_rc_node`、`keyboard_teleop_node` |

## 3. 启动与使用

```bash
ros2 launch robot_teleop cmd_vel_stack.launch.py
ros2 run robot_teleop keyboard_teleop_node
```

## 4. 关键约束

- 除 `cmd_vel_safety_gate_node` 外，任何节点不得发布 `/cmd_vel`。
- 急停、控制权未接管、输入超时三种情况必须输出零速。

## 5. 测试

`cmd_vel_safety_test`（限幅、超时、急停、接管）。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
