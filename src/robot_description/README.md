# robot_description

机器人模型与几何：URDF/xacro、几何参数与 RViz 配置。

## 1. 分层结构

```text
robot_description/
├── urdf/            # 机器人模型（xacro）
├── config/          # robot_geometry.yaml：尺寸/轮距/传感器安装
└── rviz/            # 可视化配置
```

| 目录 | 层次 | 职责 |
|---|---|---|
| urdf/ | 模型层 | 底盘、轮组、传感器、TF 树定义 |
| config/ | 参数层 | 几何参数与安装位置（单一来源） |
| rviz/ | 可视层 | RViz 显示配置 |

## 2. 使用方式

| 类型 | 名称 | 说明 |
|---|---|---|
| 启动 | `robot_bringup display.launch.py` | 模型 + `robot_state_publisher` + RViz |
| 文件 | `urdf/*.xacro` | 机器人模型 |
| 文件 | `config/robot_geometry.yaml` | 几何与安装参数 |

## 3. 启动与使用

```bash
ros2 launch robot_bringup display.launch.py
```

## 4. 关键约束

几何参数修改后需同步 `config/robot_geometry.yaml`，避免模型与控制参数不一致。

## 5. 测试

模型可解析性在启动测试中校验。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
