# robot_bringup

顶层编排：环境/参数组合、全栈启动、演示与规范化门禁测试。

## 1. 分层结构

```text
robot_bringup/
├── launch/          # bringup / amr_demo / display / tracking
├── config/          # controllers.yaml
└── test/            # 启动文件与规范门禁测试
```

| 目录 | 层次 | 职责 |
|---|---|---|
| launch/ | 编排层 | 组合各包的 launch 与参数，提供整机入口 |
| config/ | 参数层 | 控制器与整机参数 |
| test/ | 门禁层 | launch 可解析性、命名/行数规范检查 |

## 2. 顶层入口

| 启动 | 用途 |
|---|---|
| `bringup.launch.py` | 基础闭环：模型/TF + 底盘驱动 + 传感器（fake scan）+ 指令仲裁/安全闸门 + 导航 action 服务端 + 监控/故障监督（不含 Nav2 与任务中枢） |
| `amr_demo.launch.py` | 仿真 + 可视化 + 任务演示 |
| `display.launch.py` | 模型与 TF 可视化 |
| `tracking.launch.py` | 路径跟踪评测链路 |

## 3. 启动与使用

```bash
ros2 launch robot_bringup bringup.launch.py
ros2 launch robot_bringup amr_demo.launch.py gui:=true
```

## 4. 关键约束

顶层 launch 只做组合与参数注入，不实现业务逻辑。

## 5. 测试

启动文件解析测试 + `conventions` 规范门禁（`tools/check_conventions.py --strict`）。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
