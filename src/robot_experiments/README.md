# robot_experiments

实验与评测：按路径场景驱动底盘，采集跟踪误差并输出 CSV/JSON 结果。

## 1. 分层结构

```text
robot_experiments/
├── src/nodes/       # experiment_runner_node
├── paths/           # 评测路径（.txt）
└── results/         # 输出目录（csv/json）
```

| 目录 | 层次 | 职责 |
|---|---|---|
| src/nodes/ | 执行层 | 评测流程：发布路径 → 采集误差 → 落盘 |
| paths/ | 输入资源 | `demo_path.txt`、`hospital_delivery.txt`、`office_patrol.txt`、`warehouse_pick_drop.txt` |
| results/ | 输出资源 | 历次 CSV/JSON 结果（如 `nav2_advanced_*.csv`、`scenario_workflow_*.json`） |

## 2. 节点与参数

| 类型 | 名称 | 说明 |
|---|---|---|
| 执行 | `experiment_runner_node` | 实验运行器 |
| 参数 | `scenario_name`、`output_csv`、`output_json` | 场景名与输出路径 |
| 参数 | `controller_name`、`path_file`、`odom_topic` | 控制器名、参考路径与里程计来源（默认 `/odom`） |

## 3. 启动与使用

```bash
ros2 launch robot_bringup tracking.launch.py \
  controller:=stanley path_file:=$(ros2 pkg prefix robot_experiments)/share/robot_experiments/paths/office_patrol.txt
```

## 4. 关键约束

结果文件默认写入工作目录，评测时请指定 `output_csv`/`output_json` 的绝对路径。

## 5. 测试

通过启动测试与跟踪链路联调验证。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
