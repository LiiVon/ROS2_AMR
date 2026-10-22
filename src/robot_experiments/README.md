# robot_experiments

## 职责

跟踪实验运行与 CSV/JSON 指标输出。

## 主要产物

`experiment_runner_node`

## 依赖关系

bringup / tracking

## 使用

```bash
colcon build --base-paths src --packages-select robot_experiments
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
