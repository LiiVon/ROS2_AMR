# robot_utils 配置说明

本目录存放 `robot_utils` 包的运行时配置文件。当前只有一个文件：

| 文件 | 用途 |
|---|---|
| `diagnostic_aggregator.yaml` | ROS 2 诊断聚合器（`diagnostic_aggregator`）参数：把零散诊断项整理成 `Robot` 诊断树 |

---

## 一、`diagnostic_aggregator.yaml`

### 1.1 它解决的问题

机器人各节点用 `diagnostic_updater` 往 `/diagnostics` 发**零散诊断项**（一个节点可能发好几条）。
原样看是一长串平铺的状态，不便于判断"底盘坏了还是传感器坏了"。

诊断聚合器订阅 `/diagnostics`，按名称规则把诊断项**归类成树**，再发布：

| 话题 | 内容 |
|---|---|
| `/diagnostics_agg` | 归类后的完整诊断树（`rqt_robot_monitor` 消费） |
| `/diagnostics_toplevel_state` | 整棵树的最坏状态（OK / WARN / ERROR / STALE） |

### 1.2 谁加载它

| 加载方 | 场景 |
|---|---|
| `src/robot_bringup/launch/bringup.launch.py` | 整机启动（日常使用） |
| `src/robot_utils/launch/diagnostics.launch.py` | 只启动诊断，便于单独排查 |

两个 launch 都把本文件作为 `diagnostic_aggregator/aggregator_node` 的 `--params-file` 传入。

### 1.3 参数结构

```yaml
analyzers:              # ← 顶层键必须等于聚合器节点名
  ros__parameters:      # 该节点的参数集合
    path: Robot         # 诊断树根节点 → /Robot
    <分析器键名>:        # 仅用于在参数里区分多个分析器，不出现在树中
      type: ...         # 分析器插件
      path: ...         # 该分类在诊断树中的名字
      startswith: [...] # 名称前缀匹配规则
```

| 字段 | 作用 | 注意 |
|---|---|---|
| `analyzers` | 参数命名空间 | **必须与节点名一致**（`aggregator_node` 内部节点名为 `analyzers`）；写错则参数静默不生效，只看到默认空树 |
| `ros__parameters` | ROS 2 节点参数固定写法 | 少写会导致参数加载不到 |
| `path`（根） | 诊断树根节点名 | `Robot` → 各分类为 `/Robot/Chassis` 等 |
| 分析器键名 | 参数内的唯一标识 | 只用于区分多个分析器，**不显示在树里** |
| `type` | 分析器插件类 | 本项目统一用 `diagnostic_aggregator/GenericAnalyzer` |
| `path`（分类） | 树里显示的分类名 | 与键名不必相同 |
| `startswith` | 前缀匹配列表 | 命中任意一条即归入该分类；同一分类内多条会**合并成一个状态，取最坏** |

### 1.4 当前三个分类

| 分类（树节点） | 匹配前缀 | 实际发布者 | 判据 |
|---|---|---|---|
| `/Robot/Chassis` | `system/chassis` | `system_monitor_node` | 订阅 `/chassis/state`，判断连接状态与故障码 |
| | `system/battery` | `system_monitor_node` | `/chassis/state.battery_voltage` 低于阈值 |
| `/Robot/Sensors` | `system/scan` | `system_monitor_node` | `/scan` 是否在超时窗口内更新 |
| | `system/imu` | `system_monitor_node` | `/imu/data` 是否在更新 |
| | `system/odom` | `system_monitor_node` | `/odom` 是否在更新（导航与 EKF 依赖） |
| | `laser_filter` | `robot_sensors/laser_filter_node` | 本帧被过滤点数等 |
| | `imu_filter` | `robot_sensors/imu_filter_node` | IMU 坐标系/转发状态 |
| `/Robot/Safety` | `system/emergency_stop` | `system_monitor_node` | 急停是否激活、状态是否过期 |

未被任何 `startswith` 命中的诊断项会进入聚合器自动创建的 `/Robot/Other` 分组（不会丢，但不易发现）。

> 名称区分：诊断项名是 `system/emergency_stop`（无前导斜杠，属 `diagnostic_msgs/DiagnosticStatus.name`）；
> 话题 `/system/emergency_stop`（`std_msgs/Bool`）是急停状态本身，两者同名不同物。

### 1.5 与 `/system/health` 的分工

| 接口 | 粒度 | 使用方 |
|---|---|---|
| `/diagnostics_agg` | 逐项细节（哪个模块、状态、键值对） | 人看（`rqt_robot_monitor`）、排障 |
| `/system/health` | 单一结论（`OK` / `WARN` / `ERROR`） | `fault_supervisor_node` 据此决定是否触发/解除急停 |

也就是说：**聚合器负责"看得清"，`system/health` 负责"判得动"**。新增模块诊断时优先改本文件做归类，
只有在需要影响急停决策时，才去动 `system_monitor_node` 的结论合成逻辑。

### 1.6 如何扩展

**新增一个分类**（例如地图/定位）：

```yaml
    localization:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Localization
      startswith: ["system/localization", "map_server"]
```

**让新诊断项归入已有分类**：把它的名称前缀加进对应 `startswith` 即可，无需改代码。

**注意**：同一分类下的多条诊断项会被合并成一个状态（取最坏）。如果两个模块需要**分别定位**，
就不要塞进同一个分析器，另开一个分类。

### 1.7 排查清单

| 现象 | 原因 | 处理 |
|---|---|---|
| `rqt_robot_monitor` 只显示空的 `Other` | `analyzers:` 键名与节点名不一致，或 `ros__parameters` 写错层级 | 核对顶层键；确认没有多余缩进 |
| 新诊断项出现在 `Other` 下 | 没有对应前缀规则 | 把前缀加进目标分类的 `startswith` |
| 分类一直是 `STALE` | 该诊断项不再上报（对应节点已退出） | 确认相关节点在运行；`ros2 node list` |
| 分类 `ERROR` 但看不出是哪一项 | 分类内多条被合并取最坏 | 看 `/diagnostics_agg` 详情，或拆成多个分类 |
| 修改配置后无变化 | 聚合器未重启 | 重启 launch（参数只在启动时读取） |

### 1.8 验证

```bash
# 只启动聚合器
ros2 launch robot_utils diagnostics.launch.py

# 另有终端时，另起一个终端查看归类结果
ros2 topic echo /diagnostics_agg --once | head -40

# 或查看条目名称与状态
ros2 topic echo /diagnostics --once | grep -E "name:|level:"
```

启动日志里出现下面几行，说明本文件被正确加载：

```text
[AnalyzerGroup]: Retrieved 16 parameter(s) for analyzer group with prefix ''.
[GenericAnalyzerBase]: Initialized analyzer 'Chassis' with path '/Robot/Chassis'
[GenericAnalyzerBase]: Initialized analyzer 'Sensors' with path '/Robot/Sensors'
[GenericAnalyzerBase]: Initialized analyzer 'Safety'  with path '/Robot/Safety'
```
