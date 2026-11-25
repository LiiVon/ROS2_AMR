# display.rviz 配置说明

`display.rviz` 是 RViz2 的保存格式（**JSON 语法，不支持注释**），所以配置项的说明统一写在本文件中，
`display.rviz` 本身保持原样不改动。

启动方式：

```bash
# 只显示模型（robot_state_publisher + joint_state_publisher）
ros2 launch robot_bringup display.launch.py
# 或手动指定配置
rviz2 -d $(ros2 pkg prefix robot_description)/share/robot_description/rviz/display.rviz
```

## 一、整体结构

| 顶层键 | 作用 |
|---|---|
| `Panels` | 左侧面板列表，当前只保留 `Displays` 面板（显示项树） |
| `Visualization Manager` | 视图管理器：显示项、全局选项、视图参数 |
| `Visualization Manager.Displays` | 显示项数组，顺序即面板中的排列顺序 |
| `Visualization Manager.Global Options` | 全局设置：固定坐标系、背景色、帧率 |

## 二、全局选项（Global Options）

| 配置项 | 当前值 | 含义 |
|---|---|---|
| `Fixed Frame` | `base_footprint` | 视图的固定坐标系。模型展示时不依赖外部定位，用地面投影点最稳定；接 Nav2 或 EKF 后可改为 `map`/`odom` |
| `Background Color` | `48; 48; 48` | 深灰背景，避免白底看不清细线 |
| `Frame Rate` | `30` | 界面刷新帧率（Hz），与传感器频率无关 |

> 若界面提示 `Fixed Frame [xxx] does not exist`，说明该坐标系尚未发布：
> 先确认 `robot_state_publisher` 已启动，或把 Fixed Frame 改回 `base_footprint`。

## 三、显示项（Displays）

### 1. Grid（栅格）

| 配置项 | 当前值 | 含义 |
|---|---|---|
| `Class` | `rviz_default_plugins/Grid` | 地面参考网格 |
| `Enabled` | `true` | 默认显示 |
| `Cell Size` | `0.5` | 单格边长 0.5 m，用于目测机器人与站点的相对距离 |
| `Plane` | `XY` | 画在 XY 平面（地面），与 `z=0` 的 `base_footprint` 对齐 |
| `Plane Cell Count` | `20` | 每方向 20 格，覆盖 ±5 m，正好是 `indoor_room` 地图范围 |
| `Reference Frame` | `<Fixed Frame>` | 跟随全局固定坐标系 |

### 2. RobotModel（机器人模型）

| 配置项 | 当前值 | 含义 |
|---|---|---|
| `Description Source` | `Topic` | 从话题订阅 URDF，而不是读本地文件，便于 launch 动态生成模型 |
| `Description Topic.Value` | `/robot_description` | `robot_state_publisher` 发布的标准话题 |
| `Durability Policy` | `Transient Local` | 晚启动的 RViz 也能拿到已发布过的模型（latched 语义） |
| `Reliability Policy` | `Reliable` | 模型消息不允许丢包 |
| `History Policy` / `Depth` | `Keep Last` / `5` | 只保留最近 5 条 |
| `Alpha` | `1` | 不透明显示 |
| `Visual Enabled` | `true` | 显示外观（带材质颜色） |
| `Collision Enabled` | `false` | 默认不显示碰撞体，避免与外观重叠看不清；调试碰撞时再开 |
| `Update Interval` | `0` | 每帧更新（0 = 不限间隔） |
| `TF Prefix` | `""` | 单机不加前缀；多机仿真（`robot_2`）时改为 `robot_2/` |

### 3. TF（坐标变换）

| 配置项 | 当前值 | 含义 |
|---|---|---|
| `Enabled` | `false` | 默认关闭：坐标系多时箭头会遮挡模型，需要排查 TF 时手动勾选 |
| `Frame Timeout` | `15` | 单个坐标变换的显示超时（s） |
| `Marker Scale` | `0.2` | 坐标轴箭头长度，0.2 m 与 0.55 m 车体比例协调 |
| `Show Arrows` / `Show Axes` / `Show Names` | 均为 `true` | 勾选后同时显示连线、坐标轴和坐标系名称 |

## 四、常见调整

| 需求 | 做法 |
|---|---|
| 看碰撞体与外观是否一致 | 打开 `RobotModel → Collision Enabled` |
| 排查 TF 断裂 | 勾选 `TF` 显示项，把 `Fixed Frame` 设为 `base_link` 逐级往上看 |
| 看导航结果 | 追加 `Map`（topic `/map`）、`Path`（topic `/plan` 或 `/navigation/current_path`）、`LaserScan`（`/scan`）显示项后另存为新 `.rviz` 文件 |
| 多机仿真 | `RobotModel.TF Prefix` 填 `robot_2/`，并另建一份配置，避免与主车冲突 |

## 五、与代码的对应关系

| RViz 配置 | 代码/文件来源 |
|---|---|
| `/robot_description` 话题 | `robot_state_publisher`（`robot_bringup` 或 `display.launch.py` <br>用 `xacro` 生成 URDF 后传入） |
| 模型几何与坐标系 | `urdf/robot.urdf.xacro` + `config/robot_geometry.yaml` |
| `base_footprint` / `base_link` | `config/robot_geometry.yaml` 的 `frames` 段（xacro 会校验名称不可改） |
| 网格尺度 0.5 m / ±5 m | 与 `robot_navigation/maps/indoor_room.yaml`（分辨率 0.5、原点 −5,−5）一致 |
