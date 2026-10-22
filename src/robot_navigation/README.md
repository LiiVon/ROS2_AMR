# robot_navigation

导航资源管理与区域语义：Nav2/SLAM 启动编排、地图目录、区域目录与激光过滤掩膜生成。

## 1. 分层结构

```text
robot_navigation/
├── include/robot_navigation/
│   ├── model/       # 地图目录、区域目录
│   └── filters/     # 区域过滤掩膜
├── src/
│   ├── model/  filters/
│   └── nodes/       # map_manager_node、export_zone_filter_masks
├── config/          # Nav2、SLAM Toolbox、Cartographer、区域与足迹参数
├── maps/            # 地图文件
├── launch/          # nav / slam / map_manager / zone_filters
└── test/            # 目录与掩膜单元测试
```

| 目录 | 层次 | 职责 |
|---|---|---|
| include/.../model/ | 数据层 | `map_catalog`（地图清单与校验）、`zone_catalog`（区域定义与查询） |
| include/.../filters/ | 算法层 | `zone_filter_masks`：由区域几何生成激光过滤掩膜 |
| src/nodes/ | 入口层 | `map_manager_node`（地图服务）、`export_zone_filter_masks`（掩膜导出工具） |
| config/ | 参数层 | `nav2_basic.yaml`、`nav2_advanced.yaml`、`slam_toolbox.yaml`、`cartographer.lua`、`map_zones.yaml` |
| maps/ | 资源层 | 栅格地图与元数据 |

## 2. 服务与启动

| 类型 | 名称 | 说明 |
|---|---|---|
| 服务 | `/navigation/list_maps`、`/navigation/set_active_map` | 地图枚举与切换 |
| 服务 | `/navigation/list_map_zones`、`/navigation/evaluate_map_position` | 区域查询与位置判定 |
| 服务 | `/navigation/set_initial_pose_from_station` | 按站点设定初始位姿 |
| 服务 | `/navigation/set_dynamic_speed_limit` | 动态限速 |
| 话题 | `/navigation/map_state`、`/navigation/localization_health`、`/navigation/safety_state` | 地图/定位/安全状态 |
| 启动 | `nav.launch.py`、`navigation_no_docking.launch.py` | Nav2 栈（含/不含停靠行为） |
| 启动 | `slam.launch.py` | SLAM Toolbox 建图 |
| 启动 | `map_manager.launch.py`、`zone_filters.launch.py` | 地图服务、区域过滤链路 |

## 3. 启动与使用

```bash
ros2 launch robot_navigation nav.launch.py map:=warehouse.yaml
ros2 launch robot_navigation slam.launch.py
ros2 launch robot_navigation zone_filters.launch.py
```

## 4. 关键约束

- 地图与区域配置以 `config/` 为唯一来源，运行时通过服务修改并落回参数。
- 区域过滤掩膜在启动时生成，运行中变更区域需重启过滤链路。
- `/cmd_vel/nav2` 是 Nav2 的唯一输出，最终指令由安全闸门统一发布。

## 5. 测试

`map_catalog_test`、`zone_catalog_test`、`zone_filter_masks_test`。

---

构建、环境准备与全局约定见仓库根目录 `README.md`；接口与参数规范见 `需求文档.md`。
