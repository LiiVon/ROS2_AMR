# robot_simulation

## 职责

Gazebo 世界、ros_gz 桥接、RViz 标签可视化、自动演示编排、双车演示。

## 主要产物

`amr_sim_demo_director_node`、`amr_sim_visualizer_node`、`amr_sim_gazebo_entity_bridge_node`、`gazebo_odom_tf_node`、`gazebo_unpause_node`、`robot2_fleet_demo_driver_node`

## 依赖关系

bringup

## 使用

```bash
colcon build --base-paths src --packages-select robot_simulation
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
