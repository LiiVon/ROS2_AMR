# robot_bringup

## 职责

bringup / display / tracking / amr_demo 四个启动入口与统一配置。

## 主要产物

4 个 launch + controllers.yaml

## 依赖关系

全部包（运行期）

## 使用

```bash
colcon build --base-paths src --packages-select robot_bringup
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
