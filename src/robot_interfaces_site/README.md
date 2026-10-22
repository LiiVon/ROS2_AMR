# robot_interfaces_site

## 职责

站点配置校验、激活、回滚。

## 主要产物

3 srv

## 依赖关系

tasks

## 使用

```bash
colcon build --base-paths src --packages-select robot_interfaces_site
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
