# robot_interfaces_business

## 职责

订单提交与路由、履约、场景、工作流控制、运营快照。

## 主要产物

21 srv

## 依赖关系

tasks / simulation

## 使用

```bash
colcon build --base-paths src --packages-select robot_interfaces_business
source install/setup.bash
```

接口/参数约定见仓库根目录 `需求文档.md`，与旧版差异见 `docs/差异说明.md`。
