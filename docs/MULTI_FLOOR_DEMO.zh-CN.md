# Go2 双楼层换层 Demo

## 目标与边界

本实现不修改 Robonix 的 Mapping、Navigation 或 Scene 系统服务，只新增部署级
`floor_transition` skill。它只支持 `scenesmith_multilevel_house` 中人工标定的主楼梯，
提供 `UP`、`DOWN` 和 `GO_TO_FLOOR(1|2)`。它不做未知楼梯检测、不支持任意场景、
不负责楼层内物体导航，也不会把楼梯投影成一张跨层二维地图。

场景级标注位于
`assets/environments/scenesmith_multilevel_house/multifloor.yaml`，包含：

- 楼层编号与对应 Mapping `map_id`；
- 楼梯上下入口位姿、中心线、连接楼层；
- 上下楼终止坐标、预期高度、速度、超时；
- 入口误差、中心线偏移和机身直立安全阈值。

Skill 容器只读挂载该文件，因此标注的所有权属于场景，而不是系统 Scene 服务。
这满足“不修改系统服务”的约束，同时避免 skill 内硬编码另一份坐标。

## 地图资产

两层地图随项目存放在 `assets/maps/`：

- `scenesmith_multilevel_floor_1_v2`
- `scenesmith_multilevel_floor_2_v2`

每层包含 `rtabmap.db`、占据栅格、预览和元数据。`scripts/bootstrap.sh` 会调用
`scripts/install-prebuilt-maps.sh`，将缺失地图原子安装到
`${MAPPING_MAPS_DIR:-$HOME/.robonix/maps/mujoco-go2}`；已有完整同名地图不会被覆盖。

地图由运行中的 native MuJoCo 传感器和 Mapping 服务生成，不是由场景 XML 直接投影的
占位地图。一楼从 `(-2.95, -1.70)` 开始，二楼由机器人实际上楼后重置建图会话生成；
两层均覆盖了场景中可安全通行的房间、走廊和楼梯平台。最终制品数据为：

| 地图 | RTAB-Map 节点 | 栅格尺寸 | 分辨率 | 数据库大小 |
| --- | ---: | ---: | ---: | ---: |
| 一楼 `_v2` | 439 | 288 x 147 | 0.05 m | 56,991,744 B |
| 二楼 `_v2` | 787 | 241 x 153 | 0.05 m | 122,167,296 B |

这些目录是项目源资产。只有将 `assets/maps/` 一并提交到远程仓库或放入发布制品，其他
使用者才能直接安装；安装脚本只做本地复制，不会自动上传地图。Skill 加载地图时使用
`localization` 模式，实时传感器只用于定位，不会改写随包的不可变地图。要更新地图必须
显式进入 Mapping 建图模式、完成覆盖路线、保存成新的 map ID，再更新场景配置。

仓库将数据库保存为 `rtabmap.db.xz`，避免二楼数据库超过 GitHub 100 MB 单文件限制。
安装脚本会校验本机存在 `xz`，再将压缩包原子解压为运行目录中的 `rtabmap.db`；使用者
无需安装 Git LFS。

## 执行状态机

1. 激活时校验 simulator environment，并用当前 MuJoCo odom 位姿加载持久化楼层地图。
2. 楼层平面和楼梯入口对准使用基础平地步态；入口外可调用 Nav2，进入人工标注的
   连接区后由 skill 低速闭环对准。
3. 校验入口 XY 和朝向后，加载 `moe_rough` 粗糙地形策略。
4. 按楼梯中心线发布有界 Twist，同时监控横向偏差、机身直立度、X 终点和 Z 高度。
5. 到达平台后停止、切回基础步态，转到目标楼层标准朝向，再用当前 odom 实际位姿
   加载目标楼层的不可变地图。
6. 只有物理穿越和地图加载均成功后，才更新 `current_floor` 并持久化。

取消会发布零速度；若正在使用 Nav2，会按精确 `run_id` 取消并等待终态。任一安全
阈值、策略确认、定位、超时或地图加载失败都会阻止楼层状态提交。

## 启动与人工验收

```bash
bash scripts/install-prebuilt-maps.sh
bash sim/start.sh --backend native --headless --environment scenesmith_multilevel_house
```

另一个终端：

```bash
source scripts/env.sh
rbnx boot --no-update-check
```

交互终端可说“上楼”“下楼”“去二楼”“去一楼”。也可明确要求调用
`floor_transition.execute`，然后以返回的 `run_id` 查询
`floor_transition.status`。

若要绕过 Pilot/VLM 做确定性人工验收，可在另一个终端直接调用正式 skill：

```bash
bash scripts/floor-transition.sh up
bash scripts/floor-transition.sh down
bash scripts/floor-transition.sh floor 2
bash scripts/floor-transition.sh floor 1
```

脚本会动态发现当前 MCP 端点、打印 `run_id` 和阶段变化，并等待成功、失败或取消终态。
需要单独查询或取消时使用 `status RUN_ID`、`cancel RUN_ID`。验收时确认：

- `UP` 或 `GO_TO_FLOOR 2` 后 `current_floor=2`，`active_map_id` 为二楼地图；
- `DOWN` 或 `GO_TO_FLOOR 1` 后两项切回一楼；
- 重复前往当前楼层立即返回 `SUCCEEDED`，且机器人不移动；
- 楼梯过程中策略为 loaded，到达平台后恢复 unloaded；
- 不并发运行 Explore、普通 Navigation、手动驾驶或另一个换层任务。

## 已完成验收

2026-09-16 在 native MuJoCo 中完成：

- 使用完整 `_v2` 地图执行 `UP`：29.38 秒成功，实际落地高度 3.289 m，切换到
  `scenesmith_multilevel_floor_2_v2`；
- 使用完整 `_v2` 地图执行 `DOWN`：49.99 秒成功，实际落地高度 0.290 m，切换回
  `scenesmith_multilevel_floor_1_v2`；
- 重复 `GO_TO_FLOOR 1`：0.01 秒幂等成功，机器人不移动；
- 场景 native sensor/policy smoke、skill 单元测试、包校验和构建均通过。
- 完整一、二楼 `_v2` 地图分别以 439 和 787 个 RTAB-Map 节点保存并打包；
- 基础步态持续正反向原地旋转回归通过，旋转期间保持直立且没有不可接受的平移漂移；
- 二楼开放边缘增加防护，卧室家具间距调整为 Go2 可通行宽度，并由部署测试锁定。
- 换层结束后 `moe_rough.loaded=false`，Nav2 的 navigator、controller、global costmap
  和 local costmap 均保持 `active`。

运行时间会随主机负载变化，应以状态和安全条件为准，不应依赖上述秒数。
