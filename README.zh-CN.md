# Unitree Go2 MuJoCo for Robonix

<p align="center">
  <strong>简体中文</strong> | <a href="README.md">English</a>
</p>

<p align="center">
  <img src="docs/media/scene185_native.png" alt="原生 MuJoCo 中运行于 SceneSmith House 185 的 Unitree Go2" width="900">
</p>

这是一个面向 Robonix 的 Unitree Go2 仿真本体包，同时支持 Web MuJoCo WASM 与
native Python MuJoCo 后端。两种后端提供一致的底盘、LiDAR、IMU、RGB-D、建图、
探索、导航和 Scene capability。

随包提供的 `go2_rl_gym` MoE ONNX 策略可以在仿真运行期间动态 Load/Disable，且
**启动时默认关闭**。未加载策略时使用确定性关节力矩步态完成室内平地运动。本包
不包含真实宇树通信、机械臂、语音或策略训练依赖。

## 运行画面

### 室内导航

| Web MuJoCo | Native MuJoCo |
| --- | --- |
| ![Web MuJoCo 中的 House 185](docs/media/scene185_web.png) | ![原生 MuJoCo 中的 House 185](docs/media/scene185_native.png) |
| SceneSmith House 185 | SceneSmith House 185 |

两种后端发布相同的 ROS 2 传感器与里程计接口。碰撞、LiDAR 和深度数据均来自
MuJoCo 几何，SceneSmith Mesh 负责场景视觉展示。

### 全地形策略

| 楼梯场景 | 赛道场景 |
| --- | --- |
| ![原生 MuJoCo 楼梯场景](docs/media/rl_stairs_native.png) | ![原生 MuJoCo 赛道场景](docs/media/rl_track_native.png) |
| `go2_rl_stairs` | `go2_rl_track` |

这两个场景用于直接验证强化学习策略，不包含室内导航所需的标定地图和语义标注。

## 能力

| 部件或服务 | Provider | 对外能力 |
| --- | --- | --- |
| Go2 底盘 | `go2_chassis` | 里程计闭环相对运动、连续 Twist、里程计 |
| MID-360 LiDAR | `mid360_lidar` | 2D LaserScan、3D PointCloud2、单帧扫描 |
| MID-360 IMU | `mid360_imu` | 角速度和线加速度 |
| 前置 RGB-D 相机 | `front_camera` | RGB、光轴深度、标定数据、单帧采集 |
| RTAB-Map | `mapping` | 占据地图、融合点云、map-frame 位姿、地图持久化 |
| Nav2 | `nav2` | 绝对位姿导航、速度限制、障碍规避 |
| Scene | `scene` | 已观察物体、空间上下文、安全接近目标 |
| Explore | `explore` | 异步前沿探索 |
| 楼层切换 | `floor_transition` | 标定楼梯通行与分楼层地图切换 |

“向前移动 0.3 米”一类相对命令使用底盘实测里程计闭环；地图绝对坐标和 Scene
物体目标使用 Nav2。语义导航只能前往 RGB-D 相机已经观察到的物体，不会把仿真器
中的物体真值坐标直接注入 Scene。

## 本体结构

```text
rbnx chat / rbnx ask
        |
Pilot + Executor + Atlas + Soma
        |
Scene / Mapping / Nav2 / Explore / Floor Transition
        |
四个本地 primitive
        |
ROS 2 <-> WebSocket bridge <-> Web MuJoCo WASM
                         \----> native Python MuJoCo
                               |-- 关节力矩控制
                               |-- LiDAR、IMU 与 RGB-D
                               `-- 可选 ONNX 推理
```

本体尺寸和 capability 组合定义在 `soma.yaml`，坐标树位于 `urdf/go2.urdf`，
本体专属 Mapping 与 Nav2 参数位于 `config/`。运行时边界见
[仿真架构](docs/ARCHITECTURE.md)。

## 环境要求

推荐使用 x86_64 Ubuntu 22.04 或 WSL2，并准备：

- Docker Engine 与 Compose plugin；
- Node.js 20 或更高版本、Python 3.10+、`uv` 和 `curl`；
- 已安装的 `rbnx` CLI 与本地 Robonix 源码树；
- Web 后端所需的 Chromium 和 WebGL 2；
- native viewer 所需的 X11/WSLg、硬件 OpenGL 和已配置的 GPU 设备；
- bootstrap 阶段访问 npm、GitHub 和 Playwright 下载服务的网络。

native MuJoCo 物理和随包 ONNX 网络在 CPU 上运行，native viewer 与离屏 RGB
相机使用 OpenGL。`--headless` 只关闭 viewer，相机渲染仍需正常工作。

## 第一次安装

### 1. 安装 Robonix

先安装 Robonix，并确保本机可以访问其源码目录。

### 2. 获取并配置本体包

```bash
git clone <repository-url> ~/robot-unitree-go2_mujoco
cd ~/robot-unitree-go2_mujoco
cp .env.example .env
```

编辑 `.env`，至少填写：

```dotenv
ROBONIX_SOURCE_PATH=/home/your-name/robonix
VLM_BASE_URL=your-model-url
VLM_API_KEY=replace-me
VLM_MODEL=your-model-name
```

凭据应保存在被忽略的 `.env` 或环境变量中，不要提交到 Git。仿真器本身不需要
VLM key；Pilot 和 Scene 使用它完成语言与视觉推理。

### 3. 构建

```bash
bash scripts/bootstrap.sh
```

bootstrap 会安装前端依赖和 Playwright Chromium、构建 ROS 2 bridge 镜像、生成
primitive binding、验证本地 package，并构建 Robonix deployment。运行资源和预训练
策略已包含在仓库中，普通启动不需要下载 SceneSmith 数据集或重新训练策略。

## 启动本体包

建议使用三个终端。同一时刻只能有一个仿真后端和一套 Robonix 进程占用当前 ROS
图与端口。

### 终端 1：启动 MuJoCo 仿真

Web 后端：

```bash
cd ~/robot-unitree-go2_mujoco
bash sim/start.sh --backend web --environment scenesmith_house_185
```

native viewer：

```bash
bash sim/start.sh --backend native --viewer --environment scenesmith_house_185
```

native 无窗口模式：

```bash
bash sim/start.sh --backend native --headless --environment scenesmith_house_185
```

等待 `[sim/start] ... ready`。常用地址为：

```text
Web 页面：      http://127.0.0.1:5181/
Native 控制页： http://127.0.0.1:5181/?backend=native
Bridge 健康：   http://127.0.0.1:8766/health
```

### 终端 2：加载 Robonix 本体包

```bash
cd ~/robot-unitree-go2_mujoco
source scripts/env.sh
rbnx boot --no-update-check
```

必须从仓库根目录执行，使 `rbnx` 能找到 `robonix_manifest.yaml`。Mapping、Nav2、
Scene 和四个 primitive 应进入 `ACTIVE`；skill 在收到任务前保持 inactive 属于正常
状态。

### 终端 3：使用 rbnx chat

```bash
cd ~/robot-unitree-go2_mujoco
source scripts/env.sh
rbnx caps -v
rbnx tools
rbnx chat
```

可以尝试：

```text
拍摄前置相机并描述当前房间。
向前移动 0.3 米。
移动到地图坐标 x=6.09、y=2.05，最终朝向为 0 弧度。
以不超过 0.18 米每秒的速度探索房间 120 秒，并返回任务 ID。
列出 Scene 已观察到的物体。
移动到最近的已观察桌子旁边。
在双层房屋中上楼、下楼、去一楼或去二楼。
```

Explore 和 Floor Transition 都是异步任务，应保存返回的 `run_id` 用于状态查询与
取消。直接底盘运动、普通导航、探索和换层任务不能并发执行。

## 策略和操作

两种后端的策略选择与 `Load / Disable Policy` 均位于右上角
`Simulation > Policy`。加载或禁用策略不会重置机器人位姿。生产模式不注册 Web
键盘运动监听，也不注册 native viewer 运动按键，运动由 Robonix 托管。

只有需要本地调试驾驶时才添加 `--dev`：

```bash
bash sim/start.sh --backend web --dev --environment go2_rl_stairs
bash sim/start.sh --backend native --viewer --dev --environment go2_rl_stairs
```

开发按键为 W/S 前后、A/D 转向、Q/E 横移、Space 停止、X 重置；native viewer
额外支持 L 加载或禁用策略。

当前策略为 `wty-yy/go2_rl_gym` 的
`go2_moe_cts_high_slope_thre_164k_0.6715`。模型输入五帧 45 维观测，以 50 Hz
运行在 0.002 秒物理步长上，并保留上游关节顺序、动作缩放与 PD 增益。详见
[策略来源](assets/robots/go2/policy/moe_rough/UPSTREAM.md)。

策略关闭时使用确定性关节力矩步态完成室内平地运动；策略加载后，速度闭环会补偿
预训练模型在低速导航命令下的响应。两种控制器都不能保证在任意地形上安全通过。

## 场景

| 环境 ID | 用途 | Web | Native |
| --- | --- | --- | --- |
| `scenesmith_house_185` | 多房间客厅与浴室；默认建图/导航场景 | 支持 | 支持 |
| `scenesmith_house_186` | 多房间卧室与浴室 | 支持 | 支持 |
| `go2_rl_stairs` | go2_rl_gym 原始楼梯测试场景 | 支持 | 支持 |
| `go2_rl_track` | go2_rl_gym 原始赛道 | 支持 | 支持 |
| `scenesmith_multilevel_house` | 一段直楼梯连接的标定双层房屋 | 支持 | 支持；换层推荐使用 |

切换场景前先停止 Robonix 和仿真，再使用新的环境 ID 重启，避免复用上一场景的
地图和语义位姿。

### 场景画廊

#### Web MuJoCo

| House 185 | House 186 |
| --- | --- |
| ![Web MuJoCo House 185](docs/media/scene185_web.png) | ![Web MuJoCo House 186](docs/media/scene_186_web.png) |
| 楼梯 | 赛道 |
| ![Web MuJoCo 楼梯](docs/media/rl_stairs_web.png) | ![Web MuJoCo 赛道](docs/media/rl_track_web.png) |

#### Native MuJoCo

| House 185 | House 186 |
| --- | --- |
| ![原生 MuJoCo House 185](docs/media/scene185_native.png) | ![原生 MuJoCo House 186](docs/media/scene186_native.png) |
| 楼梯 | 赛道 |
| ![原生 MuJoCo 楼梯](docs/media/rl_stairs_native.png) | ![原生 MuJoCo 赛道](docs/media/rl_track_native.png) |

双层房屋：

<p align="center">
  <img src="docs/media/scene_multifloors_native.png" alt="原生 MuJoCo 中的 SceneSmith 双层房屋" width="560">
</p>

场景来源、校验和与可重复导入命令见 [docs/scenes.md](docs/scenes.md)。

## 上下楼与楼层切换

<p align="center">
  <a href="docs/media/floor_transition.mp4">
    <img src="docs/media/floor_transition.gif" alt="Go2 沿已标定直楼梯上楼" width="720">
  </a>
</p>

<p align="center">
  <a href="docs/media/floor_transition.mp4">观看 88 秒完整上楼与下楼演示</a>
</p>

`floor_transition` 是 `scenesmith_multilevel_house` 固定场景专用的 deployment
skill。它使用确定性步态接近楼梯，在楼梯段加载 `moe_rough`，持续检查入口位置、
朝向、中心线偏差、机身姿态和落地高度，最后切换到目标楼层地图。支持 `UP`、
`DOWN`、`GO_TO_FLOOR(1|2)`、状态查询和取消。

首次运行前安装随包地图：

安装脚本会将仓库中的 xz 压缩数据库展开为运行时地图目录内的 `rtabmap.db`，无需
安装 Git LFS。

```bash
bash scripts/install-prebuilt-maps.sh
bash sim/start.sh --backend native --viewer --environment scenesmith_multilevel_house
```

仿真和 Robonix 启动后，可使用不依赖 Pilot/VLM 的确定性命令：

```bash
bash scripts/floor-transition.sh up
bash scripts/floor-transition.sh down
bash scripts/floor-transition.sh floor 2
bash scripts/floor-transition.sh floor 1
```

该 skill 仅适用于一段已知直楼梯和两份随包楼层地图，不支持自动发现未知楼梯、
多个连接点选择、跨楼层单张二维路径规划或真实硬件安全认证。不要把现有坐标直接
复制到其他场景。状态流程、标注字段、地图 ID、安全检查和人工验收见
[双楼层换层 Demo](docs/MULTI_FLOOR_DEMO.zh-CN.md)。

## 验证

离线测试：

```bash
npm test
python3 -m unittest discover -s primitives/tests
```

仿真和 Robonix 启动后的在线验收：

```bash
# 传感器、provider 注册状态和地图：
bash scripts/acceptance.sh --require-stack --require-map

# 里程计闭环前进、后退与正反 30 度旋转：
bash scripts/acceptance.sh --relative 0.4

# 绝对地图位姿到达和取消；坐标应选择当前地图中的空闲位置：
bash scripts/acceptance.sh --navigate 5.6 2.4 --yaw 0

# Scene 物体接近与自主探索：
bash scripts/acceptance.sh --semantic --object-id scene.object.table_001
bash scripts/acceptance.sh --explore --explore-duration 150 --explore-timeout 240 --explore-speed 0.18

# Native 模型、传感器与策略切换：
docker exec mujoco_go2_sim python3 /workspace/sim/tests/native_smoke.py --environment all --policy
```

验收脚本会移动机器人。相对运动和绝对导航应在 `moe_rough` 关闭和加载两种状态下
分别执行，且运行时不能存在其他运动控制任务。

## 停止

```bash
cd ~/robot-unitree-go2_mujoco
source scripts/env.sh
rbnx shutdown
bash sim/stop.sh
```

两套生命周期相互独立：`rbnx shutdown` 不会停止 MuJoCo，`sim/stop.sh` 也不会停止
Robonix。运行日志位于 `.runtime/` 和 `rbnx-boot/logs/`。

## 添加场景

环境 package 统一注册在 `assets/environments/manifest.json`。Mesh 场景通常包含
`scene.xml`、`index.json`、`spawn.json` 以及引用的 mesh/texture。SceneSmith 转换、
碰撞准备、出生点搜索、地图生成和来源校验见 [docs/scenes.md](docs/scenes.md)。
新增场景应作为独立环境 package 接入，不需要为每个场景复制 Go2 primitive 或策略
控制器。

## 限制与安全

- 本包控制的是仿真 Go2，不是真实硬件安全层。
- 直接相对运动不具备避障能力；房间尺度移动应使用 Nav2。
- Scene 无法导航到尚未被传感器观察到的物体。
- Explore、Navigation、直接运动和 Floor Transition 必须独占运动控制权。
- 随包全地形策略是已发布的预训练控制器，不保证通过任意楼梯、斜坡、摩擦条件或
  障碍几何。
- Reset 或拖拽机器人会使活动 Mapping/Nav2 任务和持久化楼层状态失效。

## 目录

| 路径 | 作用 |
| --- | --- |
| `assets/robots/go2/` | Go2 MJCF、mesh、确定性控制器和 ONNX 策略 |
| `assets/environments/` | 可独立选择的场景 package |
| `primitives/` | 底盘、LiDAR、IMU 和 RGB-D provider |
| `skills/explore/` | 前沿探索适配 |
| `skills/floor_transition/` | 标定双层楼梯 skill |
| `sim/native/` | native MuJoCo 运行时与策略推理 |
| `sim/bridge/` | ROS 2 与 WebSocket bridge |
| `src/` | Web MuJoCo 运行时、传感器和控制面板 |
| `config/`、`soma.yaml`、`urdf/` | 导航参数、本体组成与坐标变换 |
| `robonix_manifest.yaml` | Robonix deployment 入口 |

## 上游项目与许可证

本接入参考 [mujoco_robonix](../mujoco_robonix) 的运行模式，基于
[MuJoCo-GS-Web](https://github.com/Vector-Wangel/MuJoCo-GS-Web)，并参考
[真实 Go2 Robonix 本体包](https://github.com/syswonder/robot-unitree-go2)。MuJoCo
本体资产保留 Unitree BSD 声明，前端基础保留 MIT 许可证，SceneSmith 资产和
go2_rl_gym 权重/场景保留各自上游声明。详见 [NOTICE](NOTICE) 与仓库许可证文件。
