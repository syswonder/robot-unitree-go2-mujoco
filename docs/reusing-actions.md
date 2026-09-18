# 在其他场景复用 Go2 动作

## 可以直接复用什么

此仓库包含 Go2 模型资源下载清单、四足行走控制、8 类动作控制、Robonix
Primitive 接口、演示脚本及测试。克隆源码不等于已下载模型或装好 ROS/Robonix；
先按 [README](../README.md#requirements-and-installation) 安装已有基础依赖并构建。
本扩展没有重新训练策略，没有包含宇树专有 Sport 控制器，也不向实机发命令。

后空翻和倒立来自公开预训练模型，采用固定提交与 SHA256 下载校验：
来源见 [stunt-assets.lock.json](../stunt-assets.lock.json)，许可见
[NOTICE](../NOTICE) 和 `third_party/`。权重在 `.runtime/assets`，不随源码重复上传。

## 1. 完整准备

从仓库根目录执行；`GO2_SIM_PYTHON` 指向按 README 准备的仿真虚拟环境，
`GO2_PROVIDER_PYTHON` 指向可导入 `robonix_api`、`grpc`、`rclpy` 的 Python。
不要照搬原作者电脑的绝对路径。

```bash
export GO2_SIM_PYTHON="$PWD/.venv/bin/python"
# 在本机安装前按所在工作区的安装许可规则确认。
"$GO2_SIM_PYTHON" -m pip install --no-cache-dir 'onnxruntime==1.22.1'
"$GO2_SIM_PYTHON" scripts/prepare_assets.py --stunts
bash build.sh
```

不需要特技模型时可省略 ONNX 安装和 `--stunts` 下载，普通行走及 6 类规则动作仍可用。
完整演示需要全部模型。构建会生成新动作的 RPC/ROS IDL，不要只复制 `provider.py`
而漏掉 `capabilities/` 和生成步骤。

## 2. 先在自带场景复现

各终端都要设置相同的 Python/Robonix 环境和仓库工作目录。

```bash
# 终端 A：自带园区（或使用 --scene room 测试单项动作）
bash start.sh --scene courtyard --viewer --follow-camera

# 终端 B：启动 4 个 Robonix Primitive
export ROBONIX_ATLAS=127.0.0.1:54151
rbnx boot -f robonix_manifest.yaml --no-update-check

# 终端 C：查询可用动作，不移动
bash scripts/action.sh --list
# 在当前站位调用一个动作，不重置、不传送、不自动选安全落点
bash scripts/action.sh bow
bash scripts/action.sh backflip
bash scripts/action.sh handstand_walk
# 完整自带园区演示（有一次起点重置）
bash scripts/showcase.sh --live
```

后空翻会向后移动，倒立也会移动；应选宽敞平地。`action.sh` 的 Ctrl+C 沿用
生命周期停用接口，停用后需按 Robonix 生命周期重新激活再发新动作。
它不新增场地审批或硬门；空中取消也不意味着能瞬间恢复站姿。

## 3. 对接别人的场景有两种情况

### 保留本仓库 Go2 运行程序，只换环境

这是最少适配的方式。参考 `go2_sim/courtyard.py` 的 `build(world, asset)`，
在 `go2_sim/scene.py` 的 `load_scene()` 中添加自己的环境构建分支，并在
`go2_sim/runtime.py` 的 `--scene` choices 中加入同名选项。只追加地形、场景物体、
材质和光照，不重复加入另一套 Go2。

保留 Go2 的关节/电机名、`base_link`、足部名、IMU、相机与雷达挂点；
保留 `floor` 地面名及兼容碰撞设置。当前重置点固定为 `(0,0,0.34)`，应在原点
留出站立空地；若换出生点，需要同步检查重置逻辑、传感器与路线。
质量、关节限制、摩擦或载荷变化会影响策略，改过后需重新验证。

启动新场景后仍可用 `action.sh backflip` 等接口；单项动作不依赖园区区块坐标。
**目前 CLI 只内置 room/courtyard，并没有通用 `--scene-file` 导入器。**
已有 XML 场景需由场景负责人合并到构建分支，解析相对 mesh 路径并消除重名，
不能承诺任意 MJCF/URDF 文件下载后无需适配就能运行。

### 对方已有另一套 Web/Native 仿真运行程序

复用的核心是 `go2_sim/actions.py`、`backflip.py`、`quad2hand.py` 和下载清单，
再根据对方模型映射关节/电机、观测量、时序、控制频率和动作完成状态。
`actions.py` 还依赖本仓库 `controller.py`，不能脱离这些约定单独复制一个 ONNX 就认为接通。
如果对方不是 Python Native 运行程序，ONNX Runtime 和 MuJoCo Python 代码也不能直接在浏览器里执行。

同一物理实例只能由一个控制循环推进 `mj_step`、写电机力矩，不能同时运行对方
行走控制器和本仓库动作控制器抢写关节。对外保留同一动作契约，或在其 Provider
做等价映射。此仓库提交不替代或自动合并其他人的 Web/Native 实现。

## 4. 接口与需要按场景调整的部分

| 项目 | 接口/位置 | 复用注意 |
| --- | --- | --- |
| 普通移动 | `robonix/primitive/chassis/move` | `linear_x/linear_y/angular_z`、`forward_m/rotate_deg/duration_sec` |
| 单项动作 | `robonix/primitive/chassis/action` | 请求 `name`，返回 `success/status/detail`；IDL 在 `capabilities/lib/` |
| 动作名 | crouch、bow、sway、stretch、dance、jump、backflip、handstand_walk | 后两项需要 ONNX 模型；以 `--list` 为准 |
| 园区路线 | `scripts/showcase.py` | 区域坐标、绕桩路点、楼梯入口、坡道方向按自己的场景修改 |
| 跨栏 | `JUMP_PARAMETERS`、起跳接近点 | 当前验证 4 cm 低栏；`crossed_hurdle` 判据针对名为 `jump_hurdle` 的固定 X 向栏杆，不是通用障碍检测 |
| 倒立速度/时长 | `go2_sim/actions.py` | 当前为固定阶段编排；还没有通过动作 RPC 开放速度和持续时间参数 |
| 楼梯/坡道 | 仍调用普通行走策略 | 不是专门的地形感知模型，当前验证台阶为 5/10/15 cm |

编排程序经 Atlas 发现能力后调用 Primitive，Primitive 通过本地 `/command`
交给 Native 控制器。预设路线利用仿真位姿反馈，并非 Scene/Nav2 自主规划。
可参考 `scripts/invoke_action.py` 在对方 Skill 中调用相同契约，而不是重写特技。

## 5. 验证与交付边界

```bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
"$GO2_SIM_PYTHON" -m unittest discover -s tests -v
"$GO2_SIM_PYTHON" scripts/test_actions.py
```

第二条命令仅在无外部运行实例的独立 MuJoCo 模型中检查八动作，不接实机。
完整 Robonix 调用和取消测试见 `scripts/test_action_rpc.py`；完整自带路线由
`showcase.sh` 生成 `.runtime/courtyard-showcase-*.json`。换地形后必须重新检查
落地、碰撞、姿态恢复和路线是否通过，不能沿用原园区的验收结论。

原有 room、ROS 2 传感器与普通行走保持可用。录像、个人周报和虚拟环境不纳入
源码；验收摘要见 [courtyard-showcase.md](courtyard-showcase.md)。
