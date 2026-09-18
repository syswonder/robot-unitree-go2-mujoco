# Go2 六区域园区与 Robonix 特技演示

这是 Native MuJoCo 本体包的可选扩展，不替换原 room、行走策略或实机导航/跟随。
本页讲的是仿真，不是实机特技验收。

首次复现或接入其他场景，请先看 [动作复用指南](reusing-actions.md)。
单个动作可用 `bash scripts/action.sh backflip` 调用，无需运行整条园区路线。

## 场景和完整路线

园区包含咖啡屋、铺装步道、座椅、树木、路灯，六区域间隔约 4–5 米。
低栏、楼梯、坡道和绕桩具有真实碰撞几何。
园区使用低饱和哑光灰地面、柔和定向光及暗色分区，避免起点聚光过曝。
这些显示调整不改变碰撞、摩擦、质量或原 room 场景。

| 区域 | 中心（m） | 演示 |
| --- | --- | --- |
| A 平地机动 | (0, 0) | 左右各转一周、侧移、后退、返回中心 |
| B 跨栏 | (4, 0) | 停稳、跳过 4 cm 高/厚低栏、落地继续行走 |
| C 绕桩 | (4, 5) | 三个实体圆柱之间的预编排折线绕行 |
| D 楼梯 | (-1, 5) | 5/10/15 cm 台阶、平台、下楼梯 |
| F 坡道 | (-5, 5) | 上坡、越顶、下坡 |
| E 特技表演 | (-5, 0) | 蹲起、鞠躬、摇摆、伸展、舞蹈、后空翻、倒立前进与恢复 |

只有开始时 Reset 一次，区域之间靠关节力矩行走，不传送机器人。
路线使用仿真位姿反馈修正预设路点，不是 Scene/Nav2 自主导航或视觉避障。

## Robonix 控制链

```text
showcase.py 区域路线与动作编排
  → Atlas 发现能力 / gRPC
  → go2_sim_chassis Primitive（同一执行锁）
       chassis/move   → 原四足 RL 行走控制器
       chassis/action → PD 动作 / ONNX 后空翻和倒立策略
  → Native runtime（已有所有权、租约、取消/停止）
  → 12 关节限幅 PD 力矩 → MuJoCo 接触动力学
  → 位姿、速度、动作测量、RGB-D / 雷达 / IMU 反馈
```

chassis/move 支持 linear_x、linear_y、angular_z（m/s、rad/s）、
duration_sec、forward_m（m）、rotate_deg（度）。
仿真限幅前后 0.5 m/s、侧向 0.3 m/s、转向 0.9 rad/s；
相对转向控制最高 0.8 rad/s。相对距离调用的 linear_x 可指定巡航速度，
路线一般 0.4、楼梯和坡道 0.25 m/s。这些只属于仿真，不改实机参数。

chassis/action 请求 name，返回 success、status、JSON detail。
动作名：crouch、bow、sway、stretch、dance、jump、backflip、handstand_walk。
自定义能力与 IDL 在 primitives/go2_sim_chassis/capabilities/。
动作和行走共用执行锁；停用/取消/租约过期沿用原停止路径。
空中取消不等于瞬间恢复站姿。

动作过程不写根位姿、不施加额外外力、不改变重力或厂商模型质量、阻尼、摩擦。
跨栏是针对当前低栏与起跳位置调试的 PD 轨迹，不是通用地形跳跃策略。

## 可选特技模型与依赖

宇树官方 MuJoCo 包提供模型和低层接口，不自动包含专有 Sport 特技控制器。
本扩展使用公开研究策略，固定提交并校验 SHA256，不运行上游安装脚本：

| 功能 | 来源与固定提交 | 许可 |
| --- | --- | --- |
| 倒立及倒立行走 | [Sang-SC/go2_quad2hand](https://github.com/Sang-SC/go2_quad2hand)，bd410e4d91d3fabc1bedf1348fabe64542446e96 | BSD-3-Clause |
| 后空翻 | [Robot-Nav/GO2_backflip](https://github.com/Robot-Nav/GO2_backflip/tree/PPO-backflip)，732ddac12eafc7191e66ca3364c5ace35a675349 | MIT |

适配源码 go2_sim/quad2hand.py、backflip.py 保留上游观测、关节顺序、历史、
缩放、延迟和控制频率约定。下载清单与哈希见 stunt-assets.lock.json；
许可证在 third_party/。不宣称宇树官方特技等价。

先完成主 README 的原有构建，再在自己的仿真虚拟环境执行：

```bash
# 安装前按工作区规则批准；当前工作区已批准并安装。
"$GO2_SIM_PYTHON" -m pip install --no-cache-dir 'onnxruntime==1.22.1'
"$GO2_SIM_PYTHON" scripts/prepare_assets.py --stunts
rbnx codegen -p primitives/go2_sim_chassis
```

模型存于 .runtime/assets，不纳入 Git。缺少 ONNX 环境/文件时普通行走仍可用，
/state 的 available_actions 不列出相应可选动作。完整演示需要两套模型。

## 启动与录屏

本工作区先在根目录 source .runtime/go2-sim-tools.sh，然后进入
packages/robot-unitree-go2-mujoco。迁移机器时设置自己的 GO2_SIM_PYTHON、
GO2_PROVIDER_PYTHON 和 rbnx PATH。

三个终端分别运行：

```bash
bash start.sh --scene courtyard --viewer --follow-camera
```

```bash
export ROBONIX_ATLAS=127.0.0.1:54151
rbnx boot -f robonix_manifest.yaml --no-update-check
```

```bash
bash scripts/showcase.sh --live
# 按 Enter 执行；Ctrl+C 取消。无交互运行用 --auto。
# 仅检查能力发现用 --check。
```

Native 显示完整 3D 场景、两侧参数面板和平滑跟随镜头。
终端显示 Robonix 调用、步骤和反馈，适合大字体并排录屏。
普通速度控制及相机页：http://127.0.0.1:18765/web。
/command 的 {"view":"overview"} / {"view":"follow"} 仅切换观察镜头。
路线需数分钟，实际耗时取决于仿真和绘图负载。

## 验证与证据

```bash
"$GO2_SIM_PYTHON" -m unittest discover -s tests
PYTHONPATH=. "$GO2_SIM_PYTHON" scripts/test_actions.py
# 完整栈启动后，用 showcase.sh 的 API/codegen 环境运行 test_action_rpc.py。
```

2026-09-17 独立八动作测试通过，记录 .runtime/actions-acceptance.json：

- 跨栏腾空约 0.375 s，前移约 0.677 m，无碰栏/非足部触地，四脚越过低栏。
- 后空翻俯仰角速度积分约 -6.277 rad，腾空约 0.365 s，落地直立，无非足部触地。
- 倒立姿态累计约 15.785 s，指定行走阶段前移约 0.709 m，恢复四足。

以上为单项测量，不代替完整连续路线验收；最大根高度也不是离地跳高。
单独楼梯/坡道通过性已检查；楼梯有少量非足部台阶接触，不宣称零碰撞。
完整路线每次保存 .runtime/courtyard-showcase-*.json，以 complete: true 为准；
异常/碰栏不会标为通过，录屏也不能代替日志。

2026-09-17 完整暗色地面录制已通过 58 个实际 RPC 步骤，记录
`.runtime/courtyard-showcase-20260917T222034.json`（scope=full route、complete=true）。
起跳接近点 x=3.73 m；楼梯分为连续两次 2.35 m 调用，速度仍为 0.25 m/s。
本次跨栏无碰栏、四脚清栏，后空翻站稳，倒立行走阶段前移约 0.710 m 后恢复四足。
工作区交付视频为 `docs/reports/2026-09-17/videos/Go2-Robonix-Courtyard-20260917T221455.mp4`，
约 5 分 5 秒，1080p/30 fps/H.264。原亮色版本保留。

本仿真包没有因此新增 Scene、Mapping、Nav2 或实机特技能力。
停止顺序：rbnx shutdown -f robonix_manifest.yaml，再 bash sim/stop.sh。

## 提交前复测（2026-09-18）

原 19 项测试加 3 项距离控速回归测试共 22 项通过；6 类基础行走和 8 类独立
动作复测通过。距离控制保留原演示参数，但显式请求低于 0.18 m/s 时不再被
死区补偿强行提高。4 个 Primitive 构建成功，新动作 IDL 已生成。
构建仍有上游无关 IDL 依赖跳过和 ROS overlay 提示，不宣称全工具链无 warning。
实际 Atlas/gRPC 单动作命令完成鞠躬；动作接口测试验证了非法动作拒绝、
动作/行走互斥、生命周期取消、重新激活后执行成功。

可随源码审阅的测量记录：

- [2026-09-17 完整路线 58 步](evidence/courtyard-full-route-2026-09-17.json)
- [2026-09-18 八动作独立复测](evidence/actions-2026-09-18.json)
