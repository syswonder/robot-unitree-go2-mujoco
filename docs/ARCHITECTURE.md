# Go2 仿真架构

```text
rbnx chat / ask
    -> Pilot + Executor + Atlas + Soma
    -> Explore / Navigation / Mapping / Scene
    -> go2_chassis / mid360_lidar / mid360_imu / front_camera
    -> ROS2 <-> sim/bridge/bridge_node.py
    -> WebSocket <-> Web MuJoCo WASM 或 Native MuJoCo
```

两种后端分别运行完整物理仿真，而不是同时控制同一个模型。
Web 在浏览器中执行物理和 ONNX；native 在 Python 中执行物理和 ONNX，
通过 MuJoCo viewer 显示场景。native 的网页只负责控制和状态显示。

## 控制

连续 Twist 的 X/Y 为机身坐标系速度，Z 为偏航角速度。
后端独立检查命令是否有效和是否超时，超时后清零期望速度。
未加载 ONNX 时使用确定性足端轨迹、关节 PD 和速度反馈；
加载后使用 go2_rl_gym 的原始观测顺序、历史和动作缩放。
所有运动由关节力矩产生，位移不会通过修改根关节位置实现。

默认策略关闭。Web 的策略选择及 Load/Disable 位于右上角 Policy 面板，
通过实际后端确认后更新界面。生产模式不注册 Web 运动按键或 native viewer
按键回调，运动由 Robonix Twist 托管；`sim/start.sh --dev` 才启用本地调试驾驶。
加载前检查权重、输入输出和控制参数；错误不能显示为加载成功。
切换策略保留机身位姿；Reset 是单独的操作，不能在建图导航任务中使用。

## 传感器和坐标

环境视觉组为 1，机器人视觉组为 2，环境碰撞组为 3，
机器人碰撞组为 4。LiDAR 和深度查询排除机器人，避免照到自身四肢。
深度射线距离乘以相机光轴方向余弦后发布为米制 optical-Z。
RGB、深度、内参采用同一采样时间戳。

ROS 的 odom 位姿保存真实机身高度、roll/pitch；速度变换到 base_link。
Mapping 发布 map->odom，桥接发布 odom->base_link 和固定传感器安装变换。
安装参数、原语清单和 topic 在 [Robonix 接入说明](ROBONIX_INTEGRATION.md) 中列出。

## 场景和语义

环境与机器人通过独立清单组合。两套新的 SceneSmith House 保留纹理，
家具静态化为保守碰撞体；楼梯和赛道保留上游几何。
[场景说明](scenes.md) 记录来源、校验和、出生点和重新导入命令。

Scene 通过真实 RGB-D 和 map TF 建立语义对象记录。
探索扩展观察范围，goal_near 返回满足地图与本体轮廓约束的接近位姿，
Navigation 执行目标。没有向 Scene 注入 XML 对象名称或真实位置的捷径。

导航以单层二维地图为基础。楼梯/赛道用于验证低层运动策略，
不承诺跨楼层 Scene/Nav2 导航。复杂接触效果受家具碰撞简化影响。
