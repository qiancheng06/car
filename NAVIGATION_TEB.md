# Hybrid-A* + TEB 实车导航说明

这套导航链用于第二圈或独立导航测试：

```text
地图 + 激光雷达
      -> Smac Hybrid-A* 全局路径
      -> TEB 局部轨迹
      -> Nav2 velocity_smoother (/cmd_vel)
      -> nav_cmd_adapter（限幅、解锁、软件停车、超时）
      -> /racecar_driver/cmd_pwm（唯一最终控制话题）
      -> racecar_driver（再次解锁、持续看门狗、串口）
      -> 底盘
```

## 四个上车入口

以后按这四个文件操作：

```bash
bash car.sh          # 只启动传感器、里程计和底盘，硬件检查用
bash gmapping.sh     # 启动或复用底盘，并开始第一圈建图
bash save.sh         # gmapping 仍运行时，在另一终端保存 ai_map
bash nav-one.sh      # 启动或复用底盘，运行 Hybrid-A* + TEB
```

`gmapping.sh` 和 `nav-one.sh` 是两种运行模式，不能同时运行。`gmapping.sh` 可以复用 `car.sh` 的手动控制驱动；导航模式要求旧控制入口全部关闭，因此从 `car.sh` 切到 `nav-one.sh` 时要先按 Ctrl+C 停掉 `car.sh`。脚本不会重复打开串口。`nav_one.sh` 仅保留为旧文件名兼容入口。

`save.sh` 默认写入 `src/racecar/map/ai_map.yaml`；`nav-one.sh` 默认直接读取这个最新文件，不需要为每次保存地图重新编译。可用 `bash save.sh 新地图名` 保存其他名称，再通过 `map:=绝对路径` 选择。

建图时先架空确认方向，解锁后可在另一终端运行低速键盘控制：

```bash
source install/setup.bash
ros2 run racecar racecar_teleop.py
```

键盘控制允许低速倒车：`,` 为直线倒车，`m`/`.` 为倒车转向。手动模式油门限制为 1475–1550 PWM；从前进切换倒车或从倒车切换前进时，第一次按键只回中，第二次才执行新方向。导航模式仍然禁止倒车。

## 小车参数和首跑限制

《智能车参数》中的物理数据：

- 车身：长 0.45 m、宽 0.16 m、高 0.26 m
- 轮距：0.35 m；车轮半径：0.06 m
- 整车质量：6.5 kg
- 理论最大正向加速度：6.67 m/s²
- 理论最大角加速度：122.71 rad/s²

现有控制代码给出的轴距为 0.305 m，原工程最小转弯半径为 0.60 m。碰撞宽度按 0.35 m 轮距并留余量，取 0.36 m。现有激光自滤框表明 `base_footprint` 靠近车头，因此当前轮廓暂取 `x=[-0.375, 0.075] m`、`y=[-0.18, 0.18] m`。上车前必须实测 `base_footprint` 到车头、车尾的距离；不一致时同步修改 TEB、局部代价地图和全局代价地图的三个轮廓。

理论极限不能直接用于实车首跑。当前限制为：

- 最大线速度：0.45 m/s
- 最大线加速度：0.80 m/s²
- 最大角速度：0.75 rad/s
- 最大角加速度：1.50 rad/s²
- 导航模式禁止倒车和原地旋转；只有 `car.sh`/`gmapping.sh` 的人工监护模式允许低速倒车
- 适配器命令超时：0.40 s
- 驱动命令超时：0.50 s，超时后以 20 Hz 持续发送中值
- 导航模式驱动油门范围：1500–1550 PWM；人工监护模式：1475–1550 PWM；舵机范围：1200–1800 PWM

主参数文件是 `src/racecar/config/nav_astar_teb.yaml`。

## 第一次安装

进入小车上这个仓库的实际根目录，不要照抄固定的用户目录：

```bash
cd /你的实际路径/car
sudo rosdep init       # 只在这台机器从未初始化 rosdep 时执行一次
rosdep update
bash setup_teb.sh
source install/setup.bash
bash check_nav.sh
```

`setup_teb.sh` 会拉取固定版本的 ROS2 Humble TEB，以及 TEB 必需的 Humble 兼容 `costmap_converter`，然后以最多两个并行任务构建，降低 Atlas 板内存不足的概率。

## 安全试跑

第一次必须架空驱动轮，或放在空旷低速测试区；必须有人握住物理断电开关。ROS 软件停车不能替代物理急停。

在仓库根目录只启动一次：

```bash
bash nav-one.sh
```

脚本带单实例锁，默认强制不解锁。另开终端，进入同一仓库并加载环境：

```bash
source install/setup.bash
ros2 topic hz /scan
ros2 topic hz /odom_combined
ros2 topic echo /nav/adapter_state
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic info /racecar_driver/cmd_pwm --verbose
```

确认 `/racecar_driver/cmd_pwm` 只有 `nav_cmd_adapter` 一个发布者，并确认雷达、里程计、地图与 TF 正常。默认 AMCL 初始位姿是地图 `(0,0,0)`；若实车不在建图起点，先在 RViz 设置正确初始位姿。

确认无误后解锁。驱动和适配器会同时收到这条命令，并且只接受解锁之后的新速度命令：

```bash
ros2 topic pub --once /nav/arm std_msgs/msg/Bool "{data: true}"
```

先发送 1–2 m 的近距离直线目标。软件停车命令是：

```bash
ros2 topic pub --once /nav/estop std_msgs/msg/Bool "{data: true}"
```

解除软件停车后不会自动恢复，必须先释放，再重新解锁：

```bash
ros2 topic pub --once /nav/estop std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /nav/arm std_msgs/msg/Bool "{data: true}"
```

需要 RViz 或指定比赛地图时，必须先停掉原来的 `nav-one.sh`，再用下列命令替代首次启动命令，不能重复启动整套导航：

```bash
bash nav-one.sh use_rviz:=true
# 或
bash nav-one.sh map:=/绝对路径/competition_map.yaml
```

也可以不重启底盘，单独打开 RViz：

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix racecar)/share/racecar/rviz/navigation.rviz"
```

## 首轮调参顺序

1. 不发目标，只检查 `/scan`、`/odom_combined` 和 `map -> odom_combined -> base_footprint`。
2. 实测 `base_footprint` 到车体四边，修正配置中的三个 footprint。
3. 架空轮子，解锁后发送 1 m 直线目标，确认油门方向和舵机左右方向。
4. 若左右相反，把 `nav_cmd_adapter.steering_direction` 从 `1.0` 改为 `-1.0`。
5. 测量实际最大舵角后修正 `wheelbase_m`、`min_turning_radius_m` 和 `steering_gain`。
6. 测刹停距离，最后才逐步提高 TEB、速度平滑器、适配器和驱动的对应上限。

不要同时启动 `lidar_tracking`、`two_lap_control`、旧视觉循迹、旧 `car_go.launch.py` 或其他底盘控制节点。安全驱动默认关闭 `/car_cmd_vel` 与 `/teleop_cmd_vel` 旧入口。
