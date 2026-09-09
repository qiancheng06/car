# racecar — ROS2 自动竞速小车（davinci-mini / Atlas 200I A2）

> 本仓库是地平线 `davinci-mini`（华为昇腾 Atlas 200I A2）上跑的 ROS2 Humble 竞速小车工程。
> 这份 README 描述两件事：**链路关系**（数据/控制如何从传感器流到轮子）与**文件关系**（入口脚本 ↔ launch ↔ config ↔ node ↔ 包的映射）。
> 远端仓库：`https://github.com/qiancheng06/car.git`（分支 `main`）。

---

## 0. 一句话总览

```
建图(gmapping) --> 定位(EKF) --> 感知(激光循迹 + NPU/CV 视觉) --> 决策(race_manager 仲裁) --> 控制(底盘 PWM 驱动)
```

所有节点通过 colcon 工作区编译，`racecar/` 是主包（放 launch / config / scripts / map），
其余 `src/*` 是驱动、SLAM、导航和循迹功能包。

---

## 1. 链路关系（运行时数据流）

### 1.1 单车自动竞速主链路（`competition.launch.py`）

```
        [ 传感器 ]                                   [ 决策/控制 ]                         [ 执行器 ]

 激光雷达 lslidar_driver ---scan--->  cpp_pkg/laser_go ---/racecar/follow_cmd_vel---> race_manager
 编码器 encoder ------------odom/vel-        (主力循迹, 沿锥桶走廊)                        │
 IMU / 轮速 -> ekf_node ----/odom_combined->                                            │ 独占仲裁
                                                                                          │ (红灯/黄带/停车/超时)
 相机 camer.py ---image---> npu_vision_node (昇腾 NPU, best_310b.om) --检测--->          │
                     \----> yellow_band_node --黄带压线--->                              │
                     \----> parking_sign_detector --停车牌--->                          v
                                                            /teleop_cmd_vel (PWM-style Twist)
                                                                     |
                                                                     v
                                                    racecar_driver --> /racecar_driver/cmd_pwm --> 底盘串口(转向 880-2200 / 油门)
                                                                     ^
                                                            /nav/estop (急停抢占)
```

要点：
- **循迹主力是 `cpp_pkg/laser_go`**（C++），已替代早期 Python 版 `lidar_tracking`；它把跟随指令发到 `/racecar/follow_cmd_vel`，**不直接驱动轮子**。
- **`race_manager.py` 是唯一的"闸门"**：订阅跟随指令 + 视觉/灯色/黄带/里程，做状态机仲裁，统一输出到 `/teleop_cmd_vel`，保证红灯、黄带、停车牌能抢占循迹。状态发 `/racecar/race_state`、灯色发 `/light`。
- **视觉有两条实现，二选一**：`npu_vision_node.py`（昇腾 ACL，跑 `.om` 模型，比赛默认开）与 `yolo_vision_node.py`（CPU/PyTorch，跑 `.pt`，调试用）。

### 1.2 定位链路（`ekf.launch.py`）

```
 encoder(轮速) + hipnuc_imu/lidar imu --> robot_localization ekf_node (config/ekf.yaml)      --> /odom        (给规划/控制)
                                            robot_localization ekf_node (config/ekf_carto.yaml) --> /odom_combined (给 gmapping/carto)
```

### 1.3 建图链路（`gmapping.sh` / `mapping-one.sh`）

```
 lslidar scan + /odom --> slam_gmapping(gmapping_node) --> /map --> map_saver --> src/racecar/map/*.pgm+*.yaml
```
建图阶段可用 `keyboard_drive.py`（或 `keyboard-drive.sh`）人工驾驶。

### 1.4 导航链路（`Run_nav.launch.py` / `nav-one.sh`，另一套模式）

```
 /map + /odom + scan --> Nav2 代价地图 --> 规划器(nav.yaml / nav_astar_teb.yaml / nav_dwa.yaml)
                     --> teb_local_planner / dwa --> /cmd_vel --> nav_cmd_adapter --> /teleop_cmd_vel --> racecar_driver
 waypoint 循环发目标：nav2_waypoint_cycle
```
> 导航(Nav2/TEB/DWA)与竞速(race_manager)是**两套并行的开车方式**：竞速靠循迹+状态机，导航靠地图+路径规划。

---

## 2. 文件关系（仓库结构 ↔ 职责）

### 2.1 顶层入口脚本（`racecar/*.sh|*.py`，比赛/日常直接跑这些）

| 脚本 | 作用 | 内部触发 |
|---|---|---|
| `car.sh` / `start_car.sh` / `racecar_init.sh` | 开机/环境初始化、起底盘 | source install + 基础 bringup |
| `two_lap.sh` | 两圈竞速主流程（第 1 圈建图+循迹，第 2 圈复用地图） | `competition.launch.py` / `two_lap_lap1.py` |
| `check_competition.sh` | 比赛前自检（话题/节点/模型是否就绪） | `race_preflight.py` |
| `gmapping.sh` / `mapping-one.sh` | 建图 / 单次建图 | `slam_gmapping` + 键盘驾驶 |
| `nav-one.sh` / `nav.sh` / `nav_one.sh` / `check_nav.sh` | 导航模式启动与自检 | `Run_nav.launch.py` |
| `keyboard-drive.sh` | 人工键盘驾驶（建图/调试） | `keyboard_drive.py` |
| `record_training.sh` + `export_training.py` | 录制 rosbag / 导出训练数据 | ros2 bag record |
| `setup_teb.sh` + `teb_humble.repos` | 拉取并编译 TEB 导航依赖 | vcs import + colcon build |
| `reverse.py` / `reverse.sh` | 倒车/脱困小工具 | 直接发 cmd_vel |
| `save.sh` | 保存地图/结果 | map_saver 等 |
| `NAVIGATION_TEB.md` / `docs/vision_npu.md` | 导航 / NPU 视觉专题文档 | — |

### 2.2 launch 文件（`src/racecar/launch/`，节点编线）

| launch | 起什么 |
|---|---|
| `competition.launch.py` | **比赛总装**：Include `Run_car.launch.py` + `camer.launch.py` + `slam_gmapping`，并挂 laser_go / race_manager / npu_vision / yolo_vision / yellow_band / parking_sign_detector |
| `Run_car.launch.py` | **底盘硬件链**：robot_state_publisher + 3× static_transform_publisher(tf) + encoder(2 节点) + racecar_driver |
| `Run_car_one.launch.py` / `car_go.launch.py` | 单车简化启动 |
| `Run_nav.launch.py` | 导航(Nav2/TEB)模式总装 |
| `bringup_launch.py` | 综合 bringup |
| `ekf.launch.py` | 两个 `robot_localization/ekf_node`（odom 与 carto odom） |
| `camer.launch.py` | 相机节点 `camer.py` |
| `vision.launch.py` | 单独起视觉（调试 npu/yolo） |
| `xunxian.launch.py` | 单独循迹调试 |

### 2.3 config（`src/racecar/config/`，节点参数）

| yaml | 喂给谁 |
|---|---|
| `race_manager.yaml` | `race_manager.py`（话题名、灯色/黄带/超时阈值、PWM 标定） |
| `ekf.yaml` / `ekf_carto.yaml` | `ekf.launch.py` 两个 EKF 实例 |
| `vision.yaml` / `vision_npu.yaml` | `yolo_vision_node.py` / `npu_vision_node.py` |
| `yellow_band.yaml` | `yellow_band_node.py` |
| `nav.yaml` / `nav_astar_teb.yaml` / `nav_dwa.yaml` | 导航规划器配置（A*+TEB / DWA） |
| `camera/` `laser_box/` | 相机标定 / 激光盒坐标 |

### 2.4 scripts（`src/racecar/scripts/`，Python 节点，按角色）

- **决策/仲裁**：`race_manager.py`(核心状态机) · `race_preflight.py`(自检) · `bus_control.py`(多车协调, ROS1 移植)
- **循迹/驾驶**：`two_lap_lap1.py`(第 1 圈循迹建图) · `track_center_node.py` · `cv_follow_node.py` · `line_follow*.py` · `zhuitong.py`(锥桶跟随) · `cone_color.py`(锥桶颜色)
- **视觉感知**：`npu_vision_node.py`(昇腾 ACL) · `yolo_vision_node.py`(PyTorch) · `parking_sign_detector.py` · `yellow_band_node.py` · `vision_fuse.py` / `vision_sidecar.py` · `hsv_calib.py` / `hsv_show.py`(HSV 标定)
- **相机**：`camer.py` · `camer.launch.py`
- **控制适配**：`nav_cmd_adapter.py`(cmd_vel→teleop) · `racecar_teleop.py` · `keyboard_drive.py` · `py_node.py`
- **数据/工具**：`record.py` · `get.py` · `reverse.py`
> 注：`src/python_myself/python_myself/node1..3.py` 为**早期 PWM 驾驶实验**（`LidarCoordinateControlNode`，油门 1500-1550 / 转向 30-145），现多被 cpp_pkg/laser_go + race_manager 取代，保留作参考。

### 2.5 `src/` 功能包分组

| 组 | 包 |
|---|---|
| **主包** | `racecar`（launch/config/scripts/map） |
| **底盘/传感器驱动** | `racecar_driver` · `racecar_usb_fix` · `encoder` · `hipnuc_imu` · `lslidar_driver` · `lslidar_msgs` · `serial-foxy` |
| **循迹/感知** | `cpp_pkg`(laser_go 主力) · `lidar_tracking`(旧) · `python_pkg` · `python_myself`(旧实验) · `two_lap_control`(从 racecar 拆出的两圈栈) |
| **SLAM 建图** | `openslam_gmapping` · `slam_gmapping` |
| **导航(Nav2)** | `teb_local_planner_ros2` · `costmap_converter_ros2` · `nav2_waypoint_cycle` |
| **第三方** | `third_party/g2o` |

### 2.6 数据/产物（`src/racecar/`）

- **地图** `src/racecar/map/`：`competition_map.pgm/.yaml`(比赛图) · `ai_map` · `test_map`
- **模型** `vision_data/models/runs/v8_exp/weights/`：`best.pt`(CPU) · `best_310b.om`(昇腾 NPU)；另有 `src/racecar/scripts/best.pt`
- **编译产物**：`build/ install/ log/`（已在 `.gitignore` 排除，勿提交）
- 仓库外：`~/my_car_ws/`（另一套并行工作区：`cone_follower` `my_car` `ros_outdoor_autocar_2026` 等）、`~/racecar_logs/`（rosbag/训练日志）。

---

## 3. 编译与运行

```bash
cd ~/racecar
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 建图
./gmapping.sh            # + keyboard-drive.sh 人工开
# 竞速（主流程）
./two_lap.sh             # 或直接: ros2 launch racecar competition.launch.py
# 导航模式
./nav-one.sh
# 比赛前自检
./check_competition.sh
```

---

## 4. 话题速查（跨节点的"电线"）

| 话题 | 生产者 → 消费者 | 含义 |
|---|---|---|
| `/racecar/follow_cmd_vel` | laser_go → race_manager | 循迹跟随指令 |
| `/teleop_cmd_vel` | race_manager → racecar_driver | 仲裁后的最终 PWM-style 指令 |
| `/racecar_driver/cmd_pwm` | (teleop 适配) → 底盘 | 转向/油门 PWM |
| `/nav/estop` | 任意 → racecar_driver | 急停抢占 |
| `/odom` `/odom_combined` | ekf_node → 感知/建图/控制 | 里程计定位 |
| `/scan` | lslidar_driver → laser_go/gmapping | 激光 |
| `/image...` | camer.py → 视觉节点 | 相机流 |
| `/racecar/race_state` `/light` | race_manager → 调试/上位 | 状态机/灯色 |
