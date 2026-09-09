# Pure Lidar B 使用说明

本目录中的 `pure_lidar_b_node.py` 已按“雷达感知 + IMU/里程计跨越黄色无锥桶区域”的方案增强。它不替代雷达驱动、Gmapping 或 AMCL，而是和这些节点并行订阅/发布数据。

## 1. 模块职责与数据流

```text
激光雷达硬件 -> lslidar_driver -> /scan
                                  |-> Pure Lidar B 循迹/障碍检测
                                  |-> Gmapping（第一圈）
                                  |-> AMCL/激光定位（第二圈）
Pure Lidar B -> /teleop_cmd_vel（控制）
             -> /pure_lidar/*（感知、状态和诊断）
```

Pure Lidar B 已实现：

- 将 `/scan` 转为车辆坐标中的点并聚类；
- 区分车辆左、右侧边界目标并拟合边界；
- 计算近处中心偏差、远处中心方向和置信度；
- 只有单侧边界时按约 1.5 m 赛道宽度推算中心并降速；
- 根据横向误差、前视方向误差和 IMU 角速度计算舵角；
- 弯道、低置信度和近障区域自动降低油门；
- 检测车头窄走廊内的挡板、锥桶、墙面等近障；
- 雷达超时或当前扫描无效时立即发布 1500/90，不使用旧控制量；
- 赛道 B 黄色无锥桶区域使用 IMU 航向和里程计距离进行两段式航迹推算；
- 发布过滤后的 LaserScan，可供调试、SLAM 或定位验证使用。

## 2. ROS 输入

| 话题 | 类型 | 用途 |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 雷达扫描 |
| `/IMU_data` | `sensor_msgs/msg/Imu` | 航向、角速度、黄色区域航向保持 |
| `/odom_combined` | `nav_msgs/msg/Odometry` | 累计距离、速度、黄色区域分段判断 |

## 2.1 参考 node1.py 的循迹优化（2026-09-07）

本版不是直接复制 `node1.py`，而是保留原有赛道 B、IMU、里程计和故障停车功能，并吸收其有效做法：

- 加入“物体前沿 + 后续连续回波”锥桶候选检测；
- 左右各只保留近处最有用的少量边界候选；
- 加入中心误差微分项，降低左右摆动；
- 转弯时连续降速，而不是在快/慢两档之间跳变。

同时修正旧版容易影响循迹的几个问题：

- 聚类保留原始扫描索引，`NaN/inf` 空隙会强制断簇，不再把两个锥桶误合并；
- 连续围挡不再因为簇太大被整体丢弃，而是切成局部边界段；
- 左右边界按相近前向距离配对后直接拟合中心线，并检查赛道宽度；
- 对离群点做残差剔除，对有效帧做轻量平滑；
- 正前方至少需要连续多点命中才判断近障，减少单点噪声急停。

离线合成扫描测试：

```bash
cd /home/davinci-mini/racecar/src/cpp_pkg/src/xdl_test/python_myself
python3 test_perception_offline.py
```

## 3. ROS 输出

| 话题 | 类型 | 含义 |
|---|---|---|
| `/teleop_cmd_vel` | `geometry_msgs/msg/Twist` | PWM 控制：`linear.x` 油门，`angular.z` 舵机角度 |
| `/pure_lidar/center_error` | `Float32` | 近处赛道中心横向偏差，单位 m |
| `/pure_lidar/heading_error` | `Float32` | 前视中心线方向误差，单位 rad |
| `/pure_lidar/confidence` | `Float32` | 边界识别置信度 0~1 |
| `/pure_lidar/boundary_points` | `Float32MultiArray` | 每 5 个数一组：x、y、半径、点数、侧别(-1左/+1右) |
| `/pure_lidar/obstacle` | `Bool` | 前方减速/停车区域是否有目标 |
| `/pure_lidar/fault` | `Bool` | 当前是否存在必须停车的故障 |
| `/pure_lidar/status` | `String` | CORRIDOR、ONE_SIDE、B_GAP_FIRST、SCAN_TIMEOUT 等状态 |
| `/pure_lidar/scan_filtered` | `LaserScan` | 保留时间戳和坐标系、无效值改为 inf 的扫描 |

默认仍建议 SLAM/AMCL 使用原始 `/scan`。只有确认过滤扫描效果后，才将其 remap 到 `/pure_lidar/scan_filtered`。

## 4. 赛道 B 黄色区域

默认方向为：左下起点沿下直道向右，绕右侧 180° 弯后，从右向左进入黄色 V 区域。进入条件同时检查：

1. 左右边界连续丢失；
2. 累计里程处于配置范围；
3. 相对起点航向变化达到阈值；
4. IMU 和里程计均为实时数据。

黄色区域按照两段约 3.06 m 处理：第一段相对入口航向约 +22.5°，第二段约 -22.5°；出口重新看到左右边界后恢复雷达循迹。若实车转向相反，将 `b_gap_direction` 从 `1.0` 改为 `-1.0`。

## 5. 启动前检查

```bash
cd /home/davinci-mini/racecar/src/cpp_pkg/src/xdl_test/python_myself
chmod +x *.sh
./check_track_b_sensors.sh
```

必须同时显示实时 `/scan`、`/IMU_data` 和 `/odom_combined`。缺少任一话题时，赛道 B 控制脚本不会启动运动。

## 6. 启动方式

仅观察输出、始终发送中性控制：

```bash
./dry_run_track_b.sh
```

传感器全部正常后启动赛道 B 控制：

```bash
./run_track_b.sh
```

第一圈同时建图：

```bash
./run_track_b_first_lap_slam.sh
# 另一个终端保存地图
bash /home/davinci-mini/racecar/save.sh
```

第二圈定位：

```bash
./run_track_b_second_lap_localization.sh
```

第二圈脚本让现有导航/定位栈管理底盘控制，Pure Lidar 以感知模式运行，并把建议命令放在 `/pure_lidar/cmd_suggestion`，避免两个控制节点同时写 `/teleop_cmd_vel`。

## 7. 状态含义

- `CORRIDOR`：左右边界正常；
- `ONE_SIDE`：只有单侧边界，已降速；
- `B_GAP_FIRST` / `B_GAP_SECOND`：黄色区域第一/第二段；
- `LIDAR_REACQUIRE`：黄色区域出口重新识别边界；
- `FRONT_OBSTACLE`：停车距离内有近障；
- `NO_BOUNDARY`：非黄色入口处边界丢失；
- `SCAN_TIMEOUT`：没有收到新扫描；
- `INVALID_SCAN`：扫描虽然到达，但有限有效点不足；
- `B_GAP_SENSOR_FAULT`：黄色区域内 IMU 或里程计中断。

二维激光雷达不能识别锥桶颜色、红绿灯颜色、A/B 标志牌或黄色地面语义，这些任务仍需要视觉或地图级定位模块。
