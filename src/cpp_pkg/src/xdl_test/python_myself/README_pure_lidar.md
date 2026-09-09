# pure_lidar

这是一个给 Davinci-mini 小车使用的“纯激光雷达走廊循迹”原型。
它不读取黄色线，也不保存赛道坐标，而是从 `/scan` 中提取左右两侧的点簇，拟合左右边界，计算赛道中心和中心线方向，再发布舵角和速度。

## 传感器和控制接口

默认订阅：

- `/scan`：`sensor_msgs/msg/LaserScan`
- `/IMU_data`：`sensor_msgs/msg/Imu`，只使用 `angular_velocity.z` 做转向阻尼
- `/odom_combined`：`nav_msgs/msg/Odometry`，使用线速度做超速保护

默认发布：

- `/teleop_cmd_vel`：`geometry_msgs/msg/Twist`
- 兼容现有驱动的约定：`linear.x=1500` 为中性，`angular.z=90` 为舵机中位

## 安全默认值

`config.yaml` 中 `enable_motion: false`。在这个默认状态下，节点只发布中性停车值，不会让车运动。先运行：

```bash
./dry_run_pure_lidar.sh
```

确认雷达回调、点簇数量和 `SAFE STOP` 逻辑正常后，才考虑在架空车轮、急停人员在场、低速空旷环境中测试。需要真正发布运动值时，必须显式使用：

```bash
./run_pure_lidar.sh -p enable_motion:=true
```

脚本不会自动加入开机启动，也不会覆盖原来的 `node1.py`。

## 算法说明

1. 按现有程序的坐标约定转换雷达点：`x=r*sin(theta)`（横向），`y=r*cos(theta)`（车头前方）。
2. 对相邻雷达点做距离连续性聚类，过滤无效距离和过大的连续面。
3. 将 `x<0` 的簇作为左侧候选、`x>0` 的簇作为右侧候选，分别拟合 `x=a*y+b`。
4. 在近距离和远距离处估计走廊中心，使用横向误差和中心线方向误差计算舵角。
5. 雷达前方有近障碍、雷达超时、左右边界都丢失时强制发送停车值。
6. 只有一侧边界时允许低速推断，置信度低会自动使用慢速档。

## 不能由纯雷达解决的任务

仅凭普通二维激光雷达不能可靠判断红绿灯颜色、A/B 牌内容、锥桶颜色或黄色线的语义。如果比赛规则要求这些任务，仍然需要视觉或其他状态输入；本程序只负责无固定黄色线的几何循迹和安全兜底。

## 现场调参顺序

1. 先用 `dry_run_pure_lidar.sh`，记录左右点簇数量、`center`、`heading`、`front`。
2. 确认雷达坐标方向：车头右侧是否对应正 `x`，若相反，将 `x=r*sin(theta)` 改为 `x=-r*sin(theta)`。
3. 先低速调 `cte_gain` 和 `heading_gain`，再调 `cluster_gap`、`side_min_y`、`side_max_y`。
4. 若误把墙面当作一个巨大点簇，提高 `min_cluster_points` 或降低 `max_cluster_radius`；若锥桶经常只有一个点，降低 `min_cluster_points` 到 1。
5. 任何失去边界仍继续沿用旧轨迹的行为都没有实现；本程序选择停车，避免失控。
