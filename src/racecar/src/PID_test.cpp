// PID_test.cpp  v6.5.7 DWA-lite 障碍感知版
// 基于锥桶/隔板的雷达循迹:聚类检测 -> 中线路径生成 -> 前视跟踪 -> DWA-lite 转向
//
// v6.5.7 增量:
//   15) 雷达前置补偿:err_after 用 proj + lidar_offset*tan(delta)(雷达在前轴前,
//       后轴模型会偏保守,弯道迟到)
//   16) 障碍模式 heading 权重 0.08 -> 0.02,绕障角不被"少打方向"压小
//   17) safe_turn 0.20 -> 0.35(0.2-0.175=2.5cm 余量太危险);硬停 0.22 -> 0.15
//   18) 强制绕障触发 0.7m -> 1.0m(高速 1m 内已晚)
//   7) 关键修复:straight_collide 判定放宽为 |zc-90| < step/2+0.001
//      (step=2 时候选角全奇数,90 永不出现,原条件恒 false -> 打满舵/死路停 两分支是死代码)
//   8) r<=0.01 极小值过滤(雷达贴脸噪声),聚类/墙段/点云全部同步
//   9) 配对 dy 速度自适应阈值提到循环外;碰撞扫掠 plen>0.01 防护
//   1) 墙段进入前方扇区(±front_deg=45°) -> 挡板障碍,不进赛道边界(中线不直指挡板)
//   2) DWA 前向碰撞:自行车圆弧按车宽扫掠 vs 障碍点云(0.15m 起),碰撞候选 +1000 绝对禁止
//   3) 前方距离排斥代价(挡板<1.2m 时近直行候选重罚) -> 提前 1 米感知绕行
//   4) 前方<0.7m 强制绕障模式;直行会撞但一侧有空 -> 打满舵向宽侧蠕行,仅两侧都堵才停
//   5) 转向范围 45~135°(需启动脚本包络 min_steering_pwm:=1000 max_steering_pwm:=2000)
//   6) 侧夹 dir 同步、safe_stop 0.45、锥桶丢失 4s 才停、速度地板 1512(1508 低于电机死区)
//
// 流程:
//   1. 聚类检测:动态间隙(0.15+0.03r)成簇,簇物理宽度 0.05~0.5m=锥桶,>0.5m=墙段
//   2. 左右分离:x>0 侧 / x≤0 侧(方向已实测:x>0=物理左侧)
//   3. 路径规划:左右按 y 排序同行配对(门宽校验+动态间距),每对中点=中线点;
//      取前视距离处的中线点作为目标 -> 提前转弯
//   4. 转向:DWA-lite 轨迹评价(候选角 45~135°,步长 dwa_step):
//      评分 = 走廊对齐 + 碰撞禁止(1000) + 前方距离排斥 + 车边TTC + heading + 平滑
//   5. 保护:正前方动态急停(两侧都堵才停)、转向侧过近禁止该方向、
//      弯道/近物减速、无桶巡航丢失超 4 秒(真实时间)停车
//
// 运行:
//   终端1: bash ~/racecar/start_car.sh
//          (内部 = ros2 launch racecar Run_car.launch.py enable_legacy_pwm_input:=true
//           min_steering_pwm:=1000 max_steering_pwm:=2000)
//   终端2: ros2 topic pub --once /nav/arm std_msgs/msg/Bool "{data: true}"
//   终端2: ros2 run racecar PID_test
// 注意:
//   - 不带 enable_legacy_pwm_input:=true -> driver 不收 /teleop_cmd_vel,车不动
//   - 包络必须是 [1000,2000](45°=2000,135°=1000),否则转向指令被驱动拒收停车
//   - speed 参数不得超过 1550(driver 油门包络上限)
//   - 每次开机都要重新 arm(重启后默认 DISARMED,驱动忽略一切指令)
// 调参(实时生效):
//   ros2 param set /PID_test w_align 1.0
//   ros2 param set /PID_test w_safe 3.0
//   ros2 param set /PID_test w_smooth 0.5
//   ros2 param set /PID_test lookahead 1.8
//   ros2 param set /PID_test speed 1520.0
//   ros2 param set /PID_test dir 1.0   (方向反了设 -1)
//   ros2 param set /PID_test front_deg 45.0  (前方障碍扇区半角)
//   ros2 param set /PID_test min_turn 16.0

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <chrono>
#include <cmath>
#include <clocale>
#include <functional>
#include <vector>

#define PI 3.14159265358979

class LaserControl : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    geometry_msgs::msg::Twist twist;

    double speed_;        // 油门 PWM,1500 停
    double speed_ratio_;  // PWM->m/s 标定系数
    double min_dist_;     // 排除该距离以内
    double max_dist_;     // 只参考该距离以内
    double back_dist_;    // 去除后方该距离以外
    double min_cone_deg_; // 锥桶最小角宽
    double lookahead_;    // 前视距离
    double safe_stop_;    // 前方急停距离(下限)
    double safe_turn_;    // 转向侧限角距离
    double pair_y_gap_;   // 同行配对 y 差阈值(米)
    double pair_min_w_, pair_max_w_;   // 赛道门宽范围(米)
    double max_slew_;     // 转向限速基础值(度/帧)
    double dwa_step_;     // 候选转向角步长(度)
    double wheelbase_;    // 轴距(米)
    double w_align_, w_safe_, w_smooth_, w_heading_;   // DWA 评分权重
    double min_turn_;     // 最小转向承诺(度):误差明显时至少打这么多
    double dir_ = 1.0;    // 方向:反了在线设 -1
    double front_deg_ = 45.0;  // 前方障碍扇区半角(度)
    double half_width_ = 0.6;   // 记忆走廊半宽
    double vehicle_hw_ = 0.175; // 车半宽(米):安全代价按车边算
    double lidar_offset_ = 0.15; // 雷达前置量(米):雷达在前轴前,DWA 后轴模型需补偿
    bool speed_deduct_extra_ = false;   // 豁口段整体降速标志
    double last_z_ = 90.0;   // 上一帧舵机角(限速用)
    std::chrono::steady_clock::time_point last_cone_t_;   // 最后一次看到锥桶的时间

    // 参数动态回调
    rcl_interfaces::msg::SetParametersResult param_cb(
        const std::vector<rclcpp::Parameter>& params)
    {
        for (const auto& p : params)
        {
            if (p.get_name() == "speed") speed_ = p.as_double();
            else if (p.get_name() == "lookahead") lookahead_ = p.as_double();
            else if (p.get_name() == "w_align") w_align_ = p.as_double();
            else if (p.get_name() == "w_safe") w_safe_ = p.as_double();
            else if (p.get_name() == "w_smooth") w_smooth_ = p.as_double();
            else if (p.get_name() == "w_heading") w_heading_ = p.as_double();
            else if (p.get_name() == "dir") dir_ = p.as_double();
            else if (p.get_name() == "min_turn") min_turn_ = p.as_double();
            else if (p.get_name() == "pwm_speed_ratio") speed_ratio_ = p.as_double();
            else if (p.get_name() == "front_deg") front_deg_ = p.as_double();
        }
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
    }

    void callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
    {
        double left_pts[200][2], right_pts[200][2];   // [x, y]
        double center_pts[200][2];                    // 正中桶(|x|<0.15)
        double obs_x[720], obs_y[720];                // 障碍点云(DWA碰撞检测用)
        int n_l = 0, n_r = 0, n_c = 0, n_obs = 0;
        speed_deduct_extra_ = false;
        double est_v = (speed_ - 1500.0) * speed_ratio_;   // PWM->m/s
        if (est_v < 0.1) est_v = 0.1;
        double lookahead_eff = lookahead_ + est_v * 1.2;   // 前视随速度
        double front_cos = std::cos(front_deg_ * PI / 180.0);
        double nearest_front = 100.0, nearest_l = 100.0, nearest_r = 100.0;
        size_t n = laser->ranges.size();

        // ---------- 1. 聚类检测 + 障碍点云 ----------
        for (size_t i = 0; i < n; i++)
        {
            double r = laser->ranges[i];
            // v6.5.4: 过滤雷达极小非法返回值(0<r<=0.01)
            if (!std::isfinite(r) || r <= 0.01) continue;

            double th0 = laser->angle_min + i * laser->angle_increment;
            double y0 = r * std::cos(th0);
            double x0 = r * std::sin(th0);

            // 障碍点云(隔点采样,供 DWA 前向碰撞检测;0.15m 起,贴脸也参与)
            if (r >= 0.15 && r <= max_dist_ && y0 > -back_dist_ && (i % 2 == 0) && n_obs < 720)
            {
                obs_x[n_obs] = x0; obs_y[n_obs] = y0; n_obs++;
            }

            // 前方防撞:扇区 ±front_deg(cos(θ) > cos(front_deg))
            if (y0 > front_cos * r && r < max_dist_ && y0 < nearest_front) nearest_front = y0;

            // 侧向防撞:任意点(含 0.5m 内)参与两侧最近距离记录
            if (y0 > -0.1)
            {
                if (x0 > 0) { if (r < nearest_l) nearest_l = r; }
                else        { if (r < nearest_r) nearest_r = r; }
            }

            if (r <= min_dist_ || r > max_dist_) continue;
            if (y0 < -back_dist_) continue;

            size_t end = i;
            while (end + 1 < n)
            {
                double r1 = laser->ranges[end], r2 = laser->ranges[end + 1];
                if (!std::isfinite(r1) || !std::isfinite(r2)) break;
                if (r2 <= 0.01 || r2 > max_dist_) break;
                double gap = 0.15 + 0.03 * r1;   // 动态聚类间隙
                if (std::fabs(r2 - r1) > gap) break;
                end++;
            }
            double ang_w = (double)(end - i) * laser->angle_increment * 180.0 / PI;
            double sum_r = 0.0;
            int cnt = 0;
            for (size_t k = i; k <= end; k++)
            {
                double rk = laser->ranges[k];
                if (std::isfinite(rk) && rk > 0.01) { sum_r += rk; cnt++; }
            }
            if (cnt == 0) { i = end; continue; }
            double rm = sum_r / (double)cnt;
            double cw = (double)(end - i) * laser->angle_increment * rm;   // 簇物理宽
            if (ang_w >= min_cone_deg_ && cw >= 0.05 && cw <= 0.5)
            {
                // 锥桶:物理宽度 0.05~0.5m
                double thm = laser->angle_min + (i + end) * 0.5 * laser->angle_increment;
                double x = rm * std::sin(thm);
                double y = rm * std::cos(thm);
                if (y >= -back_dist_)
                {
                    if (std::fabs(x) < 0.15)
                    {
                        if (n_c < 200) { center_pts[n_c][0] = x; center_pts[n_c][1] = y; n_c++; }
                    }
                    else if (x > 0 && n_l < 200) { left_pts[n_l][0] = x; left_pts[n_l][1] = y; n_l++; }
                    else if (x <= 0 && n_r < 200) { right_pts[n_r][0] = x; right_pts[n_r][1] = y; n_r++; }
                    if (x > 0) { if (rm < nearest_l) nearest_l = rm; }
                    else       { if (rm < nearest_r) nearest_r = rm; }
                }
            }
            else if (cw > 0.5 && ang_w <= 160.0)
            {
                // 墙段:只要任何部分进入前方扇区 -> 挡板障碍,不生成边界点
                bool front_block = false;
                for (size_t k = i; k <= end; k++)
                {
                    double rk = laser->ranges[k];
                    if (!std::isfinite(rk) || rk <= 0.01) continue;
                    double thk = laser->angle_min + k * laser->angle_increment;
                    double yy = rk * std::cos(thk);
                    if (yy > 0 && yy > front_cos * rk) { front_block = true; break; }
                }
                if (!front_block)
                {
                    // 侧向墙/隔板:在前视距离处采样一个虚拟锥桶点(赛道边界)
                    double best_dy = 1e9;
                    double vx = 0.0, vy = 0.0;
                    for (size_t k = i; k <= end; k++)
                    {
                        double rk = laser->ranges[k];
                        if (!std::isfinite(rk) || rk <= 0.01) continue;
                        double thk = laser->angle_min + k * laser->angle_increment;
                        double xx = rk * std::sin(thk);
                        double yy = rk * std::cos(thk);
                        if (yy < -back_dist_) continue;
                        if (std::fabs(xx) < 0.2) continue;   // 跳过正前方板(不污染走廊中心)
                        double d = std::fabs(yy - lookahead_eff);
                        if (d < best_dy) { best_dy = d; vx = xx; vy = yy; }
                    }
                    if (vx > 0 && n_l < 200) { left_pts[n_l][0] = vx; left_pts[n_l][1] = vy; n_l++; }
                    else if (vx < 0 && n_r < 200) { right_pts[n_r][0] = vx; right_pts[n_r][1] = vy; n_r++; }
                }
            }
            i = end;
        }

        // ---------- 前方急停距离(v²/(2a)+余量,下限 safe_stop_) ----------
        double stop_dist = est_v * est_v / 4.0 + 0.15;
        if (stop_dist < safe_stop_) stop_dist = safe_stop_;

        // ---------- 无锥桶:前方有挡板 -> 障碍导航;前方干净 -> 巡航 ----------
        bool obstacle_mode = false;
        if (n_l + n_r + n_c == 0)
        {
            if (nearest_front < lookahead_eff + 0.3)
            {
                // 挡板堵路:进入障碍导航(目标=正前方,碰撞代价负责选绕行角)
                obstacle_mode = true;
                last_cone_t_ = std::chrono::steady_clock::now();
            }
            else
            {
                twist.angular.z = last_z_;
                double spd = speed_ - 15;
                double floor_s2 = (speed_ > 1500.0) ? 1515.0 : 1500.0;
                if (spd < floor_s2) spd = floor_s2;
                twist.linear.x = spd;
                double lost_s = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - last_cone_t_).count();
                if (lost_s > 4.0)                          // 4 秒没看到锥桶 -> 停车(防间歇性聚类丢失误停)
                {
                    twist.linear.x = 1500;
                    twist.angular.z = 90;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "锥桶丢失超 4 秒,停车等待");
                }
                pub_->publish(twist);
                return;
            }
        }
        else if (nearest_front < 1.0)
        {
            // 前方过近(1.0m 内)无论是否检测到桶 -> 强制绕障(防挡板被误切成桶后直撞)
            obstacle_mode = true;
        }
        last_cone_t_ = std::chrono::steady_clock::now();

        // ---------- 2/3. 中线路径生成 + 前视目标 ----------
        int i;
        double target_x = 0.0;
        bool have_target = obstacle_mode;   // 障碍导航:目标=直前方
        if (obstacle_mode) speed_deduct_extra_ = true;

        if (!obstacle_mode)
        {
            // 左右按 y 排序
            for (int a = 0; a < n_l - 1; a++)
                for (int b = 0; b < n_l - a - 1; b++)
                    if (left_pts[b][1] > left_pts[b + 1][1])
                    {
                        double tx = left_pts[b][0], ty = left_pts[b][1];
                        left_pts[b][0] = left_pts[b + 1][0]; left_pts[b][1] = left_pts[b + 1][1];
                        left_pts[b + 1][0] = tx; left_pts[b + 1][1] = ty;
                    }
            for (int a = 0; a < n_r - 1; a++)
                for (int b = 0; b < n_r - a - 1; b++)
                    if (right_pts[b][1] > right_pts[b + 1][1])
                    {
                        double tx = right_pts[b][0], ty = right_pts[b][1];
                        right_pts[b][0] = right_pts[b + 1][0]; right_pts[b][1] = right_pts[b + 1][1];
                        right_pts[b + 1][0] = tx; right_pts[b + 1][1] = ty;
                    }

            // 更新记忆半宽(前视附近,记忆衰减)
            if (n_l > 0 && n_r > 0)
            {
                double lsum = 0.0, rsum = 0.0;
                int nl2 = 0, nr2 = 0;
                for (i = 0; i < n_l; i++)
                    if (left_pts[i][1] <= lookahead_eff + 0.5) { lsum += std::fabs(left_pts[i][0]); nl2++; }
                for (i = 0; i < n_r; i++)
                    if (right_pts[i][1] <= lookahead_eff + 0.5) { rsum += std::fabs(right_pts[i][0]); nr2++; }
                if (nl2 > 0 && nr2 > 0)
                {
                    double hw_new = (lsum / nl2 + rsum / nr2) / 2.0;
                    half_width_ = 0.8 * half_width_ + 0.2 * hw_new;
                    if (half_width_ < 0.3) half_width_ = 0.3;
                }
            }

            bool used_r[200] = {false};
            int pair_cnt = 0, valid_cnt = 0;
            double best_d2 = 1e9;

            if (n_l > 0 && n_r > 0)
            {
                // 第一轮:同行配对(y 差动态放宽 + 门宽过滤,不烧桶)
                // v6.5.4: dy 阈值提到循环外,速度自适应一致生效
                double base_dy_thr = std::max(pair_y_gap_, 0.5 + 0.2 * est_v);
                for (i = 0; i < n_l; i++)
                {
                    double best_dy = base_dy_thr;
                    int best_ri = -1;
                    for (int a = 0; a < n_r; a++)
                    {
                        if (used_r[a]) continue;
                        double w = left_pts[i][0] - right_pts[a][0];
                        if (w < pair_min_w_ || w > pair_max_w_) continue;
                        double dy = std::fabs(left_pts[i][1] - right_pts[a][1]);
                        if (dy < best_dy) { best_dy = dy; best_ri = a; }
                    }
                    if (best_ri >= 0)
                    {
                        used_r[best_ri] = true;
                        pair_cnt++;
                        valid_cnt++;
                        double px = (left_pts[i][0] + right_pts[best_ri][0]) / 2.0;
                        double py = (left_pts[i][1] + right_pts[best_ri][1]) / 2.0;
                        double d = std::fabs(py - lookahead_eff);
                        if (d < best_d2) { best_d2 = d; target_x = px; have_target = true; }
                    }
                }
                // 第二轮:同行没配上 -> 最近邻回退
                if (pair_cnt == 0)
                {
                    bool used2[200] = {false};
                    for (i = 0; i < n_l; i++)
                    {
                        double best_d = 1e9;
                        int best_ri = -1;
                        for (int a = 0; a < n_r; a++)
                        {
                            if (used2[a]) continue;
                            double w = left_pts[i][0] - right_pts[a][0];
                            if (w < pair_min_w_ || w > pair_max_w_) continue;
                            double dx = left_pts[i][0] - right_pts[a][0];
                            double dy = left_pts[i][1] - right_pts[a][1];
                            double d = std::sqrt(dx * dx + dy * dy);
                            if (d < best_d) { best_d = d; best_ri = a; }
                        }
                        if (best_ri >= 0)
                        {
                            used2[best_ri] = true;
                            valid_cnt++;
                            double px = (left_pts[i][0] + right_pts[best_ri][0]) / 2.0;
                            double py = (left_pts[i][1] + right_pts[best_ri][1]) / 2.0;
                            double d = std::fabs(py - lookahead_eff);
                            if (d < best_d2) { best_d2 = d; target_x = px; have_target = true; }
                        }
                    }
                }
                // 第三轮:全部不合格 -> 双侧均值中点 + 降速
                if (valid_cnt == 0)
                {
                    double lsum = 0.0, rsum = 0.0;
                    for (i = 0; i < n_l; i++) lsum += left_pts[i][0];
                    for (i = 0; i < n_r; i++) rsum += right_pts[i][0];
                    target_x = (lsum / n_l + rsum / n_r) / 2.0;
                    have_target = true;
                    speed_deduct_extra_ = true;
                }
            }
            else if (n_l >= 2)
            {
                double best_d = 1e9;
                for (i = 0; i < n_l; i++)
                {
                    double d = std::fabs(left_pts[i][1] - lookahead_eff);
                    if (d < best_d) { best_d = d; target_x = left_pts[i][0] - half_width_; have_target = true; }
                }
            }
            else if (n_r >= 2)
            {
                double best_d = 1e9;
                for (i = 0; i < n_r; i++)
                {
                    double d = std::fabs(right_pts[i][1] - lookahead_eff);
                    if (d < best_d) { best_d = d; target_x = right_pts[i][0] + half_width_; have_target = true; }
                }
            }
            else if (n_l == 1 || n_r == 1)
            {
                // 单侧 1 桶:不跟行,直行降速等更多桶
                target_x = 0.0;
                have_target = true;
                speed_deduct_extra_ = true;
            }
            else if (n_c > 0)
            {
                double best_d = 1e9;
                for (i = 0; i < n_c; i++)
                {
                    double d = std::fabs(center_pts[i][1] - lookahead_eff);
                    if (d < best_d) { best_d = d; target_x = center_pts[i][0]; have_target = true; }
                }
            }

            // dir_ 只在坐标处乘一次,不参与 DWA 内部(方向反转时 proj 不受影响)
            target_x *= dir_;
        }
        else
        {
            // v6.5.6: 障碍模式主动绕障——目标指向空闲空间大的一侧。
            // 不再乘 dir:nearest_l/r 就是物理侧(雷达原始坐标系),dir 只影响最终舵机方向
            target_x = (nearest_l > nearest_r) ? 0.5 : -0.5;
        }
        // 无有效目标:停车(防盲开)
        if (!have_target)
        {
            twist.linear.x = 1500; twist.angular.z = 90;
            pub_->publish(twist);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "无有效目标,停车等待");
            return;
        }

        // ---------- 4. DWA-lite:候选转向角轨迹评价(含前向碰撞检测) ----------
        double best_z = 90.0, best_cost = 1e18;
        bool straight_collide = false;
        double step = (dwa_step_ >= 0.5) ? dwa_step_ : 2.0;   // 参数保护
        double wb = (wheelbase_ >= 0.05) ? wheelbase_ : 0.305;
        double hw = (vehicle_hw_ > 0.05) ? vehicle_hw_ : 0.175;
        for (double zc = 45.0; zc <= 135.0001; zc += step)
        {
            double delta = (zc - 90.0) * PI / 180.0;   // 左转为正
            // 精确自行车模型 + copysign 符号修正 + 预测距离限制
            double proj = 0.0, rr = 0.0, plen = lookahead_eff;
            if (std::fabs(delta) > 0.01)
            {
                double R = wb / std::tan(delta);
                rr = std::fabs(R);
                plen = std::min(lookahead_eff, rr * 1.5);   // 大角不过弧
                proj = std::copysign(rr * (1.0 - std::cos(plen / rr)), delta);
            }

            // 轨迹前向碰撞:圆弧分段(10段)以车宽(hw+0.05)扫掠 vs 障碍点云
            bool collide = false;
            if (n_obs > 0 && plen > 0.01)
            {
                double cd = hw + 0.05;
                double cd2 = cd * cd;
                for (int s = 1; s <= 10; s++)
                {
                    double seg = plen * (double)s / 10.0;
                    double px = 0.0, py = seg;
                    if (std::fabs(delta) > 0.01)
                    {
                        double th = seg / rr;
                        px = std::copysign(rr * (1.0 - std::cos(th)), delta);
                        py = rr * std::sin(th);
                    }
                    for (int o = 0; o < n_obs; o++)
                    {
                        double dx = obs_x[o] - px, dy = obs_y[o] - py;
                        if (dx * dx + dy * dy < cd2) { collide = true; break; }
                    }
                    if (collide) break;
                }
            }
            // v6.5.4 关键修复:step=2 时候选角全为奇数(45,47,...,89,91),90 永不出现;
            // |89-90|=1.0 < step/2=1.0 恒 false -> straight_collide 永不触发。
            // 放宽为 < step/2 + 0.001,89/91 正确识别为直行候选。
            if (std::fabs(zc - 90.0) < step / 2.0 + 0.001 && collide) straight_collide = true;

            double err_after = target_x - (proj + lidar_offset_ * std::tan(delta));
            // v6.5.6: 障碍模式下目标权重加大(3x),让"绕哪边"意图压过平滑/heading,
            // 不会被 smooth 项反向选择
            double align_w = obstacle_mode ? 3.0 : w_align_;
            double cost = align_w * std::fabs(err_after);
            if (collide)
            {
                cost += 1000.0;   // 碰撞候选绝对禁止(死路时整体抬高,仍取相对最优)
            }
            else
            {
                // 车边 TTC 代价(dir 反向时避障侧同步翻转)
                double clear;
                if (dir_ > 0) clear = ((zc > 90.0) ? nearest_l : nearest_r) - hw;
                else          clear = ((zc > 90.0) ? nearest_r : nearest_l) - hw;
                if (clear < 1.0) cost += w_safe_ * est_v / std::max(clear, 0.1);
            }

            // v6.5.7: 障碍模式下 heading 权重降为 0.02,绕障角不被"少打方向"代价压小
            double heading_w = obstacle_mode ? 0.02 : w_heading_;
            cost += heading_w * std::fabs(delta);             // heading 代价:少打大角
            cost += w_smooth_ * std::fabs(zc - last_z_);      // 平滑

            // 前方距离排斥代价:挡板越近,近直行候选(±20°)重罚 -> 提前感知并绕行
            if (nearest_front < 1.2 && std::fabs(zc - 90.0) < 20.0)
                cost += 15.0 / std::max(nearest_front, 0.2);

            if (cost < best_cost) { best_cost = cost; best_z = zc; }
        }

        twist.angular.z = best_z;

        // 最小转向承诺(限速之前执行,限速随后限制跳变幅度):
        // 误差明显时转向至少 min_turn 度,防小角度低于舵机分辨率直跑
        if (!obstacle_mode && std::fabs(target_x) > 0.12)
        {
            double sign = (target_x > 0) ? 1.0 : -1.0;
            if (std::fabs(twist.angular.z - 90.0) < min_turn_)
                twist.angular.z = 90.0 + sign * min_turn_;
        }

        // ---------- 保护 2:转向侧过近 -> 禁止该方向(直接夹到 90) ----------
        // dir 反向时舵机方向与雷达侧同步翻转:物理左转 = (z>90 且 dir>0) 或 (z<90 且 dir<0)
        bool safe_limit = false;
        bool turn_left = (twist.angular.z > 90.0 && dir_ > 0.0) || (twist.angular.z < 90.0 && dir_ < 0.0);
        if (turn_left && nearest_l < safe_turn_) { twist.angular.z = 90; safe_limit = true; }
        if (!turn_left && nearest_r < safe_turn_) { twist.angular.z = 90; safe_limit = true; }

        // ---------- 转向限速:误差大时放宽,安全帧不限制 ----------
        if (!safe_limit)
        {
            double slew = max_slew_ + std::fabs(twist.angular.z - 90.0) * 0.3;
            double dz = twist.angular.z - last_z_;
            if (dz > slew) twist.angular.z = last_z_ + slew;
            if (dz < -slew) twist.angular.z = last_z_ - slew;
        }
        last_z_ = twist.angular.z;

        // ---------- 速度:前方过近 -> 蠕行绕行;真死路(两侧都堵)才停 ----------
        double speed = speed_;
        double steer_deg = std::fabs(twist.angular.z - 90.0);
        bool side_blocked = (nearest_l < 0.35 && nearest_r < 0.35);   // 两侧都堵=死路
        if (nearest_front < 0.15)
        {
            // 太近 -> 死路,停车(0.15m 内才是真贴脸;0.15~0.3 交给绕障分支)
            speed = 1500.0;
            twist.angular.z = 90.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "正前方 %.2fm 死路,停车", nearest_front);
        }
        else if (nearest_front < stop_dist)
        {
            if (straight_collide && steer_deg < 6.0 && !side_blocked)
            {
                // 直行会撞但一侧有空 -> 强制向空间大的一侧打,蠕行绕过去(不再傻停)
                // v6.5.5: 强制角也走限幅,防 90->120->90 蛇形
                double target_z = (nearest_l > nearest_r) ? 120.0 : 60.0;
                double dz2 = target_z - last_z_;
                double slew2 = max_slew_ + std::fabs(target_z - 90.0) * 0.3;
                if (dz2 > slew2) target_z = last_z_ + slew2;
                if (dz2 < -slew2) target_z = last_z_ - slew2;
                twist.angular.z = target_z;
                speed = 1515.0;   // 蠕行(1515 越过电机静摩擦区)
            }
            else if (straight_collide && steer_deg < 6.0)
            {
                speed = 1500.0;
                twist.angular.z = 90.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "正前方 %.2fm 死路(两侧均堵),停车", nearest_front);
            }
            else
            {
                speed = 1515.0;   // 蠕行(1515 越过电机静摩擦区),DWA 碰撞代价负责绕开挡板
            }
        }
        else
        {
            if (speed_deduct_extra_) speed -= 10;
            if (steer_deg > 20) speed -= 5;
            if (steer_deg > 35) speed -= 10;
            if (nearest_l < 0.5 || nearest_r < 0.5) speed -= 8;
            if (nearest_front < 1.0) speed -= 10;   // 提前减速:1.0m 内前方有物
            if (nearest_front < 0.5) speed -= 10;   // 更近再加一档
            double floor_s = (speed_ > 1500.0) ? 1512.0 : 1500.0;   // 地板 1512:低于电机死区,车不动
            if (speed < floor_s) speed = floor_s;
        }
        twist.linear.x = speed;

        pub_->publish(twist);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "左:%d 右:%d 目标x:%.2f 前:%.2f 左近:%.2f 右近:%.2f obs:%d 舵机角:%.1f 速度:%.0f%s",
                             n_l, n_r, target_x, nearest_front, nearest_l, nearest_r, n_obs,
                             twist.angular.z, twist.linear.x, obstacle_mode ? " [绕障]" : "");
    }

public:
    LaserControl() : Node("PID_test")
    {
        this->declare_parameter("speed", 1520.0);
        this->declare_parameter("pwm_speed_ratio", 0.025);   // PWM->m/s 标定(现场实测)
        this->declare_parameter("min_dist", 0.5);
        this->declare_parameter("max_dist", 3.5);
        this->declare_parameter("back_dist", 0.2);
        this->declare_parameter("min_cone_deg", 2.0);
        this->declare_parameter("lookahead", 1.8);
        this->declare_parameter("safe_stop", 0.45);
        this->declare_parameter("safe_turn", 0.35);
        this->declare_parameter("pair_y_gap", 0.8);
        this->declare_parameter("pair_min_w", 0.4);
        this->declare_parameter("pair_max_w", 3.0);
        this->declare_parameter("max_slew", 6.0);
        this->declare_parameter("dwa_step", 2.0);
        this->declare_parameter("wheelbase", 0.305);
        this->declare_parameter("w_align", 1.0);
        this->declare_parameter("w_safe", 3.0);
        this->declare_parameter("w_smooth", 0.05);       // 平滑权重(小!否则锁死舵机不转)
        this->declare_parameter("w_heading", 0.08);      // heading 权重(小)
        this->declare_parameter("min_turn", 16.0);       // 最小转向承诺(度):10° 在实车上太弱,几乎看不出转弯
        this->declare_parameter("dir", 1.0);
        this->declare_parameter("front_deg", 45.0);      // 前方障碍扇区半角(度)
        this->declare_parameter("lidar_offset", 0.15);   // 雷达前置量(米):DWA 后轴模型补偿

        speed_ = this->get_parameter("speed").as_double();
        speed_ratio_ = this->get_parameter("pwm_speed_ratio").as_double();
        min_dist_ = this->get_parameter("min_dist").as_double();
        max_dist_ = this->get_parameter("max_dist").as_double();
        back_dist_ = this->get_parameter("back_dist").as_double();
        min_cone_deg_ = this->get_parameter("min_cone_deg").as_double();
        lookahead_ = this->get_parameter("lookahead").as_double();
        safe_stop_ = this->get_parameter("safe_stop").as_double();
        safe_turn_ = this->get_parameter("safe_turn").as_double();
        pair_y_gap_ = this->get_parameter("pair_y_gap").as_double();
        pair_min_w_ = this->get_parameter("pair_min_w").as_double();
        pair_max_w_ = this->get_parameter("pair_max_w").as_double();
        max_slew_ = this->get_parameter("max_slew").as_double();
        dwa_step_ = this->get_parameter("dwa_step").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        w_align_ = this->get_parameter("w_align").as_double();
        w_safe_ = this->get_parameter("w_safe").as_double();
        w_smooth_ = this->get_parameter("w_smooth").as_double();
        w_heading_ = this->get_parameter("w_heading").as_double();
        min_turn_ = this->get_parameter("min_turn").as_double();
        dir_ = this->get_parameter("dir").as_double();
        front_deg_ = this->get_parameter("front_deg").as_double();
        lidar_offset_ = this->get_parameter("lidar_offset").as_double();

        twist.linear.x = 1500;
        twist.angular.z = 90;
        last_cone_t_ = std::chrono::steady_clock::now();

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", 5);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 5, std::bind(&LaserControl::callback, this, std::placeholders::_1));
        this->add_on_set_parameters_callback(
            std::bind(&LaserControl::param_cb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "PID_test v6.5.7 DWA-lite 障碍感知版 前视=%.1fm 速度=%.0f 权重 %.1f/%.1f/%.1f/%.1f 前扇区=%.0f°",
                    lookahead_, speed_, w_align_, w_safe_, w_smooth_, w_heading_, front_deg_);
    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
