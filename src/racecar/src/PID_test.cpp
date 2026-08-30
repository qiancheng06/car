// PID_test.cpp  v6.6.3 Cone-DWA 纯锥桶版(可编译版 + 候选/检测长度修复)
// 基于 v6.6.2,按评审修复:
//  P0-1) 补回丢失的成员变量:twist / pub_ / sub_(v6.6.2 重构遗漏,必编译错)
//  P1-1) DWA 候选保证 90° 必选(step=2 时 45,47,...,89,91 无 90 的漏洞)
//  P1-2) 碰撞检测长度不再被 1.5R 截短:plen = lookahead_eff 全程检查
//        (急转弯时最终复检必须覆盖承诺的前视范围)
//  P1-3) MAX_SAFE_CAND 64 -> 256(dwa_step=0.5 时 181 个候选不再截断)
//  P1-4) last_z_ 只在最终安全复检通过后更新一次(只保存真实执行角)
//  P1-5) 启动参数完整验证:sanitize_params() 与运行时同规则
//        (启动越界参数回退默认值并告警)
//  P2-1) 参数关系验证不依赖顺序:阶段1 写入副本,阶段2 统一检查
//        max_dist>min_dist / max_cone_deg>min_cone_deg /
//        max_cone_w>min_cone_w / cone_creep<=cone_brake / pair_max_w>pair_min_w
//  P2-2) dir 严格 ±1(999/-999 直接拒绝)
// 决策逻辑与 v6.6.2 一致(Cone-DWA、最终唯一出口、dir 仅输出映射)
//
// 消息契约(本项目自定义,勿改):
//   /teleop_cmd_vel linear.x = 油门 PWM(1500 停)
//                    angular.z = 舵机角度(90 中, >90 左, <90 右)
//
// 运行:
//   终端1: bash ~/racecar/start_car.sh
//   终端2: ros2 run racecar PID_test
// 调参(全部实时生效;非法值被拒绝并打印 reason):
//   ros2 param set /PID_test speed 1520.0
//   ros2 param set /PID_test dir -1.0
//   ros2 param set /PID_test cone_radius 0.06
//   ros2 param set /PID_test vehicle_radius 0.22
//   ros2 param set /PID_test safety_margin 0.05
//   ros2 param set /PID_test w_cone_safe 1.5
//   ros2 param set /PID_test cone_brake 0.8
//   ros2 param set /PID_test cone_creep 0.4

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <clocale>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#define PI 3.14159265358979
#define MAX_OBS 512
#define MAX_RAW_OBS 384        // 原始点上限,给锥桶预留 128
#define MAX_CONES 200
#define MAX_SAFE_CAND 256      // P1-3: 覆盖 dwa_step=0.5 的 181 候选

class LaserControl : public rclcpp::Node
{
private:
    // ============ 参数结构体 ============
    struct Params
    {
        double speed = 1520.0;
        double speed_ratio = 0.025;
        double speed_quad = 0.0;
        double decel = 2.0;
        double min_dist = 0.5;
        double max_dist = 3.5;
        double back_dist = 0.2;
        double min_cone_deg = 2.0;
        double max_cone_deg = 20.0;
        double min_cone_w = 0.05;
        double max_cone_w = 0.30;
        double cone_radius = 0.06;
        double vehicle_radius = 0.22;
        double safety_margin = 0.05;
        double w_cone_safe = 1.5;
        double cone_brake = 0.8;
        double cone_creep = 0.4;
        double lookahead = 1.8;
        double safe_stop = 0.45;
        double safe_turn = 0.35;
        double pair_y_gap = 0.8;
        double pair_min_w = 0.4;
        double pair_max_w = 3.0;
        double max_slew = 6.0;
        double dwa_step = 2.0;
        double wheelbase = 0.305;
        double w_align = 1.0;
        double w_safe = 3.0;
        double w_smooth = 0.05;
        double w_heading = 0.08;
        double steer_commit_deg = 16.0;
        double dir = 1.0;
        double front_deg = 45.0;
        double lidar_offset = 0.15;
    } p_;

    // ============ P0-1: ROS 通信成员(必须声明) ============
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    geometry_msgs::msg::Twist twist;

    // 非参数状态
    double half_width_ = 0.6;
    bool speed_deduct_extra_ = false;
    double last_z_ = 90.0;   // P1-4: 只保存"最终安全复检通过后的真实执行角"
    std::chrono::steady_clock::time_point last_cone_t_;

    // ============ 参数验证(P1-5/P2-1) ============
    // 独立范围检查(不依赖其他参数):非法返回 false
    static bool in_range(const std::string& nm, double v, double lo, double hi)
    {
        return std::isfinite(v) && v >= lo && v <= hi;
    }
    // 关系检查(阶段2 统一做,不依赖参数处理顺序)
    bool relations_ok(const Params& q, std::string& why)
    {
        if (q.max_dist <= q.min_dist + 0.1) { why = "max_dist must be > min_dist"; return false; }
        if (q.max_cone_deg <= q.min_cone_deg + 0.5) { why = "max_cone_deg must be > min_cone_deg"; return false; }
        if (q.max_cone_w <= q.min_cone_w + 0.01) { why = "max_cone_w must be > min_cone_w"; return false; }
        if (q.cone_creep > q.cone_brake) { why = "cone_creep must be <= cone_brake"; return false; }
        if (q.pair_max_w <= q.pair_min_w + 0.1) { why = "pair_max_w must be > pair_min_w"; return false; }
        return true;
    }
    // 逐字段范围检查(不依赖顺序)
    bool ranges_ok(const Params& q, std::string& why)
    {
        if (!in_range("speed", q.speed, 1000.0, 2500.0)) { why = "speed: 1000~2500"; return false; }
        if (!in_range("pwm_speed_ratio", q.speed_ratio, 0.0, 1.0)) { why = "pwm_speed_ratio: 0~1"; return false; }
        if (!in_range("speed_quad", q.speed_quad, 0.0, 1.0)) { why = "speed_quad: 0~1"; return false; }
        if (!in_range("decel", q.decel, 0.1, 20.0)) { why = "decel: >=0.1"; return false; }
        if (!in_range("min_dist", q.min_dist, 0.05, 5.0)) { why = "min_dist: 0.05~5"; return false; }
        if (!in_range("max_dist", q.max_dist, 0.15, 30.0)) { why = "max_dist: 0.15~30"; return false; }
        if (!in_range("back_dist", q.back_dist, 0.0, 5.0)) { why = "back_dist: 0~5"; return false; }
        if (!in_range("min_cone_deg", q.min_cone_deg, 0.1, 10.0)) { why = "min_cone_deg: 0.1~10"; return false; }
        if (!in_range("max_cone_deg", q.max_cone_deg, 0.6, 90.0)) { why = "max_cone_deg: 0.6~90"; return false; }
        if (!in_range("min_cone_w", q.min_cone_w, 0.01, 1.0)) { why = "min_cone_w: 0.01~1"; return false; }
        if (!in_range("max_cone_w", q.max_cone_w, 0.02, 5.0)) { why = "max_cone_w: 0.02~5"; return false; }
        if (!in_range("cone_radius", q.cone_radius, 0.0, 0.5)) { why = "cone_radius: 0~0.5"; return false; }
        if (!in_range("vehicle_radius", q.vehicle_radius, 0.05, 1.0)) { why = "vehicle_radius: 0.05~1"; return false; }
        if (!in_range("safety_margin", q.safety_margin, 0.0, 1.0)) { why = "safety_margin: 0~1"; return false; }
        if (!in_range("w_cone_safe", q.w_cone_safe, 0.0, 100.0)) { why = "w_cone_safe: 0~100"; return false; }
        if (!in_range("cone_brake", q.cone_brake, 0.1, 5.0)) { why = "cone_brake: 0.1~5"; return false; }
        if (!in_range("cone_creep", q.cone_creep, 0.05, 5.0)) { why = "cone_creep: 0.05~5"; return false; }
        if (!in_range("lookahead", q.lookahead, 0.3, 5.0)) { why = "lookahead: 0.3~5"; return false; }
        if (!in_range("safe_stop", q.safe_stop, 0.1, 3.0)) { why = "safe_stop: 0.1~3"; return false; }
        if (!in_range("safe_turn", q.safe_turn, 0.05, 2.0)) { why = "safe_turn: 0.05~2"; return false; }
        if (!in_range("pair_y_gap", q.pair_y_gap, 0.1, 3.0)) { why = "pair_y_gap: 0.1~3"; return false; }
        if (!in_range("pair_min_w", q.pair_min_w, 0.1, 3.0)) { why = "pair_min_w: 0.1~3"; return false; }
        if (!in_range("pair_max_w", q.pair_max_w, 0.2, 20.0)) { why = "pair_max_w: 0.2~20"; return false; }
        if (!in_range("max_slew", q.max_slew, 0.5, 60.0)) { why = "max_slew: 0.5~60"; return false; }
        if (!in_range("dwa_step", q.dwa_step, 0.5, 10.0)) { why = "dwa_step: 0.5~10"; return false; }
        if (!in_range("wheelbase", q.wheelbase, 0.05, 2.0)) { why = "wheelbase: 0.05~2"; return false; }
        if (!in_range("w_align", q.w_align, 0.0, 100.0)) { why = "w_align: 0~100"; return false; }
        if (!in_range("w_safe", q.w_safe, 0.0, 100.0)) { why = "w_safe: 0~100"; return false; }
        if (!in_range("w_smooth", q.w_smooth, 0.0, 100.0)) { why = "w_smooth: 0~100"; return false; }
        if (!in_range("w_heading", q.w_heading, 0.0, 100.0)) { why = "w_heading: 0~100"; return false; }
        if (!in_range("steer_commit_deg", q.steer_commit_deg, 0.0, 90.0)) { why = "steer_commit_deg: 0~90"; return false; }
        if (q.dir != 1.0 && q.dir != -1.0) { why = "dir must be +1 or -1"; return false; }
        if (!in_range("front_deg", q.front_deg, 5.0, 89.0)) { why = "front_deg: 5~89"; return false; }
        if (!in_range("lidar_offset", q.lidar_offset, 0.0, 1.0)) { why = "lidar_offset: 0~1"; return false; }
        return true;
    }

    // P1-5: 启动参数完整验证(越界回退默认值并告警)
    void sanitize_params()
    {
        std::string why;
        if (!ranges_ok(p_, why))
        {
            RCLCPP_WARN(this->get_logger(), "启动参数非法(%s),已回退默认值", why.c_str());
            p_ = Params();
        }
        else if (!relations_ok(p_, why))
        {
            RCLCPP_WARN(this->get_logger(), "启动参数关系非法(%s),已回退默认值", why.c_str());
            p_ = Params();
        }
    }

    // 运行时参数回调:P2-1 两阶段(阶段1 写入副本 -> 阶段2 统一检查)
    rcl_interfaces::msg::SetParametersResult param_cb(
        const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult bad;
        bad.successful = false;

        Params tmp = p_;
        // 阶段1:写入副本(仅类型检查)
        for (const auto& par : params)
        {
            const std::string& nm = par.get_name();
            if (par.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                bad.reason = nm + ": must be double";
                return bad;
            }
            double v = par.as_double();
            if (nm == "speed") tmp.speed = v;
            else if (nm == "pwm_speed_ratio") tmp.speed_ratio = v;
            else if (nm == "speed_quad") tmp.speed_quad = v;
            else if (nm == "decel") tmp.decel = v;
            else if (nm == "min_dist") tmp.min_dist = v;
            else if (nm == "max_dist") tmp.max_dist = v;
            else if (nm == "back_dist") tmp.back_dist = v;
            else if (nm == "min_cone_deg") tmp.min_cone_deg = v;
            else if (nm == "max_cone_deg") tmp.max_cone_deg = v;
            else if (nm == "min_cone_w") tmp.min_cone_w = v;
            else if (nm == "max_cone_w") tmp.max_cone_w = v;
            else if (nm == "cone_radius") tmp.cone_radius = v;
            else if (nm == "vehicle_radius") tmp.vehicle_radius = v;
            else if (nm == "safety_margin") tmp.safety_margin = v;
            else if (nm == "w_cone_safe") tmp.w_cone_safe = v;
            else if (nm == "cone_brake") tmp.cone_brake = v;
            else if (nm == "cone_creep") tmp.cone_creep = v;
            else if (nm == "lookahead") tmp.lookahead = v;
            else if (nm == "safe_stop") tmp.safe_stop = v;
            else if (nm == "safe_turn") tmp.safe_turn = v;
            else if (nm == "pair_y_gap") tmp.pair_y_gap = v;
            else if (nm == "pair_min_w") tmp.pair_min_w = v;
            else if (nm == "pair_max_w") tmp.pair_max_w = v;
            else if (nm == "max_slew") tmp.max_slew = v;
            else if (nm == "dwa_step") tmp.dwa_step = v;
            else if (nm == "wheelbase") tmp.wheelbase = v;
            else if (nm == "w_align") tmp.w_align = v;
            else if (nm == "w_safe") tmp.w_safe = v;
            else if (nm == "w_smooth") tmp.w_smooth = v;
            else if (nm == "w_heading") tmp.w_heading = v;
            else if (nm == "steer_commit_deg") tmp.steer_commit_deg = v;
            else if (nm == "dir") tmp.dir = v;
            else if (nm == "front_deg") tmp.front_deg = v;
            else if (nm == "lidar_offset") tmp.lidar_offset = v;
        }
        // 阶段2:统一范围 + 关系检查(与参数处理顺序无关)
        std::string why;
        if (!ranges_ok(tmp, why))
        {
            bad.reason = why;
            return bad;
        }
        if (!relations_ok(tmp, why))
        {
            bad.reason = why;
            return bad;
        }
        p_ = tmp;
        rcl_interfaces::msg::SetParametersResult ok;
        ok.successful = true;
        return ok;
    }

    void callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
    {
        // ============ 1. 扫描(统一车轴系) ============
        using Pt = std::array<double, 2>;
        std::array<Pt, MAX_CONES> left_pts, right_pts;   // std::array 才能 std::sort
        double obs_x[MAX_OBS], obs_y[MAX_OBS], obs_r[MAX_OBS];   // 车轴系
        int n_l = 0, n_r = 0, n_c = 0, n_cone = 0, n_obs = 0;
        speed_deduct_extra_ = false;
        double dv = p_.speed - 1500.0;
        double est_v = 0.0;
        if (dv > 0.0)
        {
            est_v = dv * p_.speed_ratio + dv * dv * p_.speed_quad;
            if (!std::isfinite(est_v) || est_v < 0.0) est_v = 0.0;
            if (est_v > 5.0) est_v = 5.0;
        }
        double lookahead_eff = p_.lookahead + est_v * 1.2;
        double front_cos = std::cos(p_.front_deg * PI / 180.0);
        double nearest_front = 100.0, nearest_l = 100.0, nearest_r = 100.0;
        double nearest_l_x = 100.0, nearest_r_x = 100.0;
        double nearest_cone_front = 100.0;
        size_t n = laser->ranges.size();

        for (size_t i = 0; i < n; i++)
        {
            double r = laser->ranges[i];
            if (!std::isfinite(r) || r <= 0.01) continue;

            double th0 = laser->angle_min + i * laser->angle_increment;
            double x0 = r * std::sin(th0);
            double y0 = r * std::cos(th0) - p_.lidar_offset;   // 车轴系

            if (r >= 0.15 && r <= p_.max_dist && y0 > -p_.back_dist && (i % 2 == 0) && n_obs < MAX_RAW_OBS)
            {
                obs_x[n_obs] = x0; obs_y[n_obs] = y0; obs_r[n_obs] = 0.0; n_obs++;
            }

            if (y0 > front_cos * r && r < p_.max_dist && y0 < nearest_front) nearest_front = y0;

            if (y0 > -0.1)
            {
                if (x0 > 0) { if (r < nearest_l) nearest_l = r; }
                else        { if (r < nearest_r) nearest_r = r; }
                if (y0 <= 0.8)
                {
                    double ax = std::fabs(x0);
                    if (x0 > 0) { if (ax < nearest_l_x) nearest_l_x = ax; }
                    else        { if (ax < nearest_r_x) nearest_r_x = ax; }
                }
            }

            if (r <= p_.min_dist || r > p_.max_dist) continue;
            if (y0 < -p_.back_dist) continue;

            size_t end = i;
            while (end + 1 < n)
            {
                double r1 = laser->ranges[end], r2 = laser->ranges[end + 1];
                if (!std::isfinite(r1) || !std::isfinite(r2)) break;
                if (r1 <= 0.01 || r2 <= 0.01) break;
                if (r2 <= p_.min_dist || r2 > p_.max_dist) break;
                double gap = 0.15 + 0.03 * r1;
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
            double cw = (double)(end - i) * laser->angle_increment * rm;

            if (ang_w >= p_.min_cone_deg && ang_w <= p_.max_cone_deg &&
                cw >= p_.min_cone_w && cw <= p_.max_cone_w)
            {
                double thm = laser->angle_min + (i + end) * 0.5 * laser->angle_increment;
                double x = rm * std::sin(thm);
                double y = rm * std::cos(thm) - p_.lidar_offset;
                if (y >= -p_.back_dist)
                {
                    if (std::fabs(x) < 0.15)
                    {
                        n_c++;
                    }
                    else if (x > 0 && n_l < MAX_CONES) { left_pts[n_l][0] = x; left_pts[n_l][1] = y; n_l++; }
                    else if (x <= 0 && n_r < MAX_CONES) { right_pts[n_r][0] = x; right_pts[n_r][1] = y; n_r++; }

                    if (n_cone < MAX_CONES) n_cone++;
                    if (n_obs < MAX_OBS)
                    {
                        obs_x[n_obs] = x; obs_y[n_obs] = y;
                        obs_r[n_obs] = p_.cone_radius;
                        n_obs++;
                    }

                    if (y > 0.0 && std::fabs(x) < 0.577 * y)
                    {
                        if (rm < nearest_cone_front) nearest_cone_front = rm;
                    }
                }
            }
            i = end;
        }

        // ============ 2. 急停距离 ============
        double stop_dist = est_v * est_v / (2.0 * p_.decel) + 0.15;
        if (stop_dist < p_.safe_stop) stop_dist = p_.safe_stop;

        // ============ 3. 无锥桶巡航 / 障碍模式 ============
        bool obstacle_mode = false;
        if (n_l + n_r + n_c == 0)
        {
            if (nearest_front < lookahead_eff + 0.3)
            {
                obstacle_mode = true;
                last_cone_t_ = std::chrono::steady_clock::now();
            }
            else
            {
                twist.angular.z = 90.0 + p_.dir * (last_z_ - 90.0);
                double spd = p_.speed - 15;
                double floor_s2 = (p_.speed > 1500.0) ? 1515.0 : 1500.0;
                if (spd < floor_s2) spd = floor_s2;
                twist.linear.x = spd;
                double lost_s = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - last_cone_t_).count();
                if (lost_s > 4.0)
                {
                    twist.linear.x = 1500;
                    twist.angular.z = 90;
                    last_z_ = 90.0;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "锥桶丢失超 4 秒,停车等待");
                }
                pub_->publish(twist);
                return;
            }
        }
        else if (nearest_cone_front < 1.0)
        {
            obstacle_mode = true;
        }
        last_cone_t_ = std::chrono::steady_clock::now();

        // ============ 4. 中线路径生成 + 前视目标 ============
        int i;
        double target_x = 0.0;
        bool have_target = obstacle_mode;
        if (obstacle_mode) speed_deduct_extra_ = true;

        if (!obstacle_mode)
        {
            std::sort(left_pts.begin(), left_pts.begin() + n_l,
                      [](const Pt& a, const Pt& b) { return a[1] < b[1]; });
            std::sort(right_pts.begin(), right_pts.begin() + n_r,
                      [](const Pt& a, const Pt& b) { return a[1] < b[1]; });

            if (n_l > 0 && n_r > 0)
            {
                double lsum = 0.0, rsum = 0.0;
                int nl2 = 0, nr2 = 0;
                for (i = 0; i < n_l; i++)
                    if (left_pts[i][1] >= 0.3 && left_pts[i][1] <= lookahead_eff + 0.5) { lsum += std::fabs(left_pts[i][0]); nl2++; }
                for (i = 0; i < n_r; i++)
                    if (right_pts[i][1] >= 0.3 && right_pts[i][1] <= lookahead_eff + 0.5) { rsum += std::fabs(right_pts[i][0]); nr2++; }
                if (nl2 > 0 && nr2 > 0)
                {
                    double hw_new = (lsum / nl2 + rsum / nr2) / 2.0;
                    half_width_ = 0.8 * half_width_ + 0.2 * hw_new;
                    if (half_width_ < 0.3) half_width_ = 0.3;
                }
            }

            bool used_r[MAX_CONES] = {false};
            int pair_cnt = 0, valid_cnt = 0;
            double best_d2 = 1e9;

            if (n_l > 0 && n_r > 0)
            {
                double base_dy_thr = std::max(p_.pair_y_gap, 0.5 + 0.2 * est_v);
                for (i = 0; i < n_l; i++)
                {
                    double best_dy = base_dy_thr;
                    int best_ri = -1;
                    for (int a = 0; a < n_r; a++)
                    {
                        if (used_r[a]) continue;
                        double w = left_pts[i][0] - right_pts[a][0];
                        if (w < p_.pair_min_w || w > p_.pair_max_w) continue;
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
                if (pair_cnt == 0)
                {
                    bool used2[MAX_CONES] = {false};
                    for (i = 0; i < n_l; i++)
                    {
                        double best_d = 1e9;
                        int best_ri = -1;
                        for (int a = 0; a < n_r; a++)
                        {
                            if (used2[a]) continue;
                            double w = left_pts[i][0] - right_pts[a][0];
                            if (w < p_.pair_min_w || w > p_.pair_max_w) continue;
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
                target_x = 0.0;
                have_target = true;
                speed_deduct_extra_ = true;
            }
            else if (n_c > 0)
            {
                target_x = 0.0;
                have_target = true;
                speed_deduct_extra_ = true;
            }
        }
        else
        {
            target_x = (nearest_l > nearest_r) ? 0.5 : -0.5;
        }
        if (!have_target)
        {
            twist.linear.x = 1500; twist.angular.z = 90;
            last_z_ = 90.0;
            pub_->publish(twist);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "无有效目标,停车等待");
            return;
        }

        // ============ 5. DWA-lite(物理系) ============
        double wb = (p_.wheelbase >= 0.05) ? p_.wheelbase : 0.305;
        double step = (p_.dwa_step >= 0.5) ? p_.dwa_step : 2.0;

        struct TrajResult
        {
            bool collide;
            double min_cone_d2;
        };
        // P1-2: 碰撞检测长度 = lookahead_eff 全程(不再被 1.5R 截短)
        auto evaluate_traj = [&](double zc) -> TrajResult
        {
            TrajResult res;
            res.collide = false;
            res.min_cone_d2 = std::numeric_limits<double>::infinity();
            double delta = (zc - 90.0) * PI / 180.0;
            double rr = 0.0;
            double plen = lookahead_eff;
            if (std::fabs(delta) > 0.01)
            {
                rr = std::fabs(wb / std::tan(delta));
            }
            if (n_obs == 0 || plen <= 0.01) return res;
            int samples = std::max(10, (int)std::ceil(plen / 0.08));
            for (int s = 0; s <= samples; s++)
            {
                double seg = plen * (double)s / (double)samples;
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
                    double d2 = dx * dx + dy * dy;
                    if (obs_r[o] > 0.001 && d2 < res.min_cone_d2) res.min_cone_d2 = d2;
                    double safe_r = p_.vehicle_radius + obs_r[o] + p_.safety_margin;
                    if (d2 < safe_r * safe_r)
                    {
                        res.collide = true;
                        return res;
                    }
                }
            }
            return res;
        };

        double best_z = 90.0, best_cost = std::numeric_limits<double>::infinity();
        bool straight_collide = false;
        bool any_safe = false;
        double safe_z[MAX_SAFE_CAND];
        int n_safe = 0;

        // P1-1: 候选评估(抽成 lambda,保证 90° 必选)
        auto evaluate_candidate = [&](double zc)
        {
            double delta = (zc - 90.0) * PI / 180.0;
            double proj = 0.0, rr = 0.0;
            double plen = lookahead_eff;
            if (std::fabs(delta) > 0.01)
            {
                rr = std::fabs(wb / std::tan(delta));
                proj = std::copysign(rr * (1.0 - std::cos(plen / rr)), delta);
            }

            TrajResult tr = evaluate_traj(zc);
            if (std::fabs(zc - 90.0) < step / 2.0 + 0.001 && tr.collide) straight_collide = true;
            if (tr.collide) return;
            if (n_safe < MAX_SAFE_CAND) safe_z[n_safe++] = zc;
            any_safe = true;

            double err_after = target_x - proj;
            double align_w = obstacle_mode ? 3.0 : p_.w_align;
            double cost = align_w * std::fabs(err_after);

            double clear = ((zc > 90.0) ? nearest_l : nearest_r) - p_.vehicle_radius;
            if (clear < 1.0) cost += p_.w_safe * est_v / std::max(clear, 0.1);

            if (tr.min_cone_d2 < std::numeric_limits<double>::infinity() && tr.min_cone_d2 > 0.0)
            {
                double min_clearance = std::sqrt(tr.min_cone_d2) - p_.vehicle_radius - p_.cone_radius;
                cost += p_.w_cone_safe / std::max(min_clearance, 0.05);
            }

            double heading_w = obstacle_mode ? 0.02 : p_.w_heading;
            cost += heading_w * std::fabs(delta);
            cost += p_.w_smooth * std::fabs(zc - last_z_);

            if (!obstacle_mode && std::fabs(target_x) > 0.12)
            {
                double commit = std::min(p_.steer_commit_deg, std::fabs(target_x) * 30.0);
                double dz_90 = std::fabs(zc - 90.0);
                if (dz_90 < commit)
                    cost += (commit - dz_90) * 0.3;
            }

            if (nearest_cone_front < 1.2 && std::fabs(zc - 90.0) < 20.0)
                cost += 15.0 / std::max(nearest_cone_front, 0.2);

            if (cost < best_cost) { best_cost = cost; best_z = zc; }
        };

        bool has90 = false;
        for (double zc = 45.0; zc <= 135.0001; zc += step)
        {
            if (std::fabs(zc - 90.0) < 1e-9) has90 = true;
            evaluate_candidate(zc);
        }
        if (!has90) evaluate_candidate(90.0);   // P1-1: 90° 必选

        if (!any_safe)
        {
            twist.linear.x = 1500.0;
            twist.angular.z = 90.0;
            last_z_ = 90.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "DWA所有候选轨迹均碰撞,停车");
            pub_->publish(twist);
            return;
        }

        twist.angular.z = best_z;

        // ============ 6. 侧向安全 + 转向限速(物理系;P1-4 不再提前更新 last_z_) ============
        bool safe_limit = false;
        if (twist.angular.z > 90.0 && nearest_l_x < p_.safe_turn) { twist.angular.z = 90.0; safe_limit = true; }
        if (twist.angular.z < 90.0 && nearest_r_x < p_.safe_turn) { twist.angular.z = 90.0; safe_limit = true; }

        if (!safe_limit)
        {
            double slew = p_.max_slew + std::fabs(twist.angular.z - 90.0) * 0.3;
            double dz = twist.angular.z - last_z_;
            if (dz > slew) twist.angular.z = last_z_ + slew;
            if (dz < -slew) twist.angular.z = last_z_ - slew;
        }

        // ============ 7. 速度控制(可能修改舵角) ============
        double speed = p_.speed;
        double steer_deg = std::fabs(twist.angular.z - 90.0);
        bool side_blocked = (nearest_l_x < 0.35 && nearest_r_x < 0.35);

        if (nearest_front < 0.15)
        {
            speed = 1500.0;
            twist.angular.z = 90.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "正前方 %.2fm 死路,停车", nearest_front);
        }
        else if (nearest_front < stop_dist)
        {
            if (straight_collide && steer_deg < 6.0 && !side_blocked)
            {
                double target_z = (nearest_l > nearest_r) ? 120.0 : 60.0;
                double dz2 = target_z - last_z_;
                double slew2 = p_.max_slew + std::fabs(target_z - 90.0) * 0.3;
                if (dz2 > slew2) target_z = last_z_ + slew2;
                if (dz2 < -slew2) target_z = last_z_ - slew2;
                twist.angular.z = target_z;
                speed = 1515.0;
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
                speed = 1515.0;
            }
        }
        else
        {
            if (speed_deduct_extra_) speed -= 10;
            if (steer_deg > 20) speed -= 5;
            if (steer_deg > 35) speed -= 10;
            if (nearest_l_x < 0.5 || nearest_r_x < 0.5) speed -= 8;

            if (nearest_cone_front < p_.cone_creep)
            {
                speed = std::min(speed, 1515.0);
            }
            else if (nearest_cone_front < p_.cone_brake)
            {
                speed -= 8;
            }

            double floor_s = (p_.speed > 1500.0) ? 1512.0 : 1500.0;
            if (speed < floor_s) speed = floor_s;
        }
        twist.linear.x = speed;

        // ============ 8. 最终执行角唯一出口 ============
        bool z_unsafe = evaluate_traj(twist.angular.z).collide;
        if (z_unsafe && n_safe > 0)
        {
            double best_dz = 1e9, repl = twist.angular.z;
            for (int k = 0; k < n_safe; k++)
            {
                double dd = std::fabs(safe_z[k] - twist.angular.z);
                if (dd < best_dz) { best_dz = dd; repl = safe_z[k]; }
            }
            twist.angular.z = repl;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "最终舵角复检不安全,回退到 %.1f°", repl);
        }
        if (evaluate_traj(twist.angular.z).collide)
        {
            twist.linear.x = 1500.0;
            twist.angular.z = 90.0;
            last_z_ = 90.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "复检后仍无安全角,停车");
            pub_->publish(twist);
            return;
        }
        last_z_ = twist.angular.z;   // P1-4: 唯一更新点(真实执行角)

        // dir 仅输出映射
        twist.angular.z = 90.0 + p_.dir * (twist.angular.z - 90.0);

        pub_->publish(twist);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "左:%d 右:%d 中:%d 桶:%d 目标x:%.2f 桶前:%.2f 前:%.2f obs:%d 舵角:%.1f 速度:%.0f%s",
                             n_l, n_r, n_c, n_cone, target_x, nearest_cone_front, nearest_front,
                             n_obs, twist.angular.z, twist.linear.x, obstacle_mode ? " [绕障]" : "");
    }

public:
    LaserControl() : Node("PID_test")
    {
        this->declare_parameter("speed", 1520.0);
        this->declare_parameter("pwm_speed_ratio", 0.025);
        this->declare_parameter("speed_quad", 0.0);
        this->declare_parameter("decel", 2.0);
        this->declare_parameter("min_dist", 0.5);
        this->declare_parameter("max_dist", 3.5);
        this->declare_parameter("back_dist", 0.2);
        this->declare_parameter("min_cone_deg", 2.0);
        this->declare_parameter("max_cone_deg", 20.0);
        this->declare_parameter("min_cone_w", 0.05);
        this->declare_parameter("max_cone_w", 0.30);
        this->declare_parameter("cone_radius", 0.06);
        this->declare_parameter("vehicle_radius", 0.22);
        this->declare_parameter("safety_margin", 0.05);
        this->declare_parameter("w_cone_safe", 1.5);
        this->declare_parameter("cone_brake", 0.8);
        this->declare_parameter("cone_creep", 0.4);
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
        this->declare_parameter("w_smooth", 0.05);
        this->declare_parameter("w_heading", 0.08);
        this->declare_parameter("steer_commit_deg", 16.0);
        this->declare_parameter("dir", 1.0);
        this->declare_parameter("front_deg", 45.0);
        this->declare_parameter("lidar_offset", 0.15);

        p_.speed = this->get_parameter("speed").as_double();
        p_.speed_ratio = this->get_parameter("pwm_speed_ratio").as_double();
        p_.speed_quad = this->get_parameter("speed_quad").as_double();
        p_.decel = this->get_parameter("decel").as_double();
        p_.min_dist = this->get_parameter("min_dist").as_double();
        p_.max_dist = this->get_parameter("max_dist").as_double();
        p_.back_dist = this->get_parameter("back_dist").as_double();
        p_.min_cone_deg = this->get_parameter("min_cone_deg").as_double();
        p_.max_cone_deg = this->get_parameter("max_cone_deg").as_double();
        p_.min_cone_w = this->get_parameter("min_cone_w").as_double();
        p_.max_cone_w = this->get_parameter("max_cone_w").as_double();
        p_.cone_radius = this->get_parameter("cone_radius").as_double();
        p_.vehicle_radius = this->get_parameter("vehicle_radius").as_double();
        p_.safety_margin = this->get_parameter("safety_margin").as_double();
        p_.w_cone_safe = this->get_parameter("w_cone_safe").as_double();
        p_.cone_brake = this->get_parameter("cone_brake").as_double();
        p_.cone_creep = this->get_parameter("cone_creep").as_double();
        p_.lookahead = this->get_parameter("lookahead").as_double();
        p_.safe_stop = this->get_parameter("safe_stop").as_double();
        p_.safe_turn = this->get_parameter("safe_turn").as_double();
        p_.pair_y_gap = this->get_parameter("pair_y_gap").as_double();
        p_.pair_min_w = this->get_parameter("pair_min_w").as_double();
        p_.pair_max_w = this->get_parameter("pair_max_w").as_double();
        p_.max_slew = this->get_parameter("max_slew").as_double();
        p_.dwa_step = this->get_parameter("dwa_step").as_double();
        p_.wheelbase = this->get_parameter("wheelbase").as_double();
        p_.w_align = this->get_parameter("w_align").as_double();
        p_.w_safe = this->get_parameter("w_safe").as_double();
        p_.w_smooth = this->get_parameter("w_smooth").as_double();
        p_.w_heading = this->get_parameter("w_heading").as_double();
        p_.steer_commit_deg = this->get_parameter("steer_commit_deg").as_double();
        p_.dir = this->get_parameter("dir").as_double();
        p_.front_deg = this->get_parameter("front_deg").as_double();
        p_.lidar_offset = this->get_parameter("lidar_offset").as_double();
        // P1-5: 启动参数完整验证(非法整体回退默认值)
        sanitize_params();

        twist.linear.x = 1500;
        twist.angular.z = 90;
        last_z_ = 90.0;
        last_cone_t_ = std::chrono::steady_clock::now();

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", 5);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 5, std::bind(&LaserControl::callback, this, std::placeholders::_1));
        this->add_on_set_parameters_callback(
            std::bind(&LaserControl::param_cb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "PID_test v6.6.3 Cone-DWA(可编译版) 前视=%.1fm 速度=%.0f 碰撞=%.2f+%.2f+%.2f",
                    p_.lookahead, p_.speed, p_.vehicle_radius, p_.cone_radius, p_.safety_margin);
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
