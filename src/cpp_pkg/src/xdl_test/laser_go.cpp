#include "rclcpp/rclcpp.hpp"                  // ROS 2 核心头文件
#include "sensor_msgs/msg/laser_scan.hpp"     // ROS 2 激光雷达消息
#include "geometry_msgs/msg/twist.hpp"        // ROS 2 速度控制消息
#include "nav_msgs/msg/odometry.hpp"          // ROS 2 里程计消息（黄线圆弧航向积分）
#include <cmath>                              // C++ 数学库
#include <algorithm>                          // std::sort（按 y 配对排序）
//版本16（回归求和误差+方向自适应特判：转向权限恢复版本10水平，弯道方向修正）
#define SPEED           1525   // 直道基础线速度
#define SPEED_SLOW      1522   // 弯道/特判减速（中速，避免死区）
#define LOOKAHEAD_Y     2.5    // 前瞻距离
#define ARC_STEER       106    // 黄线左半圆弧(R=2m)固定舵角
#define LOST_STRAIGHT   5      // 无桶后直行帧数
#define POST_ARC_STRAIGHT 15   // 黄线弧后直行帧数
#define TURN_LEFT       140    // 只看到左(红)桶：左转
#define TURN_RIGHT      40     // 只看到右(蓝)桶：右转
#define STABLE_RECOVER  3      // 走廊找回所需连续帧数
#define GAIN            55     // 循迹增益（误差已归一化，55倍保证弯道力度不饱和）
#define RECOVERY_FRAMES 8      // 恢复期时长

// 1. 直角坐标系结构体
struct PointRectangular {
    double x = 0.0;  // x 坐标（>0左/<0右）
    double y = 0.0;  // y 坐标（前方）
};
// 2. 极坐标转直角坐标
void cal_point(double theta, double range, double &tmp_x, double &tmp_y) {
    tmp_x = range * std::sin(theta);
    tmp_y = range * std::cos(theta);
}
// 3. 角度差归一化到 [-pi, pi]
double wrapPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
// 4. 节点类
class LaserGoNode : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double smoothed_error_ = 0.0;  // 误差低通滤波
    double last_out_z_ = 90.0;     // 上一帧舵角（转向限幅）
    int recover_frames_ = 0;       // 恢复期计数
    int corridor_frames_ = 0;      // 强走廊连续帧数（黄线弧退出用）
    int one_side_frames_ = 0;      // 单侧桶持续帧数（连续2帧才满舵，防闪烁）
    bool lost_was_straight_ = true; // 丢桶首帧是否直行（黄线弧进入条件）
    // 黄线弧状态机：0=直行接近，1=左弧+航向积分，2=弧后直行，3=扫描
    int phase_ = 0;
    int phase_frames_ = 0;
    double arc_start_yaw_ = 0.0;
    double yaw_now_ = 0.0;
    bool have_odom_ = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double w = msg->pose.pose.orientation.w;
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        yaw_now_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        have_odom_ = true;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser) {
        int i = 0, red_cone_Index = 0, blue_cone_Index = 0;
        int ranges_size = laser->ranges.size();
        double error = 0.0, theta = 0.0, tmp_x = 0.0, tmp_y = 0.0, angle = 0.0;
        geometry_msgs::msg::Twist twist;

        PointRectangular red_p[30] = {{0.0, 0.0}};
        PointRectangular blue_p[30] = {{0.0, 0.0}};

        // -------------------------- 第一步：锥桶检测（跳变+簇验证，防漏检/防噪声） --------------------------
        for (i = 1; i < ranges_size; i++) {
            // 跳变阈值1.2m：真实锥桶跳变1.5~1.8m不漏检；<2.0m原值防噪声靠簇验证
            if ((laser->ranges[i-1] - laser->ranges[i] >= 1.2) &&
                (laser->ranges[i] < LOOKAHEAD_Y) &&
                (std::isfinite(laser->ranges[i]))) {
                // 簇验证：i,i+1,i+2 连续三点距离相近（锥桶体），单点噪声直接丢弃
                bool cluster = false;
                if (i + 2 < ranges_size) {
                    double r0 = laser->ranges[i];
                    double r1 = laser->ranges[i+1];
                    double r2 = laser->ranges[i+2];
                    if (std::isfinite(r1) && std::isfinite(r2) &&
                        std::fabs(r1 - r0) < 0.5 && std::fabs(r2 - r0) < 0.5) {
                        cluster = true;
                    }
                }
                if (!cluster) continue;
                theta = i * laser->angle_increment + laser->angle_min;
                cal_point(theta, laser->ranges[i], tmp_x, tmp_y);
                if ((tmp_x <= 1.5 && tmp_x >= -1.5) && (tmp_y <= LOOKAHEAD_Y && tmp_y >= -0.1)) {
                    if (tmp_x > 0 && red_cone_Index < 30) {
                        red_p[red_cone_Index].x = tmp_x;
                        red_p[red_cone_Index].y = tmp_y;
                        red_cone_Index++;
                    } else if (tmp_x <= 0 && blue_cone_Index < 30) {
                        blue_p[blue_cone_Index].x = tmp_x;
                        blue_p[blue_cone_Index].y = tmp_y;
                        blue_cone_Index++;
                    }
                }
            }
        }

        // -------------------------- 黄线左弧进行中（先于一切） --------------------------
        if (phase_ == 1) {
            phase_frames_++;
            double yaw_diff = std::fabs(wrapPi(yaw_now_ - arc_start_yaw_));
            bool strong_corridor = (red_cone_Index >= 3 && blue_cone_Index >= 3);
            double min_r = 99.0;
            for (int k = 0; k < red_cone_Index; k++) min_r = std::min(min_r, red_p[k].y);
            for (int k = 0; k < blue_cone_Index; k++) min_r = std::min(min_r, blue_p[k].y);
            bool near_cone = (min_r < 0.7);
            if (strong_corridor) corridor_frames_++;
            else corridor_frames_ = 0;
            bool stuck = (phase_frames_ > 30 && yaw_diff < 0.05);
            if (corridor_frames_ >= 3 || near_cone || (have_odom_ && yaw_diff >= 2.8) ||
                phase_frames_ > 95 || stuck) {
                phase_ = 2; phase_frames_ = 0; corridor_frames_ = 0;
            }
            twist.linear.x = SPEED;
            twist.angular.z = ARC_STEER;
            last_out_z_ = twist.angular.z;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "ARC f:%d yawdiff:%.2f corr:%d z:%.1f",
                phase_frames_, yaw_diff, strong_corridor ? 1 : 0, twist.angular.z);
            cmd_vel_pub_->publish(twist);
            return;
        }

        // -------------------------- 完全无桶：黄线半圆弧状态机 --------------------------
        if (red_cone_Index == 0 && blue_cone_Index == 0) {
            smoothed_error_ = 0.0;
            recover_frames_ = 0;
            // 首帧记录丢桶前是否直行（防遮挡/漏检误触发黄线弧）
            if (phase_frames_ == 0) lost_was_straight_ = (std::fabs(last_out_z_ - 90.0) < 15.0);
            phase_frames_++;
            twist.linear.x = SPEED;
            if (phase_ == 0) {
                if (phase_frames_ > LOST_STRAIGHT) {
                    if (lost_was_straight_) {
                        phase_ = 1; phase_frames_ = 0; arc_start_yaw_ = yaw_now_;  // 直行中丢桶→真黄线弧
                    } else {
                        phase_ = 3; phase_frames_ = 0;                            // 转弯中丢桶→直接扫描
                    }
                }
                twist.angular.z = 90;
            } else if (phase_ == 2) {
                if (phase_frames_ > POST_ARC_STRAIGHT) { phase_ = 3; phase_frames_ = 0; }
                twist.angular.z = 90;
            } else {
                int cyc = phase_frames_ % 40;
                if (cyc < 13) twist.angular.z = 90;
                else if (cyc < 26) twist.angular.z = 118;     // 扫左
                else twist.angular.z = 62;                    // 扫右
            }
            if (twist.angular.z - last_out_z_ > 40) twist.angular.z = last_out_z_ + 40;
            if (last_out_z_ - twist.angular.z > 40) twist.angular.z = last_out_z_ - 40;
            last_out_z_ = twist.angular.z;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "LOST r:%d b:%d ph:%d f:%d yawdiff:%.2f odom:%d z:%.1f",
                red_cone_Index, blue_cone_Index, phase_, phase_frames_,
                std::fabs(wrapPi(yaw_now_ - arc_start_yaw_)), have_odom_ ? 1 : 0, twist.angular.z);
            cmd_vel_pub_->publish(twist);
            return;
        }

        // -------------------------- 真正单侧有桶：向可见侧满舵（连续2帧才执行，防闪烁） --------------------------
        if (red_cone_Index == 0 || blue_cone_Index == 0) {
            smoothed_error_ = 0.0;
            recover_frames_ = 0;
            bool only_red = (blue_cone_Index == 0);   // 只剩左(红)桶→弯道向左
            one_side_frames_++;
            if (one_side_frames_ > 1) {
                twist.linear.x = SPEED_SLOW;              // 转弯减速
                twist.angular.z = only_red ? TURN_LEFT : TURN_RIGHT;
            } else {
                twist.linear.x = SPEED;
                twist.angular.z = 90;                     // 首帧直行确认
            }
            if (twist.angular.z - last_out_z_ > 40) twist.angular.z = last_out_z_ + 40;
            if (last_out_z_ - twist.angular.z > 40) twist.angular.z = last_out_z_ - 40;
            last_out_z_ = twist.angular.z;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "LOST r:%d b:%d ph:%d f:%d yawdiff:%.2f odom:%d z:%.1f",
                red_cone_Index, blue_cone_Index, phase_, phase_frames_,
                std::fabs(wrapPi(yaw_now_ - arc_start_yaw_)), have_odom_ ? 1 : 0, twist.angular.z);
            cmd_vel_pub_->publish(twist);
            return;
        }
        one_side_frames_ = 0;

        // -------------------------- 第二步：正常循迹（左右红绿锥桶配对匹配，求和误差） --------------------------
        // 取红蓝桶数量一致
        if (red_cone_Index > blue_cone_Index) red_cone_Index = blue_cone_Index;
        else blue_cone_Index = red_cone_Index;
        // 按 y 从远到近排序，同层配对
        std::sort(red_p, red_p + red_cone_Index, [](const PointRectangular& a, const PointRectangular& b) {
            return a.y > b.y;
        });
        std::sort(blue_p, blue_p + blue_cone_Index, [](const PointRectangular& a, const PointRectangular& b) {
            return a.y > b.y;
        });
        // 求和误差 + 距离加权前瞻（远处配对权重更大=提前感知弯道）
        // 权重：近处0.7 → 最远1.3，直道上对称抵消无影响，弯道上远桶先偏→提前转向
        for (i = 0; i <= red_cone_Index - 1; i++) {
            double y_mid = (red_p[i].y + blue_p[i].y) / 2.0;   // 该配对的平均距离
            double w = 0.7 + 0.6 * (y_mid / LOOKAHEAD_Y);      // 0.7~1.3
            error += (blue_p[i].x + red_p[i].x) * w;
        }
        // ★ 归一化：除以桶对数，控制量与检测桶数量无关（防桶多饱和/桶少无力）
        if (red_cone_Index > 0) error /= red_cone_Index;
        double alpha = std::abs(error) > 1.2 ? 0.6 : 0.5;
        smoothed_error_ = smoothed_error_ * (1 - alpha) + error * alpha;
        error = smoothed_error_;
        angle = error * GAIN;

        phase_ = 0; phase_frames_ = 0; corridor_frames_ = 0;
        recover_frames_++;
        bool in_recovery = (recover_frames_ <= RECOVERY_FRAMES);

        twist.angular.z = 90 + angle;
        if (twist.angular.z > 145) twist.angular.z = 145;
        if (twist.angular.z < 30)  twist.angular.z = 30;
        if (twist.angular.z - last_out_z_ > 60) twist.angular.z = last_out_z_ + 60;
        if (last_out_z_ - twist.angular.z > 60) twist.angular.z = last_out_z_ - 60;
        last_out_z_ = twist.angular.z;

        double abs_err = std::abs(error);
        if (abs_err > 1.5) {
            twist.linear.x = SPEED_SLOW;                  // 急弯：减速
        } else if (in_recovery) {
            twist.linear.x = SPEED_SLOW;                  // 恢复期：中速
        } else {
            twist.linear.x = SPEED;
        }

        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.linear.y  = 0.0;
        twist.linear.z  = 0.0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "TRACK r:%d b:%d raw:%.2f sm:%.2f z:%.1f",
            red_cone_Index, blue_cone_Index, error, smoothed_error_, twist.angular.z);
        cmd_vel_pub_->publish(twist);
    }

public:
    LaserGoNode() : Node("laser_go") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", 5);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 5,
            std::bind(&LaserGoNode::laser_callback, this, std::placeholders::_1)
        );
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_combined", 10,
            std::bind(&LaserGoNode::odom_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "LaserGoNode has been initialized!");
    }
};
// 5. 主函数
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserGoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
