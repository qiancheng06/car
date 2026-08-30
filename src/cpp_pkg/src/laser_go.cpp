#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

#define SPEED 1525

struct PointPolar {
    double range = 0.0;
    double theta = 0.0;
};

struct PointRectangular {
    double x = 0.0;
    double y = 0.0;
};

struct VectorQuantity {
    double a = 0.0;
    double b = 0.0;
};

void cal_point(double theta, double range, double &tmp_x, double &tmp_y) {
    tmp_x = range * std::sin(theta);
    tmp_y = range * std::cos(theta);
}

class LaserGoNode : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    // 状态：0=有锥桶(循迹), 1=无锥桶(旋转)
    int no_cone_state_ = 0;
    bool turning_ = false;
    double turn_start_time_ = 0.0;

    // [参数] 旋转公式可调系数
    double turn_arc_ = 6.283;      // 转弯弧长（转一整圈=2π）
    double speed_ratio_ = 0.025;   // PWM转m/s系数

    // ★新增:可调参数(支持 ros2 param 在线调,不用重新编译)
    double speed_ = 1525.0;        // 巡航速度PWM
    double base_angle_ = 81.75;    // 舵机基准角(保留原车手工微调值)
    double gain_ = 33.0;           // 误差(m)→舵角(°)增益
    double lane_hold_ = 0.75;      // 单侧循迹时与可见锥桶排保持的距离(m)
    double pair_y_match_ = 1.6;    // 红蓝最近锥桶纵向距离差超过此值(m)视为不可配对。直道错排约0.7m,弯道更大,1.0会误杀大量有效配对
    double pair_ahead_min_ = 1.0;  // 配对优先取 y>=此值 的前方锥桶,避免瞄准车辆正侧方导致原地绕圈
    double last_log_time_ = 0.0;       // 检测统计日志限流
    double last_pair_log_time_ = 0.0;  // 配对/单侧决策日志限流(★独立时间戳,原来与检测日志共用导致配对日志永远打不出来)
    bool debug_dump_ = false;          // 锥桶坐标逐帧打印开关(默认关)

    // 正前方障碍处置参数
    double ahead_dist_ = 0.5;      // 触发距离(m)
    double ahead_width_ = 0.30;    // 触发横向半宽(m)
    double avoid_angle_ = 35.0;    // 绕行转向角(度)
    double avoid_speed_ = 1518.0;  // 绕行速度PWM(高于电机死区,低速爬行)
    double avoid_hold_s_ = 1.2;    // 绕行方向保持时间(s),防逐帧抖动
    double avoid_until_ = 0.0;     // 当前绕行保持到何时
    double avoid_dir_ = 0.0;       // 当前绕行转向角

    // ★修复:单侧锥桶比例循迹(替代固定60/117避让)。
    //   原实现只看到一排锥桶就打死方向(红→60°,蓝→117°),检测断续时左右猛甩=画龙出赛道。
    //   现改为把可见锥桶排保持在 lane_hold_(0.75m)处:角向偏差×增益,±20°限幅,柔和回中。
    //   红排(x>0,右):error = red.x - lane_hold_,正→右打;
    //   蓝排(x<0,左):error = blue.x + lane_hold_,负→左打。
    double singleSideAngle(const PointRectangular& p, bool is_right_row) {
        double e = is_right_row ? (p.x - lane_hold_) : (p.x + lane_hold_);
        double a = e * gain_;
        if (a > 20.0) a = 20.0;
        if (a < -20.0) a = -20.0;
        return a;
    }

    // ★新增:正前方锥桶处置。8月30日实测:车顶住正前方0.3m的红锥桶原地推了3分钟,
    //   配对算法对"正前方障碍"无能为力(它只会居中,不会避障)。
    //   比赛规则:撞锥桶每次罚5s,任意位置停止>10s判任务失败——所以不能停死,
    //   用低速+大转向绕开。触发:任一锥桶 0<y<ahead_dist 且 |x|<ahead_width。
    //   绕行方向:锥桶偏哪边就往反方向打;几乎正中(±0.1m内)按逆时针赛道方向左转。
    //   avoid_hold_s 内保持绕行,防止逐帧抖动。
    bool aheadObstacle(const PointRectangular red_p[], int j,
                       const PointRectangular blue_p[], int k,
                       double& steer_away) {
        for (int m = 0; m < j; m++) {
            if (red_p[m].y > 0.05 && red_p[m].y < ahead_dist_ && std::abs(red_p[m].x) < ahead_width_) {
                steer_away = (red_p[m].x > 0.1) ? -avoid_angle_ : ((red_p[m].x < -0.1) ? avoid_angle_ : -avoid_angle_);
                return true;
            }
        }
        for (int m = 0; m < k; m++) {
            if (blue_p[m].y > 0.05 && blue_p[m].y < ahead_dist_ && std::abs(blue_p[m].x) < ahead_width_) {
                steer_away = (blue_p[m].x > 0.1) ? -avoid_angle_ : ((blue_p[m].x < -0.1) ? avoid_angle_ : -avoid_angle_);
                return true;
            }
        }
        return false;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser) {
        int i = 0, red_cone_Index = 0, blue_cone_Index = 0;
        int ranges_size = laser->ranges.size();
        double error = 0.0, theta = 0.0, tmp_x = 0.0, tmp_y = 0.0, angle = 0.0;
        geometry_msgs::msg::Twist twist;

        PointRectangular red_p[30] = {{0.0, 0.0}};
        PointRectangular blue_p[30] = {{0.0, 0.0}};

        // 第一步：聚类检测锥桶（断点分段 + 连续点数 + 取中间点）
        i = 0;
        while (i < ranges_size - 1) {
            // 跳过无效点
            if (!std::isfinite(laser->ranges[i]) || laser->ranges[i] <= 0.01) { i++; continue; }
            // 找一簇连续点：相邻两点距离 <= 0.2m
            int start = i;
            int end = i;
            while (end + 1 < ranges_size) {
                double r1 = laser->ranges[end];
                double r2 = laser->ranges[end + 1];
                if (!std::isfinite(r1) || !std::isfinite(r2) || r1 <= 0.01 || r2 <= 0.01) break;
                double th1 = end * laser->angle_increment + laser->angle_min;
                double th2 = (end + 1) * laser->angle_increment + laser->angle_min;
                double x1 = r1 * std::sin(th1), y1 = r1 * std::cos(th1);
                double x2 = r2 * std::sin(th2), y2 = r2 * std::cos(th2);
                double dist = std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
                if (dist > 0.20) break;   // 断点：相邻点距 > 20cm
                end++;
            }
            int count = end - start + 1;
            // 连续点数 > 6 才算锥桶
            if (count > 6) {
                int mid = (start + end) / 2;
                double rm = laser->ranges[mid];
                if (std::isfinite(rm) && rm > 0.01 && rm < 2.5) {
                    theta = mid * laser->angle_increment + laser->angle_min;
                    cal_point(theta, rm, tmp_x, tmp_y);
                    if ((tmp_x <= 1.5 && tmp_x >= -1.5) && (tmp_y <= 2.0 && tmp_y >= -0.1)) {
                        if (tmp_x > 0) {
                            red_p[red_cone_Index].x = tmp_x;
                            red_p[red_cone_Index].y = tmp_y;
                            red_cone_Index++;
                        }
                        else {
                            blue_p[blue_cone_Index].x = tmp_x;
                            blue_p[blue_cone_Index].y = tmp_y;
                            blue_cone_Index++;
                        }
                    }
                }
            }
            i = end + 1;
        }

        // 打印检测到的红蓝锥桶数量（★限流到0.5s一条,原实现每帧刷屏）
        double now_s = this->now().seconds();
        if (now_s - last_log_time_ >= 0.5) {
            RCLCPP_INFO(this->get_logger(), "检测到: %d红 %d蓝", red_cone_Index, blue_cone_Index);
            last_log_time_ = now_s;
        }

        // ===== 正前方障碍优先处置(最高优先级) =====
        double avoid_steer = 0.0;
        if (aheadObstacle(red_p, red_cone_Index, blue_p, blue_cone_Index, avoid_steer)) {
            if (now_s > avoid_until_) {
                avoid_until_ = now_s + avoid_hold_s_;
                avoid_dir_ = avoid_steer;
            }
            no_cone_state_ = 0;
            turning_ = false;
            twist.linear.x = avoid_speed_;
            twist.angular.z = base_angle_ + avoid_dir_;
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            twist.angular.x = 0.0; twist.angular.y = 0.0;
            twist.linear.y = 0.0; twist.linear.z = 0.0;
            if (now_s - last_pair_log_time_ >= 0.5) {
                RCLCPP_INFO(this->get_logger(), "正前方障碍! 绕行 dir=%.1f speed=%.0f", avoid_dir_, twist.linear.x);
                last_pair_log_time_ = now_s;
            }
            cmd_vel_pub_->publish(twist);
            return;
        }

        // ========== 状态判断：三种情况 ==========
        if (red_cone_Index == 0 && blue_cone_Index == 0) {
            // 情况1：无锥桶 → 旋转
            if (no_cone_state_ == 0) {
                no_cone_state_ = 1;
                turning_ = true;
                turn_start_time_ = this->now().seconds();
                RCLCPP_INFO(this->get_logger(), "no cone, start turning");
            }
            double speed_ms = (speed_ - 1500.0) * speed_ratio_;
            double turn_duration = turn_arc_ / (speed_ms > 0.01 ? speed_ms : 0.625) + 4.0;
            double elapsed = this->now().seconds() - turn_start_time_;
            if (turning_ && elapsed < turn_duration) {
                twist.linear.x = speed_;
                twist.angular.z = 110;
                cmd_vel_pub_->publish(twist);
                RCLCPP_INFO(this->get_logger(), "turning, t=%.2f/%.2f", elapsed, turn_duration);
                return;
            }
            turning_ = false;
            twist.linear.x = 1500;
            twist.angular.z = base_angle_;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "turn done, stop waiting");
            return;
        }
        else if (red_cone_Index == 0 || blue_cone_Index == 0) {
            // 情况2：单侧有锥桶 → 比例保持 lane_hold_ 距离（★替代固定角度避让）
            no_cone_state_ = 0;
            turning_ = false;
            twist.linear.x = speed_;
            if (red_cone_Index > 0) {
                // 只有红排（x>0）→ 以最近红桶为基准保持 lane_hold_
                int nearest = 0;
                for (int m = 1; m < red_cone_Index; m++) {
                    if (red_p[m].y < red_p[nearest].y) nearest = m;
                }
                twist.angular.z = base_angle_ + singleSideAngle(red_p[nearest], true);
                RCLCPP_INFO(this->get_logger(), "only red, hold %.2fm, angle=%.1f",
                            red_p[nearest].x, twist.angular.z);
            } else {
                // 只有蓝排（x<0）→ 以最近蓝桶为基准保持 lane_hold_
                int nearest = 0;
                for (int m = 1; m < blue_cone_Index; m++) {
                    if (blue_p[m].y < blue_p[nearest].y) nearest = m;
                }
                twist.angular.z = base_angle_ + singleSideAngle(blue_p[nearest], false);
                RCLCPP_INFO(this->get_logger(), "only blue, hold %.2fm, angle=%.1f",
                            blue_p[nearest].x, twist.angular.z);
            }
            cmd_vel_pub_->publish(twist);
            return;
        }
        else {
            // 情况3：两侧都有 → 循迹
            no_cone_state_ = 0;
            turning_ = false;
        }
        // ===================================

        // 第二步：修正锥桶分组
        if (blue_cone_Index >= red_cone_Index) {
            int m = 100;
            for (int i = 1; i <= blue_cone_Index - 1; i++) {
                double dist = std::sqrt(
                    std::pow(blue_p[i-1].x - blue_p[i].x, 2) +
                    std::pow(blue_p[i-1].y - blue_p[i].y, 2)
                );
                if (dist >= 1.4) { m = i; break; }
            }
            if (m != 100) {
                for (int i = red_cone_Index - 1; i >= 0; i--)
                    red_p[i + blue_cone_Index - m] = red_p[i];
                for (int i = 0; i <= blue_cone_Index - m - 1; i++)
                    red_p[i] = blue_p[m + i];
                red_cone_Index += (blue_cone_Index - m);
                blue_cone_Index = m;
            }
        }
        else {
            int m = 100;
            for (int i = red_cone_Index - 2; i >= 0; i--) {
                double dist = std::sqrt(
                    std::pow(red_p[i].x - red_p[i+1].x, 2) +
                    std::pow(red_p[i].y - red_p[i+1].y, 2)
                );
                if (dist >= 1.4) { m = i; }
            }
            if (m != 100) {
                for (int i = 0; i <= m; i++)
                    blue_p[blue_cone_Index + i] = red_p[i];
                for (int i = 0; i <= red_cone_Index - m - 2; i++)
                    red_p[i] = red_p[i + m + 1];
                red_cone_Index -= (m + 1);
                blue_cone_Index += (m + 1);
            }
        }

        // 打印锥桶坐标(★改为debug_dump参数控制,默认关。原实现每帧N行刷屏,
        //   是3核板子负载飙到19、SSH掉线的元凶之一;需要时 ros2 param set ... debug_dump true)
        if (debug_dump_) {
            for (int i = 0; i <= red_cone_Index - 1; i++)
                RCLCPP_INFO(this->get_logger(), "red_bucket:(%.2f, %.2f),lidar scan times:%d", red_p[i].x, red_p[i].y, ranges_size);
            for (int i = 0; i <= blue_cone_Index - 1; i++)
                RCLCPP_INFO(this->get_logger(), "blue_bucket:(%.2f, %.2f),lidar scan times:%d", blue_p[i].x, blue_p[i].y, ranges_size);
        }

        // 第三步：计算误差（只配距离小车最近的一对红蓝锥桶）
        // ★修复:优先取前方(y >= pair_ahead_min)的锥桶配对。原实现取"全局最近",
        //   最近锥桶往往在车辆正侧方(y≈0.4m),配对中点落在车旁而不是车前,
        //   误差恒定→恒定转向→原地绕圈(8月30日实测转向恒定76°画圈,直线走不明白)。
        int nearest_red = -1;          // 最终选择(前方优先)
        int red_global = -1;           // 全局最近(兜底)
        double nearest_red_dist = 1e9;
        double best_red_ahead = 1e9;
        for (i = 0; i < red_cone_Index; i++) {
            if (red_p[i].y < nearest_red_dist) { nearest_red_dist = red_p[i].y; red_global = i; }
            if (red_p[i].y >= pair_ahead_min_ && red_p[i].y < best_red_ahead) {
                best_red_ahead = red_p[i].y; nearest_red = i;
            }
        }
        if (nearest_red < 0) nearest_red = red_global;
        // 找最近的蓝色锥桶(同样优先前方,无前方锥桶退回全局最近)
        int nearest_blue = -1;
        int blue_global = -1;
        double nearest_blue_dist = 1e9;
        double best_blue_ahead = 1e9;
        for (i = 0; i < blue_cone_Index; i++) {
            double d = blue_p[i].y;
            if (d < nearest_blue_dist) { nearest_blue_dist = d; blue_global = i; }
            if (blue_p[i].y >= pair_ahead_min_ && blue_p[i].y < best_blue_ahead) {
                best_blue_ahead = blue_p[i].y; nearest_blue = i;
            }
        }
        if (nearest_blue < 0) nearest_blue = blue_global;
        // 最近红蓝锥桶配对，求中心线
        if (nearest_red >= 0 && nearest_blue >= 0 &&
            std::abs(red_p[nearest_red].y - blue_p[nearest_blue].y) <= pair_y_match_) {
            // ★新增:纵向距离校验。最近的红桶和蓝桶前后相差太大时(弯道/漏检),
            //   它们的中点不是赛道中心,强行取中点会把方向带偏。
            error = (blue_p[nearest_blue].x + red_p[nearest_red].x) / 2.0;
            if (now_s - last_pair_log_time_ >= 0.5) {
                RCLCPP_INFO(this->get_logger(), "配对: 红(%.2f,%.2f) 蓝(%.2f,%.2f) err=%.3f",
                            red_p[nearest_red].x, red_p[nearest_red].y,
                            blue_p[nearest_blue].x, blue_p[nearest_blue].y, error);
                last_pair_log_time_ = now_s;
            }
        } else if (nearest_red >= 0 || nearest_blue >= 0) {
            // ★新增:配对失败(或纵向错位太大)→ 退化为单侧保持,不再硬取中点
            if (nearest_red >= 0 && (nearest_blue < 0 || red_p[nearest_red].y <= blue_p[nearest_blue].y)) {
                error = 0.0;
                angle = singleSideAngle(red_p[nearest_red], true);
                if (now_s - last_pair_log_time_ >= 0.5) {
                    RCLCPP_INFO(this->get_logger(), "配对失败,单侧跟红排 x=%.2f angle=%.1f",
                                red_p[nearest_red].x, angle);
                    last_pair_log_time_ = now_s;
                }
            } else {
                error = 0.0;
                angle = singleSideAngle(blue_p[nearest_blue], false);
                if (now_s - last_pair_log_time_ >= 0.5) {
                    RCLCPP_INFO(this->get_logger(), "配对失败,单侧跟蓝排 x=%.2f angle=%.1f",
                                blue_p[nearest_blue].x, angle);
                    last_pair_log_time_ = now_s;
                }
            }
            twist.angular.z = base_angle_ + angle;
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            twist.linear.x = speed_;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            cmd_vel_pub_->publish(twist);
            return;
        } else {
            error = 0.0;
        }

        angle = error * gain_;

        // 第四步：构造速度指令
        twist.angular.z = base_angle_ + angle;
        if (twist.angular.z > 150) twist.angular.z = 150;
        if (twist.angular.z < 30) twist.angular.z = 30;
        twist.linear.x = speed_;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;

        cmd_vel_pub_->publish(twist);
    }

public:
    LaserGoNode() : Node("laser_go") {
        // [新增] 声明可调参数
        this->declare_parameter("turn_arc", 6.283);
        this->declare_parameter("speed_ratio", 0.025);
        turn_arc_ = this->get_parameter("turn_arc").as_double();
        speed_ratio_ = this->get_parameter("speed_ratio").as_double();

        // ★新增:循迹核心参数(支持 ros2 param set 在线调整)
        this->declare_parameter("speed", 1525.0);          // 巡航速度PWM
        this->declare_parameter("base_angle", 81.75);      // 舵机基准角(原车微调值)
        this->declare_parameter("gain", 33.0);             // 误差(m)→舵角(°)增益
        this->declare_parameter("lane_hold", 0.75);        // 单侧循迹保持距离(m)
        this->declare_parameter("pair_y_match", 1.6);      // 红蓝配对纵向距离容差(m)
        this->declare_parameter("pair_ahead_min", 1.0);    // 配对优先取前方锥桶的最近纵向距离(m)
        this->declare_parameter("debug_dump", false);      // 锥桶坐标逐帧打印开关
        debug_dump_ = this->get_parameter("debug_dump").as_bool();
        // ★新增:正前方障碍处置参数
        this->declare_parameter("ahead_dist", 0.5);        // 触发距离(m)
        this->declare_parameter("ahead_width", 0.30);      // 触发横向半宽(m)
        this->declare_parameter("avoid_angle", 35.0);      // 绕行转向角(度)
        this->declare_parameter("avoid_speed", 1518.0);    // 绕行速度PWM
        this->declare_parameter("avoid_hold_s", 1.2);      // 绕行保持时间(s)
        ahead_dist_ = this->get_parameter("ahead_dist").as_double();
        ahead_width_ = this->get_parameter("ahead_width").as_double();
        avoid_angle_ = this->get_parameter("avoid_angle").as_double();
        avoid_speed_ = this->get_parameter("avoid_speed").as_double();
        avoid_hold_s_ = this->get_parameter("avoid_hold_s").as_double();
        this->declare_parameter("topic", "/teleop_cmd_vel"); // 输出话题(比赛链路重映射到 /racecar/follow_cmd_vel)
        speed_ = this->get_parameter("speed").as_double();
        base_angle_ = this->get_parameter("base_angle").as_double();
        gain_ = this->get_parameter("gain").as_double();
        lane_hold_ = this->get_parameter("lane_hold").as_double();
        pair_y_match_ = this->get_parameter("pair_y_match").as_double();
        pair_ahead_min_ = this->get_parameter("pair_ahead_min").as_double();
        std::string topic = this->get_parameter("topic").as_string();

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 5);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 5,
            std::bind(&LaserGoNode::laser_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(),
                    "LaserGoNode initialized, topic=%s speed=%.0f base=%.2f gain=%.1f lane_hold=%.2f",
                    topic.c_str(), speed_, base_angle_, gain_, lane_hold_);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserGoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}