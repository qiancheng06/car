#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "racecar/control.hpp"

namespace
{
constexpr int kScanResolution = 1440;
constexpr int kBucketThreshold = 10;
constexpr int kValidBucketThreshold = 6;
}

struct BucketPoint
{
  double x{0.0};
  double y{0.0};
};

class LaserControl : public rclcpp::Node
{
public:
  LaserControl()
  : rclcpp::Node("car_test"),
    pid_(),
    speed_(declare_parameter<double>("speed", 1530.0)),
    max_left_distance_(declare_parameter<double>("max_left_distance", 1.8)),
    min_right_distance_(declare_parameter<double>("min_right_distance", 1.5)),
    base_link_x_(0.0),
    base_link_y_(0.0),
    flag_go_(false),
    flag_stop_(false),
    flag_change_(false)
  {
    rclcpp::QoS qos{rclcpp::KeepLast(10)};
    laser_pub_ = create_publisher<geometry_msgs::msg::Twist>("laser_control", qos);
    //laser_pub_ = create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", qos);
    stop_pub_ = create_publisher<std_msgs::msg::Bool>("red_stop", qos);
    start_change_pub_ = create_publisher<std_msgs::msg::Bool>("start_change", qos);

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      std::bind(&LaserControl::laserCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/encoder_imu_odom", qos,
      std::bind(&LaserControl::odomCallback, this, std::placeholders::_1));

    twist_.angular.x = 0.0;
    twist_.angular.y = 0.0;
    twist_.angular.z = 90.0;
    twist_.linear.x = speed_;
    twist_.linear.y = 0.0;
    twist_.linear.z = 0.0;

    RCLCPP_INFO(get_logger(), "car_test node initialised");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    base_link_x_ = msg->pose.pose.position.x;
    base_link_y_ = msg->pose.pose.position.y;
    const double pose_norm = std::hypot(base_link_x_, base_link_y_);
    RCLCPP_INFO(get_logger(), "base_link_x: %.3f  base_link_y: %.3f  pose_norm: %.3f",
      base_link_x_, base_link_y_, pose_norm);
    RCLCPP_INFO(get_logger(), "hello");

    if (pose_norm > 9.5 && !flag_go_)
    {
      flag_go_ = true;
      RCLCPP_INFO(get_logger(), "Vehicle left the origin region");
    }

    if (pose_norm <= 8.7 && flag_go_ && !flag_stop_)
    {
      flag_stop_ = true;
      std_msgs::msg::Bool msg_out;
      msg_out.data = true;
      stop_pub_->publish(msg_out);
      RCLCPP_INFO(get_logger(), "Red light stop triggered");
    }
    else if (pose_norm <= 2.3 && flag_go_ && !flag_change_)
    {
      flag_change_ = true;
      std_msgs::msg::Bool msg_out;
      msg_out.data = true;
      start_change_pub_->publish(msg_out);
      RCLCPP_INFO(get_logger(), "Start changeover triggered");
    }
    
  }

  void publishTwist()
  {
    twist_.linear.x = std::clamp(twist_.linear.x, 0.0, 2000.0);
    twist_.angular.z = std::clamp(twist_.angular.z, 0.0, 180.0);
    laser_pub_->publish(twist_);
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::array<BucketPoint, 30U> red_points{};
    std::array<BucketPoint, 30U> blue_points{};
    int red_count = 0;
    int blue_count = 0;
    RCLCPP_INFO(get_logger(), "hello");

    for (int i = 1; i < kScanResolution - kBucketThreshold; ++i)
    {
      const float current = msg->ranges[i];
      if (current <= 0.0f || current > 10.0f)
      {
        continue;
      }
      if (msg->ranges[i - 1] - current < 2.0f)
      {
        continue;
      }
      if (msg->ranges[i - 1] - msg->ranges[i + 1] < 2.0f)
      {
        continue;
      }

      int continue_ranges = 0;
      for (int idx = 1; idx < kBucketThreshold; ++idx)
      {
        if (std::abs(current - msg->ranges[i + idx]) < 0.2f)
        {
          ++continue_ranges;
          if (continue_ranges >= kValidBucketThreshold)
          {
            break;
          }
        }
      }

      if (continue_ranges < kValidBucketThreshold)
      {
        continue;
      }

      const double theta = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const double x = current * std::sin(theta);
      const double y = current * std::cos(theta);

      if (x <= max_left_distance_ && x >= -min_right_distance_ && y < 2.0 && y > -0.2)
      {
        if (x > 0.0 && red_count < static_cast<int>(red_points.size()))
        {
          red_points[red_count++] = BucketPoint{x, y};
        }
        else if (x <= 0.0 && blue_count < static_cast<int>(blue_points.size()))
        {
          blue_points[blue_count++] = BucketPoint{x, y};
        }
      }
    }

    adjustBucketCounts(red_points, red_count, blue_points, blue_count);

    const int error_count = std::min(red_count, blue_count);
    if (error_count > 0)
    {
      double weighted_error = 0.0;
      double weight_sum = 0.0;
      for (int idx = 0; idx < error_count; ++idx)
      {
        const double weight = (blue_points[idx].y + red_points[error_count - 1 - idx].y) / 2.0;
        weighted_error += (blue_points[idx].x + red_points[error_count - 1 - idx].x) * weight;
        weight_sum += weight;
      }

      double error = weight_sum > 0.0 ? (weighted_error / weight_sum) * 20.0 : 0.0;
      if(error>0)
      {
        error = error * 0.9;
      }
      const double angle = pid_.PIDPositional(error);
      twist_.angular.z = 90.0 + angle;
      twist_.linear.x = speed_;

      if (std::abs(angle) >= 20.0)
      {
        twist_.linear.x -= 3.0;
      }

      if (twist_.angular.z > 170.0)
      {
        twist_.angular.z = 170.0;
      }
      else if (twist_.angular.z < 0.0)
      {
        twist_.angular.z = 0.0;
      }
    }

    publishTwist();
  }

  static void adjustBucketCounts(
    std::array<BucketPoint, 30U> &red_points,
    int &red_count,
    std::array<BucketPoint, 30U> &blue_points,
    int &blue_count)
  {
    if (blue_count >= red_count)
    {
      int split_index = 100;
      for (int i = 1; i < blue_count; ++i)
      {
        const double distance_squared =
          std::pow(blue_points[i - 1].x - blue_points[i].x, 2) +
          std::pow(blue_points[i - 1].y - blue_points[i].y, 2);
        if (distance_squared > 1.4 * 1.4)
        {
          split_index = i;
          break;
        }
      }

      if (split_index != 100)
      {
        for (int i = red_count - 1; i >= 0; --i)
        {
          if (i + blue_count - split_index < static_cast<int>(red_points.size()))
          {
            red_points[i + blue_count - split_index] = red_points[i];
          }
        }
        for (int i = 0; i < blue_count - split_index; ++i)
        {
          red_points[i] = blue_points[split_index + i];
        }
        red_count += (blue_count - split_index);
        blue_count = split_index;
      }
    }
    else
    {
      int split_index = 100;
      for (int i = red_count - 2; i >= 0; --i)
      {
        const double distance_squared =
          std::pow(red_points[i].x - red_points[i + 1].x, 2) +
          std::pow(red_points[i].y - red_points[i + 1].y, 2);
        if (distance_squared > 1.4 * 1.4)
        {
          split_index = i;
          break;
        }
      }

      if (split_index != 100)
      {
        for (int i = 0; i < split_index + 1 && blue_count + i < static_cast<int>(blue_points.size()); ++i)
        {
          blue_points[blue_count + i] = red_points[i];
        }
        for (int i = 0; i < red_count - split_index - 1; ++i)
        {
          red_points[i] = red_points[i + split_index + 1];
        }
        red_count -= (split_index + 1);
        blue_count += (split_index + 1);
      }
    }

    red_count = std::min(red_count, 3);
    blue_count = std::min(blue_count, 3);

    if (red_count > blue_count && blue_count > 0)
    {
      if (blue_count == 1)
      {
        if (red_count > 2)
        {
          for (int i = 0; i < 2; ++i)
          {
            red_points[i] = red_points[i + 1];
          }
          red_count = 2;
        }
        blue_points[1] = blue_points[0];
        blue_count = 2;
      }
      else if (blue_count == 2)
      {
        blue_points[2] = blue_points[1];
        blue_count = 3;
      }
    }
    else if (blue_count > red_count && red_count > 0)
    {
      if (red_count == 1)
      {
        blue_count = 2;
        red_points[1] = red_points[0];
        red_count = 2;
      }
      else if (red_count == 2)
      {
        red_points[2] = red_points[1];
        red_points[1] = red_points[0];
        red_count = 3;
      }
    }
  }

  racecar::PID pid_;
  double speed_;
  double max_left_distance_;
  double min_right_distance_;
  double base_link_x_;
  double base_link_y_;
  bool flag_go_;
  bool flag_stop_;
  bool flag_change_;

  geometry_msgs::msg::Twist twist_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr laser_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_change_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
