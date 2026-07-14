// Safety-gated serial bridge for the physical racecar.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "../include/racecar_driver.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class RacecarDriver : public rclcpp::Node
{
public:
  RacecarDriver()
  : Node("racecar_driver")
  {
    this->declare_parameter<std::string>("serial_port", "/dev/car");
    this->declare_parameter<int>("baud_rate", 38400);
    this->declare_parameter<double>("command_timeout_s", 0.50);

    // Only the safety adapter publishes this final actuator command topic.
    this->declare_parameter<std::string>("command_topic", "/racecar_driver/cmd_pwm");
    this->declare_parameter<std::string>("arm_topic", "/nav/arm");
    this->declare_parameter<std::string>("estop_topic", "/nav/estop");
    this->declare_parameter<bool>("default_armed", false);
    this->declare_parameter<bool>("enable_legacy_inputs", false);
    this->declare_parameter<bool>("enable_legacy_pwm_input", false);
    this->declare_parameter<bool>("enable_legacy_normalized_input", false);

    // Conservative first-run actuator envelope. Reverse requires an explicit
    // launch override and is enabled only for supervised manual control.
    this->declare_parameter<double>("neutral_throttle_pwm", 1500.0);
    this->declare_parameter<double>("min_throttle_pwm", 1500.0);
    this->declare_parameter<double>("max_throttle_pwm", 1550.0);
    this->declare_parameter<double>("neutral_steering_pwm", 1500.0);
    this->declare_parameter<double>("min_steering_pwm", 1200.0);
    this->declare_parameter<double>("max_steering_pwm", 1800.0);

    // Optional legacy conversions. They remain disabled in the safe launch.
    this->declare_parameter<double>("normalized_speed_to_pwm", 100.0);
    this->declare_parameter<double>("normalized_yaw_to_pwm", 1300.0);
    this->declare_parameter<double>("teleop_zero_degree_pwm", 2500.0);
    this->declare_parameter<double>("teleop_degree_to_pwm", 2000.0 / 180.0);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    command_timeout_s_ = this->get_parameter("command_timeout_s").as_double();
    command_topic_ = this->get_parameter("command_topic").as_string();
    arm_topic_ = this->get_parameter("arm_topic").as_string();
    estop_topic_ = this->get_parameter("estop_topic").as_string();
    armed_ = this->get_parameter("default_armed").as_bool();
    const bool enable_all_legacy = this->get_parameter("enable_legacy_inputs").as_bool();
    enable_legacy_pwm_input_ =
      enable_all_legacy || this->get_parameter("enable_legacy_pwm_input").as_bool();
    enable_legacy_normalized_input_ =
      enable_all_legacy || this->get_parameter("enable_legacy_normalized_input").as_bool();

    neutral_throttle_pwm_ = this->get_parameter("neutral_throttle_pwm").as_double();
    min_throttle_pwm_ = this->get_parameter("min_throttle_pwm").as_double();
    max_throttle_pwm_ = this->get_parameter("max_throttle_pwm").as_double();
    neutral_steering_pwm_ = this->get_parameter("neutral_steering_pwm").as_double();
    min_steering_pwm_ = this->get_parameter("min_steering_pwm").as_double();
    max_steering_pwm_ = this->get_parameter("max_steering_pwm").as_double();
    normalized_speed_to_pwm_ = this->get_parameter("normalized_speed_to_pwm").as_double();
    normalized_yaw_to_pwm_ = this->get_parameter("normalized_yaw_to_pwm").as_double();
    teleop_zero_degree_pwm_ = this->get_parameter("teleop_zero_degree_pwm").as_double();
    teleop_degree_to_pwm_ = this->get_parameter("teleop_degree_to_pwm").as_double();

    validateParameters();

    std::vector<char> serial_port_buffer(serial_port_.begin(), serial_port_.end());
    serial_port_buffer.push_back('\0');
    const int init_result = art_racecar_init(baud_rate_, serial_port_buffer.data());
    if (init_result < 0) {
      throw std::runtime_error("Unable to initialize racecar serial port " + serial_port_);
    }

    // Stop before accepting any ROS command.
    for (int i = 0; i < 3; ++i) {
      stopNow();
    }

    arm_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      arm_topic_, 10,
      std::bind(&RacecarDriver::armCallback, this, std::placeholders::_1));
    estop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      estop_topic_, 10,
      std::bind(&RacecarDriver::estopCallback, this, std::placeholders::_1));

    if (!enable_legacy_pwm_input_ && !enable_legacy_normalized_input_) {
      command_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        command_topic_, 10,
        std::bind(&RacecarDriver::pwmCommandCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Protected navigation input %s is active", command_topic_.c_str());
    } else if (enable_legacy_normalized_input_) {
      normalized_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/car_cmd_vel", 5,
        std::bind(&RacecarDriver::normalizedTwistCallback, this, std::placeholders::_1));
      RCLCPP_WARN(this->get_logger(), "Legacy normalized input /car_cmd_vel is enabled");
    } else {
      legacy_pwm_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/teleop_cmd_vel", 5,
        std::bind(&RacecarDriver::pwmCommandCallback, this, std::placeholders::_1));
      RCLCPP_WARN(this->get_logger(), "Legacy PWM input /teleop_cmd_vel is enabled");
    }

    last_command_time_ = std::chrono::steady_clock::now();
    watchdog_timer_ = this->create_wall_timer(50ms, std::bind(&RacecarDriver::watchdog, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Racecar driver ready on %s at %d baud; command=%s; armed=%s; watchdog=%.2f s",
      serial_port_.c_str(), baud_rate_, command_topic_.c_str(), armed_ ? "true" : "false",
      command_timeout_s_);
  }

  ~RacecarDriver() override
  {
    for (int i = 0; i < 3; ++i) {
      stopNow();
    }
  }

private:
  static bool finiteTwist(const geometry_msgs::msg::Twist & twist)
  {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
           std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
           std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
  }

  static void validatePwmRange(
    const char * name, double minimum, double neutral, double maximum)
  {
    if (!std::isfinite(minimum) || !std::isfinite(neutral) || !std::isfinite(maximum) ||
      minimum < 0.0 || minimum > neutral || neutral > maximum || maximum > 65535.0)
    {
      throw std::invalid_argument(std::string("Invalid ") + name + " PWM range");
    }
  }

  void validateParameters() const
  {
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ < 0.05) {
      throw std::invalid_argument("command_timeout_s must be finite and >= 0.05");
    }
    if (enable_legacy_pwm_input_ && enable_legacy_normalized_input_) {
      throw std::invalid_argument("Only one chassis command input mode may be enabled");
    }
    validatePwmRange(
      "throttle", min_throttle_pwm_, neutral_throttle_pwm_, max_throttle_pwm_);
    validatePwmRange(
      "steering", min_steering_pwm_, neutral_steering_pwm_, max_steering_pwm_);
    if (!std::isfinite(normalized_speed_to_pwm_) || normalized_speed_to_pwm_ <= 0.0 ||
      !std::isfinite(normalized_yaw_to_pwm_) || normalized_yaw_to_pwm_ <= 0.0 ||
      !std::isfinite(teleop_zero_degree_pwm_) ||
      !std::isfinite(teleop_degree_to_pwm_) || teleop_degree_to_pwm_ <= 0.0)
    {
      throw std::invalid_argument("Invalid command conversion parameter");
    }
  }

  void stopNow() const
  {
    (void)send_cmd(
      static_cast<uint16_t>(std::lround(neutral_throttle_pwm_)),
      static_cast<uint16_t>(std::lround(neutral_steering_pwm_)));
  }

  void rejectAndStop(const std::string & reason)
  {
    command_received_ = false;
    stopNow();
    RCLCPP_ERROR(this->get_logger(), "Rejected chassis command: %s", reason.c_str());
  }

  void sendMotionCommand(double throttle_pwm, double steering_pwm)
  {
    if (!std::isfinite(throttle_pwm) || !std::isfinite(steering_pwm)) {
      rejectAndStop("non-finite actuator value");
      return;
    }
    if (throttle_pwm < min_throttle_pwm_ || throttle_pwm > max_throttle_pwm_) {
      rejectAndStop("throttle is outside the safe PWM envelope");
      return;
    }
    if (steering_pwm < min_steering_pwm_ || steering_pwm > max_steering_pwm_) {
      rejectAndStop("steering is outside the safe PWM envelope");
      return;
    }

    (void)send_cmd(
      static_cast<uint16_t>(std::lround(throttle_pwm)),
      static_cast<uint16_t>(std::lround(steering_pwm)));
    last_command_time_ = std::chrono::steady_clock::now();
    command_received_ = true;
    watchdog_reported_ = false;

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Chassis command: throttle_pwm=%.0f steering_pwm=%.0f",
      throttle_pwm, steering_pwm);
  }

  bool outputEnabled()
  {
    if (!armed_ || estop_) {
      stopNow();
      return false;
    }
    return true;
  }

  void pwmCommandCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    if (!outputEnabled()) {
      return;
    }
    if (!finiteTwist(*twist)) {
      rejectAndStop("non-finite Twist");
      return;
    }

    const double throttle_pwm = twist->linear.x;
    const double steering_pwm =
      teleop_zero_degree_pwm_ - twist->angular.z * teleop_degree_to_pwm_;
    sendMotionCommand(throttle_pwm, steering_pwm);
  }

  void normalizedTwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    if (!outputEnabled()) {
      return;
    }
    if (!finiteTwist(*twist)) {
      rejectAndStop("non-finite legacy Twist");
      return;
    }

    const double throttle_pwm =
      neutral_throttle_pwm_ + twist->linear.x * normalized_speed_to_pwm_;
    const double steering_pwm =
      neutral_steering_pwm_ - twist->angular.z * normalized_yaw_to_pwm_;
    sendMotionCommand(throttle_pwm, steering_pwm);
  }

  void armCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && estop_) {
      RCLCPP_WARN(this->get_logger(), "Arm request ignored while software stop is active");
      return;
    }
    armed_ = msg->data;
    command_received_ = false;
    watchdog_reported_ = false;
    stopNow();
    RCLCPP_INFO(this->get_logger(), "Driver output %s", armed_ ? "ARMED" : "DISARMED");
  }

  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    estop_ = msg->data;
    armed_ = false;
    command_received_ = false;
    watchdog_reported_ = false;
    stopNow();
    if (estop_) {
      RCLCPP_ERROR(this->get_logger(), "SOFTWARE STOP asserted; driver is disarmed");
    } else {
      RCLCPP_WARN(this->get_logger(), "Software stop released; explicit re-arm is required");
    }
  }

  void watchdog()
  {
    const auto now = std::chrono::steady_clock::now();
    const double age = std::chrono::duration<double>(now - last_command_time_).count();
    const bool stale = !command_received_ || age > command_timeout_s_;

    if (!armed_ || estop_ || stale) {
      // Repeat neutral at 20 Hz; one lost serial frame must not leave stale throttle active.
      stopNow();
      if (armed_ && !estop_ && stale && !watchdog_reported_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Command watchdog is holding neutral (last command age %.2f s)", age);
        watchdog_reported_ = true;
      }
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr normalized_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr legacy_pwm_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::string serial_port_;
  std::string command_topic_;
  std::string arm_topic_;
  std::string estop_topic_;
  int baud_rate_{38400};
  double command_timeout_s_{0.50};
  double neutral_throttle_pwm_{1500.0};
  double min_throttle_pwm_{1500.0};
  double max_throttle_pwm_{1550.0};
  double neutral_steering_pwm_{1500.0};
  double min_steering_pwm_{1200.0};
  double max_steering_pwm_{1800.0};
  double normalized_speed_to_pwm_{100.0};
  double normalized_yaw_to_pwm_{1300.0};
  double teleop_zero_degree_pwm_{2500.0};
  double teleop_degree_to_pwm_{2000.0 / 180.0};

  std::chrono::steady_clock::time_point last_command_time_;
  bool armed_{false};
  bool estop_{false};
  bool enable_legacy_pwm_input_{false};
  bool enable_legacy_normalized_input_{false};
  bool command_received_{false};
  bool watchdog_reported_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacecarDriver>());
  rclcpp::shutdown();
  return 0;
}
