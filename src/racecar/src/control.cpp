#include "racecar/control.hpp"

namespace racecar
{
PID::PID()
: kp_(0.0),
  ki_(0.0),
  kd_(0.0),
  error_out_(0.0),
  last_error_(0.0),
  integral_(0.0),
  integral_max_(0.0),
  last_diff_(0.0)
{
  Init();
}

double PID::PIDPositional(double error)
{
  integral_ += error;
  if (integral_ > integral_max_)
  {
    integral_ = integral_max_;
  }
  error_out_ = (kp_ * error) + (ki_ * integral_) + (kd_ * (error - last_error_));
  last_error_ = error;
  return error_out_;
}

double PID::PIDIncremental(double error)
{
  error_out_ = kp_ * (error - last_error_) + ki_ * error + kd_ * ((error - last_error_) - last_diff_);
  last_diff_ = error - last_error_;
  last_error_ = error;
  return error_out_;
}

void PID::Init()
{
  kp_ = 3.0;
  ki_ = 0.0;
  kd_ = 1.0;
  error_out_ = 0.0;
  last_error_ = 0.0;
  integral_ = 0.0;
  integral_max_ = 8.0;
  last_diff_ = 0.0;
}
}  // namespace racecar
