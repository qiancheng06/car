#pragma once

namespace racecar
{
class PID
{
public:
  PID();

  double PIDPositional(double error);
  double PIDIncremental(double error);
  void Init();

private:
  double kp_;
  double ki_;
  double kd_;
  double error_out_;
  double last_error_;
  double integral_;
  double integral_max_;
  double last_diff_;
};
}
