/**
 * @file PID.h
 * @brief 位置式 PID 控制器(参考实现配套头文件)
 * @details 与 lidar_tracking 包的 control.hpp 同源,放本包 src 目录下
 */

namespace Control
{
    class PID
    {
    public:
        double kp;                    // 比例系数
        double ki;                    // 积分系数
        double kd;                    // 微分系数
        double error_out;             // 控制器输出
        double last_error;            // 上一次误差
        double integral;              // 积分项
        double inte_max;              // 积分限幅
        double last_diff;             // 上一次微分值(增量式用)

        double PIDPositional(double error)
        {
            integral += error;
            if (integral > inte_max)
                integral = inte_max;
            if (integral < -inte_max)
                integral = -inte_max;

            error_out = (kp * error) + (ki * integral) + (kd * (error - last_error));
            last_error = error;
            return error_out;
        }

        double PIDIncremental(double error)
        {
            error_out = kp * (error - last_error) +
                        ki * error +
                        kd * ((error - last_error) - last_diff);
            last_diff = error - last_error;
            last_error = error;
            return error_out;
        }

        void Init()
        {
            kp = 0.88;
            ki = 0.0;
            kd = 1.2;
            error_out = 0.0;
            last_error = 0.0;
            integral = 0.0;
            inte_max = 8.0;
            last_diff = 0.0;
        }
    };
} // namespace Control
