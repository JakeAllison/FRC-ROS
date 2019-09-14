#ifndef SPEED_LIMITER_H
#define SPEED_LIMITER_H

namespace swerve_controller
{
    class SpeedLimiter
    {
        public:
        SpeedLimiter(
            bool has_velocity_limits = false,
            bool has_acceleration_limits = false,
            bool has_jerk_limits = false,
            double min_velocity = 0.0,
            double max_velocity = 0.0,
            double min_acceleration = 0.0,
            double max_acceleration = 0.0,
            double min_jerk = 0.0,
            double max_jerk = 0.0
        );

        double limit(double& v, double v0, double v1, double dt);
        double limit_velocity(double& v);
        double limit_acceleration(double& v, double v0, double dt);
        double limit_jerk(double& v, double v0, double v1, double dt);

        public:
        bool has_velocity_limits;
        bool has_acceleration_limits;
        bool has_jerk_limits;

        double min_velocity;
        double max_velocity;

        double min_acceleration;
        double max_acceleration;

        double min_jerk;
        double max_jerk;
    };
} // namespace swerve_controller

#endif