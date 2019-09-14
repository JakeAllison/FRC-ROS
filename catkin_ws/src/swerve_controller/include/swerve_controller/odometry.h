#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace swerve_controller {
    namespace bacc = boost::accumulators;
    class Odometry {
        public:
        typedef boost::function<void(double, double)> IntegrationFunction;
        Odometry(size_t velocity_rolling_window_size=10);
        void init(const ros::Time& time);
        bool update(double front_left_wheel_pos, double front_left_pivot_pos, 
                    double front_right_wheel_pos, double front_right_pivot_pos,
                    double back_left_wheel_pos, double back_left_pivot_pos,
                    double back_right_wheel_pos, double back_right_pivot_pos,
                    const ros::Time& time);
        void updateOpenLoop(double linearX, double linearY, double angular, const ros::Time& time);
        double getHeading() const { return heading_; }
        double getX() const { return x_; }
        double getY() const { return y_; }
        double getLinearX() const { return linearX_; }
        double getLinearY() const { return linearY_; }
        double getAngular() const { return angular_; }
        void setWheelParams(double wheelbase_width, double wheelbase_length, double wheel_radius);
        void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);
        private:
        typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean>> RollingMeanAcc;
        typedef bacc::tag::rolling_window RollingWindow;
        void integrateRungeKutta2(double linearX, double linearY, double angular);
        void integrateExact(double linearX, double linearY, double angular);
        void resetAccumulators();
        ros::Time timestamp_;
        double x_;
        double y_;
        double heading_;
        double linearX_;
        double linearY_;
        double angular_;
        double wheelbase_width_;
        double wheelbase_length_;
        double wheel_radius_;
        double front_left_wheel_old_pos_;
        double front_right_wheel_old_pos_;
        double back_left_wheel_old_pos_;
        double back_right_wheel_old_pos_;
        size_t velocity_rolling_window_size_;
        RollingMeanAcc linearX_acc_;
        RollingMeanAcc linearY_acc_;
        RollingMeanAcc angular_acc_;
        IntegrationFunction integrate_fun_;
    };
} //namespace swerve_controller

#endif