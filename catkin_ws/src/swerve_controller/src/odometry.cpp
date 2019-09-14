#include <swerve_controller/odometry.h>

#include <boost/bind.hpp>

namespace swerve_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linearX_(0.0)
  , linearY_(0.0)
  , angular_(0.0)
  , wheelbase_length_(0.0)
  , wheelbase_width_(0.0)
  , wheel_radius_(0.0)
  , front_left_wheel_old_pos_(0.0)
  , front_right_wheel_old_pos_(0.0)
  , back_left_wheel_old_pos_(0.0)
  , back_right_wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linearX_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , linearY_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2))
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double front_left_wheel_pos, double front_left_pivot_pos, 
                    double front_right_wheel_pos, double front_right_pivot_pos,
                    double back_left_wheel_pos, double back_left_pivot_pos,
                    double back_right_wheel_pos, double back_right_pivot_pos,
                    const ros::Time& time)
  {
    /// Get current wheel joint positions:
    const double left_wheel_cur_pos  = left_pos  * left_wheel_radius_;
    const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

    /// Update old position with current:
    left_wheel_old_pos_  = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
    const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

    /// Integrate odometry:
    integrate_fun_(linear, angular);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linearX_acc_(linearX/dt);
    linearY_acc_(linearY/dt);
    angular_acc_(angular/dt);

    linearX_ = bacc::rolling_mean(linearX_acc_);
    linearY_ = bacc::rolling_mean(linearY_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }

  void Odometry::updateOpenLoop(double linearX, double linearY, double angular, const ros::Time &time)
  {
    /// Save last linear and angular velocity:
    linearX_ = linearX;
    linearY_ = linearY;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linearX * dt, angular * dt);
    integrate_fun_(linearY * dt, angular * dt);
  }

  void Odometry::setWheelParams(double wheelbase_width, double wheelbase_length, double wheel_radius)
  {
    wheelbase_length_   = wheelbase_width;
    wheelbase_length_  = wheelbase_length;
    wheel_radius_ = wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linearX, double linearY, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linearX * cos(direction);
    y_       += linearY * sin(direction);
    heading_ += angular;
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateExact(double linearX, double linearY, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linearX, linearY, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linearX/angular;
      const double r2 = linearY/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
      //x_       +=  r2 * (sin(heading_) - sin(heading_old));
      //y_       += -r2 * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::resetAccumulators()
  {
    linearX_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    linearY_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace swerve_controller