#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <swerve_controller/odometry.h>
#include <swerve_controller/speed_limiter.h>

namespace swerve_controller {
    class SwerveController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
        public:
        SwerveController();
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
        void update(const ros::Time& time, const ros::Duration& period);
        void starting(const ros::Time& time);
        void stopping(const ros::Time&);
        private:
        std::string name_;
        ros::Duration publish_period_;
        ros::Time last_state_publish_time_;
        bool open_loop_;

        hardware_interface::JointHandle front_left_wheel_joint_;
        hardware_interface::JointHandle front_right_wheel_joint_;
        hardware_interface::JointHandle back_left_wheel_joint_;
        hardware_interface::JointHandle back_right_wheel_joint_;

        hardware_interface::JointHandle front_left_pivot_joint_;
        hardware_interface::JointHandle front_right_pivot_joint_;
        hardware_interface::JointHandle back_left_pivot_joint_;
        hardware_interface::JointHandle back_right_pivot_joint_;

        struct Commands {
            double lin;
            double ang;
            double trans;
            ros::Time stamp;
            
            Commands() : lin(0.0) , ang(0.0), trans(0.0), stamp(0.0) {}
        };

        realtime_tools::RealtimeBuffer<Commands> command_;
        Commands command_struct_;
        ros::Subscriber sub_command_;

        boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> cmd_vel_pub_;

        boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
        boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> odom_pub_;
        Odometry odometry_;

        double wheelbase_width_;
        double wheelbase_length_;
        double wheel_radius_;

        double wheelbase_width_multiplier_;
        double wheelbase_length_multiplier_;
        double wheel_radius_multiplier_;

        double cmd_timeout_;

        bool allow_multiple_cmd_vel_publishers_;

        std::string base_frame_id_;

        std::string odom_frame_id_;

        bool enable_odom_tf_;

        size_t wheel_joint_size_;

        Commands last1_cmd_;
        Commands last0_cmd_;
        SpeedLimiter limiter_lin_;
        SpeedLimiter limiter_ang_;

        bool publish_cmd_;

        struct DynamicParams {
            bool update;

            double wheelbase_width_multiplier;
            double wheelbase_length_multiplier;
            double wheel_radius_multiplier;

            bool publish_cmd;

            double publish_rate;
            bool enable_odom_tf;

            DynamicParams()
                : wheelbase_width_multiplier(1.0)
                , wheelbase_length_multiplier(1.0)
                , wheel_radius_multiplier(1.0)
                , publish_cmd(false)
                , publish_rate(50)
                , enable_odom_tf(true)
            {}

            friend std::ostream& operator<<(std::ostream& os, const DynamicParams& params) {
                os << "DynamicParams:\n"
                //
                << "\tOdometry parameters:\n"
                << "\t\twheelbase length: "   << params.wheelbase_length_multiplier  << "\n"
                << "\t\twheelbase width: "  << params.wheelbase_width_multiplier << "\n"
                << "\t\twheel radius: "    << params.wheel_radius_multiplier   << "\n"
                //
                << "\tPublication parameters:\n"
                << "\t\tPublish executed velocity command: " << params.publish_cmd << "\n"
                << "\t\tPublication rate: " << params.publish_rate                 << "\n"
                << "\t\tPublish frame odom on tf: " << params.enable_odom_tf;

                return os;
            }
        };

        realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;

        //typedef dynamic_reconfigure::Server<SwerveControllerConfig> ReconfigureServer;
        //    boost::shared_ptr<ReconfigureServer> dyn_reconf_server_;

        private:
        void brake();
        void cmdVelCallback(const geometry_msgs::Twist& command);
        bool getWheelNames(ros::NodeHandle& controller_nh, const std::string& wheel_param, std::vector<std::string>& wheel_names);
        bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh, const std::string& front_left_wheel_name, const std::string& front_left_pivot_name,
            const std::string& front_right_wheel_name, const std::string& front_right_pivot_name,
            const std::string& back_left_wheel_name, const std::string& back_left_pivot_name,
            const std::string& back_right_wheel_name, const std::string& back_right_pivot_name,
            bool lookup_wheelbase_length, bool lookup_wheelbase_width, bool lookup_wheel_radius);

        void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
        //void reconfCallback(SwerveControllerConfig& config, uint_32_t);
        void updateDynamicParams();
    };

    PLUGINLIB_EXPORT_CLASS(swerve_controller::SwerveController, controller_interface::ControllerBase);
} //namespace swerve_controller