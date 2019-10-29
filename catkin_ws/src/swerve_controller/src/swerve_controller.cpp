#include "swerve_controller/swerve_controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "swerve_controller");
    SwerveController swervecontroller;
    ros::spin();
    return 0;
}


SwerveController::SwerveController() {
    std::string temp1 = "cmd_vel_param";
    std::string temp2 = "/cmd_vel";
    _nh.param<std::string>(temp1, _cmd_vel_topic, temp2);
    _sub = _nh.subscribe(_cmd_vel_topic, 10, &SwerveController::CmdVelCallback, this);
}

void SwerveController::CmdVelCallback(const geometry_msgs::Twist& msg) {
    _linx = msg.linear.x;
    _liny = msg.linear.y;
    _linz = msg.linear.z;
    _angx = msg.angular.x;
    _angy = msg.angular.y;
    _angz = msg.angular.z;
    
    ROS_INFO("Vel Received: [%f], [%f], [%f]", _linx, _liny, _angz);
}
