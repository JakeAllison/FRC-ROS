#include "swerve_controller/swerve_controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "swerve_controller");
    SwerveController swerve;
    ros::spin();
    return 0;
}


SwerveController::SwerveController() {
    std::string temp1 = "cmd_vel_param";
    std::string temp2 = "cmd_vel";
    _nh.param<std::string>(temp1, _cmd_vel_topic, temp2);
    _sub = _nh.subscribe(_cmd_vel_topic, 1, &SwerveController::CmdVelCallback, this);
}

void SwerveController::CmdVelCallback(const geometry_msgs::Twist& msg) {
    _linx = msg.linear.x;
    _liny = msg.linear.y;
    _linz = msg.linear.z;
    _angx = msg.angular.x;
    _angy = msg.angular.y;
    _angz = msg.angular.z;
    
    //x + is forward
    //y + is left
    //rot + is CCW
    double strafeAxis = _liny;
    double fowAxis = _linx;
    double rotAxis = _angz;
    
    /*
    // Using magnitude of the strafe and forward rather than individuals.
    if((strafeAxis * strafeAxis + fowAxis * fowAxis) < (0.01 * 0.01)) {
        strafeAxis = 0;
        fowAxis = 0;
    }
    else {
        strafeAxis = strafeAxis * fabs(strafeAxis);
        fowAxis = fowAxis * fabs(fowAxis);
    }
    
    if(rotAxis > -0.01 && rotAxis < 0.01) {
        rotAxis = 0;
    }
    else {
        rotAxis = rotAxis * fabs(rotAxis);
    }
    */
    
    const auto xSpeed = _linx * kMeterPerSecond;
    const auto ySpeed = _liny * kMeterPerSecond;
    const auto rotSpeed = _angz * kRadianPerSecond;
    m_swerve.Drive(xSpeed, ySpeed, rotSpeed, false);
    
    ROS_INFO("Vel Received: [%f], [%f], [%f]", _linx, _liny, _angz);
}
