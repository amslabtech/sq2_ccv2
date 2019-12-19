#include "sq2_ccv2/ccv2_teleoperator.h"

CCV2Teleoperator::CCV2Teleoperator(void)
:local_nh("~")
{
    std::cout << "=== ccv2_teleoperator ===" << std::endl;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub = nh.subscribe("joy", 1, &CCV2Teleoperator::joy_callback, this, ros::TransportHints().tcpNoDelay());
    local_nh.param<double>("MAX_VELOCITY", MAX_VELOCITY, {1.0});
    local_nh.param<double>("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY, {M_PI});
    local_nh.param<double>("MAX_STEERING_ANGLE", MAX_STEERING_ANGLE, {M_PI / 12.0});
}

void CCV2Teleoperator::process(void)
{
    ros::spin();
}

void CCV2Teleoperator::joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
    double v = 0.0;
    double w = 0.0;
    double steering = 0.0;
    if(msg->buttons[L1]){
        v = msg->axes[L_STICK_V] * MAX_VELOCITY;
        w = msg->axes[L_STICK_H] * MAX_ANGULAR_VELOCITY;
        if(msg->buttons[L2] && !msg->buttons[R2]){
            w = 0.0;
            steering = MAX_STEERING_ANGLE * (1.0 - (msg->axes[L2_STICK] + 1.0) * 0.5);
        }else if(msg->buttons[R2] && !msg->buttons[L2]){
            w = 0.0;
            steering = -MAX_STEERING_ANGLE * (1.0 - (msg->axes[R2_STICK] + 1.0) * 0.5);
        }else if(msg->buttons[R2] && msg->buttons[L2]){
            std::cout << "brake" << std::endl;
            v = 0.0;
            w = 0.0;
        }
    }else{
        std::cout << "press L1 to move" << std::endl;
    }
    std::cout << "v: " << v << ", " << "w: " << w << ", " << "steering: " << steering << std::endl;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = w;
    cmd_vel_pub.publish(cmd_vel);
    YPSpurWrapper::VelocityData vd;
}
