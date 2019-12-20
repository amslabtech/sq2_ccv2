#ifndef __CCV2_TELEOPERATOR_H
#define __CCV2_TELEOPERATOR_H

#include <ctime>
#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include "ypspur_mqtt/velocity_data.h"

#include <mosquitto.h>

class CCV2Teleoperator
{
public:
    CCV2Teleoperator(void);

    void process(void);
    void joy_callback(const sensor_msgs::JoyConstPtr&);

private:
    // axes
    static constexpr int L_STICK_H = 0;
    static constexpr int L_STICK_V = 1;
    static constexpr int L2_STICK = 2;
    static constexpr int R_STICK_H = 3;
    static constexpr int R_STICK_V = 4;
    static constexpr int R2_STICK = 5;
    // buttons
    static constexpr int L1 = 4;
    static constexpr int L2 = 6;
    static constexpr int R2 = 7;

    double MAX_VELOCITY;
    double MAX_ANGULAR_VELOCITY;
    double MAX_STEERING_ANGLE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber joy_sub;

    // MQTT
    struct mosquitto *mosq;
};

#endif// __CCV2_TELEOPERATOR_H
