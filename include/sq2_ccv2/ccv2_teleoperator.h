#ifndef __CCV2_TELEOPERATOR_H
#define __CCV2_TELEOPERATOR_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

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
    static constexpr int R_STICK_H = 2;
    static constexpr int L2_STICK = 3;
    static constexpr int R2_STICK = 4;
    static constexpr int R_STICK_V = 5;
    // buttons
    static constexpr int L1 = 4;
    static constexpr int L2 = 6;
    static constexpr int R2 = 7;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Subscriber joy_sub;

    // MQTT
    // Mosquitto cmd_vel_publisher;
};

#endif// __CCV2_TELEOPERATOR_H
