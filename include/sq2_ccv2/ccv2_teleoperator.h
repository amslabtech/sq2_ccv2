#ifndef __CCV2_TELEOPERATOR_H
#define __CCV2_TELEOPERATOR_H

#include <ctime>
#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include "ypspur_mqtt/velocity_data.h"

#include <mosquitto.h>

#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>

class CCV2Teleoperator
{
public:
    CCV2Teleoperator(void);

    void process(void);
    void joy_callback(const sensor_msgs::JoyConstPtr&);
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void cmd_pos_callback(const ccv_dynamixel_msgs::CmdPoseByRadian::ConstPtr &msg);

private:
    // axes
    static constexpr int L_STICK_H = 0;
    static constexpr int L_STICK_V = 1;
    static constexpr int L2_STICK = 2;
    static constexpr int R_STICK_H = 3;
    static constexpr int R_STICK_V = 4;
    static constexpr int R2_STICK = 5;
    // buttons
    static constexpr int CROSS = 0;
    static constexpr int CIRCLE = 1;
    static constexpr int TRIANGLE = 2;
    static constexpr int SQUARE = 3;
    static constexpr int L1 = 4;
    static constexpr int L2 = 6;
    static constexpr int R2 = 7;

    double MAX_VELOCITY;
    double MAX_ANGULAR_VELOCITY;
    double MAX_STEERING_ANGLE;
    double MAX_PITCH_ANGLE;
    double MAX_ROLL_ANGLE;
    double PITCH_OFFSET;
    double STEER_R_OFFSET;
    double STEER_L_OFFSET;
    double TREAD;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher cmd_vel_pub;
    ros::Publisher pub_cmd_pos_;
    ros::Subscriber joy_sub;
    ros::Subscriber sub_cmd_vel_;
    ros::Subscriber sub_cmd_pos_;

    sensor_msgs::Joy joy_;
    geometry_msgs::Twist cmd_vel_;
    geometry_msgs::Twist joy_vel_;
    ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos_;
    ccv_dynamixel_msgs::CmdPoseByRadian joy_pos_;
    bool joy_subscribed;
    bool auto_flag_, move_flag_, joy_flag_;

    // MQTT
    struct mosquitto *mosq;
};

#endif// __CCV2_TELEOPERATOR_H
