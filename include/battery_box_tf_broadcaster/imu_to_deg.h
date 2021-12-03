#ifndef IMU_TO_DEG_H
#define IMU_TO_DEG_H

#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>

class ImuToDeg
{
public:
    ImuToDeg();
    ~ImuToDeg();
    void process();

private:
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void dxl_state_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr&msg);

    int hz_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_dxl_state_;
    ros::Publisher pub_roll_;
    ros::Publisher pub_pitch_;
    ros::Publisher pub_yaw_;
    ros::Publisher pub_roll_servo_;
    ros::Publisher pub_pitch_servo_;
    ros::Publisher pub_fore_servo_;
    ros::Publisher pub_rear_servo_;
    ros::Publisher pub_steer_r_servo_;
    ros::Publisher pub_steer_l_servo_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};

#endif //IMU_TO_DEG_H
