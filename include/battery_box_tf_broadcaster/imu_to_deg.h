#ifndef IMU_TO_DEG_H
#define IMU_TO_DEG_H

#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
class ImuToDeg
{
public:
    ImuToDeg();
    ~ImuToDeg();
    void process();

private:
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);

    int hz_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_imu_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};

#endif //IMU_TO_DEG_H
