#ifndef __ODOM_CONVERTER_H
#define __ODOM_CONVERTER_H

#include <ctime>
#include <sys/time.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "ypspur_mqtt/odometry_data.h"

#include <mosquitto.h>

class OdomConverter
{
public:
    OdomConverter(void);

    void process(void);

private:
    double HZ;
    std::string FRAME_ID;
    std::string CHILD_FRAME_ID;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher odom_pub;

    tf::TransformBroadcaster broadcaster;

    // MQTT
    struct mosquitto *mosq;
};

#endif// __ODOM_CONVERTER_H
