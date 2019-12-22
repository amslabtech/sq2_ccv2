#ifndef IMU_CONVERTER
#define IMU_CONVERTER

#include <ctime>
#include <sys/time.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <mosquitto.h>

class IMUConverter
{
public:
    IMUConverter(void);

    void process(void);

private:
    double HZ;
    std::string FRAME_ID;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher imu_pub;

    // MQTT
    struct mosquitto *mosq;
};

#endif// IMU_CONVERTER
