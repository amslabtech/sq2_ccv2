#include "sq2_ccv2/imu_converter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_converter");
    IMUConverter imu_converter;
    imu_converter.process();
    return 0;
}
