#include "sq2_ccv2/odom_converter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_converter");
    OdomConverter odom_converter;
    odom_converter.process();
    return 0;
}
