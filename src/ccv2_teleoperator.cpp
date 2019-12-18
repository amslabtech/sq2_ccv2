#include "sq2_ccv2/ccv2_teleoperator.h"

CCV2Teleoperator::CCV2Teleoperator(void)
:local_nh("~")
{
    std::cout << "=== ccv2_teleoperator ===" << std::endl;
}

void CCV2Teleoperator::process(void)
{
    ros::spin();
}

void CCV2Teleoperator::joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
    YPSpurWrapper::VelocityData v;
}
