#include "sq2_ccv2/ccv2_teleoperator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ccv2_teleoperator");
    CCV2Teleoperator ccv2_teleoperator;
    ccv2_teleoperator.process();
    return 0;
}
