#include<battery_box_tf_broadcaster/imu_to_deg.h>

ImuToDeg::ImuToDeg(): private_nh_("~"), nh_(""), tfListener(tfBuffer)
{
    private_nh_.param("hz", hz_, {100});

    sub_imu_ = nh_.subscribe("/imu/data", 10 ,&ImuToDeg::imu_callback, this, ros::TransportHints().tcpNoDelay());
}

ImuToDeg::~ImuToDeg(){}

void ImuToDeg::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion quat,rotation;
    tf2::convert(msg->orientation, quat);

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("imu", "base_link",
                ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    tf2::convert(transformStamped.transform.rotation, rotation);
    quat = rotation*quat;
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    // if(fabs(roll/M_PI*180)>0.1)
    // {
    ROS_INFO("roll: %lf pitch: %lf yaw: %lf", roll/M_PI*180, pitch/M_PI*180, yaw/M_PI*180);
    ROS_INFO("quaternion: %lf %lf %lf %lf", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // }
}

void ImuToDeg::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_to_deg");
    ImuToDeg imu_to_deg;
    imu_to_deg.process();
    return 0;
}
