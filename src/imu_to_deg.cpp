#include<battery_box_tf_broadcaster/imu_to_deg.h>

ImuToDeg::ImuToDeg(): private_nh_("~"), nh_(""), tfListener(tfBuffer)
{
    private_nh_.param("hz", hz_, {100});

    sub_imu_ = nh_.subscribe("/imu/data", 10 ,&ImuToDeg::imu_callback, this, ros::TransportHints().tcpNoDelay());
    sub_dxl_state_ = nh_.subscribe("/ccv_dynamixel_controller/dynamixel_state", 10, &ImuToDeg::dxl_state_callback, this, ros::TransportHints().tcpNoDelay());

    pub_roll_ = nh_.advertise<std_msgs::Float32>("/imu_vis_state/roll", 1);
    pub_pitch_ = nh_.advertise<std_msgs::Float32>("/imu_vis_state/pitch", 1);
    pub_yaw_ = nh_.advertise<std_msgs::Float32>("/imu_vis_state/yaw", 1);
    pub_roll_servo_ = nh_.advertise<std_msgs::Float32>("/servo_vis_state/roll", 1);
    pub_pitch_servo_ = nh_.advertise<std_msgs::Float32>("/servo_vis_state/pitch", 1);
    pub_fore_servo_ = nh_.advertise<std_msgs::Float32>("/servo_vis_state/fore", 1);
    pub_rear_servo_ = nh_.advertise<std_msgs::Float32>("/servo_vis_state/rear", 1);
    pub_steer_r_servo_ = nh_.advertise<std_msgs::Float32>("/servo_vis_state/steer_r", 1);
    pub_steer_l_servo_ = nh_.advertise<std_msgs::Float32>("/servo_vis_state/steer_l", 1);
}

ImuToDeg::~ImuToDeg(){}

void ImuToDeg::dxl_state_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg)
{
    const double RP_COEFFICIENT = 180.0/250961;// number for Fore Rear Roll
    const double STEER_COEFFICIENT = 180.0/151875;// number for SeerR,L
    double steer_r,steer_l,fore,rear,roll;
    for(const auto &s:msg->dynamixel_state)
    {
        if(s.name=="SteerR") steer_r = s.present_position*STEER_COEFFICIENT;
        if(s.name=="SteerL") steer_l = s.present_position*STEER_COEFFICIENT;
        if(s.name=="Fore") fore = s.present_position*RP_COEFFICIENT;
        if(s.name=="Rear") rear = s.present_position*RP_COEFFICIENT;
        if(s.name=="Roll") roll = s.present_position*RP_COEFFICIENT;
    }

    // std::cout<<"steer L,R "<<steer_l<<", "<<steer_r<<std::endl;
    // std::cout<<"roll,fore,rear "<<roll<<", "<<fore<<", "<<rear<<std::endl;
    std_msgs::Float32 theta;
    theta.data = roll;
    pub_roll_servo_.publish(theta);
    theta.data = fore;
    pub_fore_servo_.publish(theta);
    theta.data = rear;
    pub_rear_servo_.publish(theta);
    theta.data = steer_l;
    pub_steer_l_servo_.publish(theta);
    theta.data = steer_r;
    pub_steer_r_servo_.publish(theta);


}

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
    ROS_INFO("quaternion_norm: %lf\n",sqrt(msg->orientation.x*msg->orientation.x+msg->orientation.y*msg->orientation.y+msg->orientation.z*msg->orientation.z+msg->orientation.w*msg->orientation.w ));
    // ROS_INFO("quaternion: %lf %lf %lf %lf", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    std_msgs::Float32 theta;
    theta.data = roll/M_PI*180;
    pub_roll_.publish(theta);
    theta.data = pitch/M_PI*180;
    pub_pitch_.publish(theta);
    theta.data = yaw/M_PI*180;
    pub_yaw_.publish(theta);
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
