#include "sq2_ccv2/odom_converter.h"

YPSpurWrapper::OdometryData* od = NULL;

void on_connect(struct mosquitto *mosq, void *obj, int result)
{
    std::cout << __FUNCTION__ << std::endl;
    mosquitto_subscribe(mosq, NULL, "odom", 0);
}

void on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
    std::cout << "\033[31mdisconnected from broker!!!\033[0m" << std::endl;
    mosquitto_loop_stop(mosq, true);
}

void on_publish(struct mosquitto *mosq, void *userdata, int mid)
{
    std::cout << __FUNCTION__ << std::endl;
}

void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
    std::cout << __FUNCTION__ << std::endl;
    if(message->payloadlen){
        std::cout << message->topic << std::endl;
        od = (YPSpurWrapper::OdometryData*)message->payload;
        od->print_data();
    }else{
        printf("%s (null)\n", message->topic);
        std::cout << "payloadlen is 0" << std::endl;
    }
}

OdomConverter::OdomConverter(void)
:local_nh("~")
{
    std::cout << "=== odom_converter ===" << std::endl;

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

    local_nh.param<double>("HZ", HZ, {50});
    local_nh.param<std::string>("FRAME_ID", FRAME_ID, {"odom"});
    local_nh.param<std::string>("CHILD_FRAME_ID", CHILD_FRAME_ID, {"base_link"});

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "FRAME_ID: " << FRAME_ID << std::endl;
    std::cout << "CHILD_FRAME_ID: " << CHILD_FRAME_ID << std::endl;

    mosq = NULL;
}

void OdomConverter::process(void)
{
    char *id = "odom_converter";
    char *host = "localhost";
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    mosquitto_lib_init();
    mosq = mosquitto_new(id, clean_session, NULL);
    if(!mosq){
        fprintf(stderr, "Cannot create mosquitto object\n");
        mosquitto_lib_cleanup();
        return;
    }
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_disconnect_callback_set(mosq, on_disconnect);
    mosquitto_publish_callback_set(mosq, on_publish);
    mosquitto_message_callback_set(mosq, on_message);

    if(mosquitto_connect_bind(mosq, host, port, keepalive, NULL)){
        fprintf(stderr, "failed to connect broker.\n");
        mosquitto_lib_cleanup();
        return;
    }

    mosquitto_loop_start(mosq);

    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        if(od != NULL){
            nav_msgs::Odometry odom;
            odom.header.frame_id = FRAME_ID;
            // double stamp = od->sec + od->usec * 1e-6;
            // odom.header.stamp = ros::Time(stamp);
            odom.header.stamp = ros::Time::now();
            static uint32_t seq = 0;
            odom.header.seq = seq;
            seq++;
            odom.child_frame_id = CHILD_FRAME_ID;
            odom.pose.pose.position.x = od->x;
            odom.pose.pose.position.y = od->y;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(od->yaw);
            odom.twist.twist.linear.x = od->v;
            odom.twist.twist.angular.z = od->w;
            odom_pub.publish(odom);
            std::cout << odom << std::endl;

            tf::Transform odom_to_robot;
            tf::poseMsgToTF(odom.pose.pose, odom_to_robot);
            broadcaster.sendTransform(tf::StampedTransform(odom_to_robot, odom.header.stamp, odom.header.frame_id, odom.child_frame_id));

            od = NULL;
        }else{
            std::cout << "no odom data" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
}
