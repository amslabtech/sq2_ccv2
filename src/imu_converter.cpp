#include "sq2_ccv2/imu_converter.h"
#include "imu_structure.hpp"

ImuStructure* data = NULL;

void on_connect(struct mosquitto *mosq, void *obj, int result)
{
    std::cout << __FUNCTION__ << std::endl;
    mosquitto_subscribe(mosq, NULL, imu::topic, 0);
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
        data = (ImuStructure*)message->payload;
        data->print9();
    }else{
        printf("%s (null)\n", message->topic);
        std::cout << "payloadlen is 0" << std::endl;
    }
}

IMUConverter::IMUConverter(void)
:local_nh("~")
{
    std::cout << "=== imu_converter ===" << std::endl;

    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 1);

    local_nh.param<double>("HZ", HZ, {200});
    local_nh.param<std::string>("FRAME_ID", FRAME_ID, {"imu"});

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "FRAME_ID: " << FRAME_ID << std::endl;

    mosq = NULL;
}

void IMUConverter::process(void)
{
    char *id = "imu_converter";
    // char *host = "localhost";
    char *host = "192.168.0.34";
    // char *host = "192.168.0.172";
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
        std::cout << "=== imu_converter ===" << std::endl;
        if(data != NULL){
            sensor_msgs::Imu imu;
            imu.header.frame_id = FRAME_ID;
            imu.header.stamp = ros::Time::now();
            static uint32_t seq = 0;
            imu.header.seq = seq;
            seq++;
            // imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1*data->fusion[0], data->fusion[1], data->fusion[2]);
            imu.orientation.x = data->quaternion[0];
            imu.orientation.y = data->quaternion[1];
            imu.orientation.z = data->quaternion[2];
            imu.orientation.w = data->quaternion[3];
            imu.angular_velocity.x = data->gyro[0];
            imu.angular_velocity.y = data->gyro[1];
            imu.angular_velocity.z = -1*data->gyro[2];
            imu.linear_acceleration.x = data->accel[0];
            imu.linear_acceleration.y = data->accel[1];
            imu.linear_acceleration.z = -1*data->accel[2];
            imu_pub.publish(imu);
            std::cout << imu << std::endl;
            // std::cout<<"R,P,Y "<<data->fusion[0]<<","<<data->fusion[1]<<","<<data->fusion[2]<<std::endl;

            data = NULL;
        }else{
            std::cout << "no imu data" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
}
