#include "sq2_ccv2/ccv2_teleoperator.h"

void on_connect(struct mosquitto *mosq, void *obj, int result)
{
    std::cout << __FUNCTION__ << std::endl;
    mosquitto_subscribe(mosq, NULL, "cmd_vel", 0);
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
        YPSpurWrapper::VelocityData* vd;
        vd = (YPSpurWrapper::VelocityData*)message->payload;
        vd->print_data();
    }else{
        printf("%s (null)\n", message->topic);
        std::cout << "payloadlen is 0" << std::endl;
    }
}

CCV2Teleoperator::CCV2Teleoperator(void)
:local_nh("~")
{
    std::cout << "=== ccv2_teleoperator ===" << std::endl;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub = nh.subscribe("joy", 1, &CCV2Teleoperator::joy_callback, this, ros::TransportHints().tcpNoDelay());
    local_nh.param<double>("MAX_VELOCITY", MAX_VELOCITY, {1.0});
    local_nh.param<double>("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY, {M_PI});
    local_nh.param<double>("MAX_STEERING_ANGLE", MAX_STEERING_ANGLE, {M_PI / 12.0});

    mosq = NULL;
}

void CCV2Teleoperator::process(void)
{
    char *id = "ccv2_teleoperator";
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

    ros::spin();

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
}

void CCV2Teleoperator::joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
    double v = 0.0;
    double w = 0.0;
    double steering = 0.0;
    if(msg->buttons[L1]){
        v = msg->axes[L_STICK_V] * MAX_VELOCITY;
        w = msg->axes[L_STICK_H] * MAX_ANGULAR_VELOCITY;
        if(msg->buttons[L2] && !msg->buttons[R2]){
            w = 0.0;
            steering = MAX_STEERING_ANGLE * (1.0 - (msg->axes[L2_STICK] + 1.0) * 0.5);
        }else if(msg->buttons[R2] && !msg->buttons[L2]){
            w = 0.0;
            steering = -MAX_STEERING_ANGLE * (1.0 - (msg->axes[R2_STICK] + 1.0) * 0.5);
        }else if(msg->buttons[R2] && msg->buttons[L2]){
            std::cout << "brake" << std::endl;
            v = 0.0;
            w = 0.0;
        }
    }else{
        std::cout << "press L1 to move" << std::endl;
    }
    std::cout << "v: " << v << ", " << "w: " << w << ", " << "steering: " << steering << std::endl;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = w;
    cmd_vel_pub.publish(cmd_vel);

    struct timeval ts;
    gettimeofday(&ts, NULL);
    YPSpurWrapper::VelocityData vd;
    vd.sec = ts.tv_sec;
    vd.usec = ts.tv_usec;
    vd.v = v;
    vd.w = w;
    mosquitto_publish(mosq, NULL, "cmd_vel", sizeof(vd), (void*)&vd, 0, 0);
}
