#include "sq2_ccv2/ccv2_teleoperator.h"
#include "ccv_servo_structure.hpp"

void on_connect(struct mosquitto *mosq, void *obj, int result)
{
    // std::cout << __FUNCTION__ << std::endl;
    mosquitto_subscribe(mosq, NULL, "cmd_vel", 0);
}

void on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
    // std::cout << "\033[31mdisconnected from broker!!!\033[0m" << std::endl;
    mosquitto_loop_stop(mosq, true);
}

void on_publish(struct mosquitto *mosq, void *userdata, int mid)
{
    // std::cout << __FUNCTION__ << std::endl;
}

void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
    // std::cout << __FUNCTION__ << std::endl;
    if(message->payloadlen){
        // std::cout << message->topic << std::endl;
        YPSpurWrapper::VelocityData* vd;
        vd = (YPSpurWrapper::VelocityData*)message->payload;
        // vd->print_data();
    }else{
        // printf("%s (null)\n", message->topic);
        // std::cout << "payloadlen is 0" << std::endl;
    }
}

CCV2Teleoperator::CCV2Teleoperator(void)
:local_nh("~")
{
    // std::cout << "=== ccv2_teleoperator ===" << std::endl;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub = nh.subscribe("joy", 1, &CCV2Teleoperator::joy_callback, this, ros::TransportHints().tcpNoDelay());
    local_nh.param<double>("MAX_VELOCITY", MAX_VELOCITY, {1.5});
    local_nh.param<double>("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY, {M_PI});
    local_nh.param<double>("MAX_STEERING_ANGLE", MAX_STEERING_ANGLE, {0.418});
    local_nh.param<double>("MAX_PITCH_ANGLE", MAX_PITCH_ANGLE, {M_PI / 12.0});
    local_nh.param<double>("MAX_ROLL_ANGLE", MAX_ROLL_ANGLE, {M_PI / 24.0});
    local_nh.param<double>("PITCH_OFFSET", PITCH_OFFSET, {3.0 * M_PI / 180.0});

    joy_subscribed = false;
    mode = 0;

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
        // fprintf(stderr, "Cannot create mosquitto object\n");
        mosquitto_lib_cleanup();
        return;
    }
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_disconnect_callback_set(mosq, on_disconnect);
    mosquitto_publish_callback_set(mosq, on_publish);
    mosquitto_message_callback_set(mosq, on_message);

    if(mosquitto_connect_bind(mosq, host, port, keepalive, NULL)){
        // fprintf(stderr, "failed to connect broker.\n");
        mosquitto_lib_cleanup();
        return;
    }

    mosquitto_loop_start(mosq);

    ros::Rate loop_rate(50);
    while(ros::ok()){
        if(joy_subscribed){
            double v = 0.0;
            double w = 0.0;
            double steering = 0.0;
            double pitch = 0.0;
            double roll = 0.0;
            if(joy.buttons[CIRCLE]){
                mode = 0;
            }else if(joy.buttons[SQUARE]){
                mode = 1;
            }
            std::cout << "mode: " << mode << std::endl;
            if(joy.buttons[L1]){
                if(mode == 0){
                    v = joy.axes[L_STICK_V] * MAX_VELOCITY;
                    w = joy.axes[L_STICK_H] * MAX_ANGULAR_VELOCITY;
                    double r_h = joy.axes[R_STICK_H];
                    double r_v = joy.axes[R_STICK_V];
                    double stick_angle = 0;
                    if(sqrt(r_h * r_h + r_v * r_v) > 0.5){
                        stick_angle = atan2(r_h, r_v);
                    }
                    // std::cout << joy.axes[R_STICK_H] << ", " << joy.axes[R_STICK_V] << std::endl;
                    // std::cout << stick_angle << std::endl;
                    steering = stick_angle / 2.0;
                }else if(mode == 1){
                    v = joy.axes[L_STICK_V] * MAX_VELOCITY;
                    w = joy.axes[L_STICK_H] * MAX_ANGULAR_VELOCITY;
                    if(joy.buttons[L2] && !joy.buttons[R2]){
                        // w = 0.0;
                        steering = MAX_STEERING_ANGLE * (1.0 - (joy.axes[L2_STICK] + 1.0) * 0.5);
                    }else if(joy.buttons[R2] && !joy.buttons[L2]){
                        // w = 0.0;
                        steering = -MAX_STEERING_ANGLE * (1.0 - (joy.axes[R2_STICK] + 1.0) * 0.5);
                    }else if(joy.buttons[R2] && joy.buttons[L2]){
                        // std::cout << "brake" << std::endl;
                        v = 0.0;
                        w = 0.0;
                    }
                    pitch = joy.axes[R_STICK_V] * MAX_PITCH_ANGLE;
                    pitch = std::max(-MAX_PITCH_ANGLE, std::min(MAX_PITCH_ANGLE, pitch));
                    roll = joy.axes[R_STICK_H] * MAX_ROLL_ANGLE;
                    roll = std::max(-MAX_ROLL_ANGLE, std::min(MAX_ROLL_ANGLE, roll));
                }
                steering = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, steering));
            }else{
                // std::cout << "press L1 to move" << std::endl;
            }
            // std::cout << "v: " << v << ", " << "w: " << w << ", " << "steering: " << steering << std::endl;
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

            CcvServoStructure servo_command{0, 0, 0};
            servo_command.command_position[servo::STEER] = -steering;
            servo_command.command_position[servo::FORE] = -pitch + PITCH_OFFSET;
            servo_command.command_position[servo::REAR] = pitch + PITCH_OFFSET;
            servo_command.command_position[servo::ROLL] = -roll;
            mosquitto_publish(mosq, NULL, servo::topic_write, sizeof(servo_command), (void*)&servo_command, 0, 0);
            // std::cout << "servo states: " << std::endl;
            servo_command.print_command();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
}

void CCV2Teleoperator::joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
    joy = *msg;
    joy_subscribed = true;
}
