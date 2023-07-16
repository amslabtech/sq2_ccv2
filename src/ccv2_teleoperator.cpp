#include "sq2_ccv2/ccv2_teleoperator.h"
// #include "ccv_servo_structure.hpp"

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
    pub_cmd_pos_ = nh.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("cmd_pos", 1);
    joy_sub = nh.subscribe("joy", 1, &CCV2Teleoperator::joy_callback, this, ros::TransportHints().tcpNoDelay());
    sub_cmd_pos_ = nh.subscribe("/local/cmd_pos", 1, &CCV2Teleoperator::cmd_pos_callback, this, ros::TransportHints().tcpNoDelay());
    sub_cmd_vel_ = nh.subscribe("/local/cmd_vel", 1, &CCV2Teleoperator::cmd_vel_callback, this, ros::TransportHints().tcpNoDelay());

    local_nh.param<double>("MAX_VELOCITY", MAX_VELOCITY, {1.5});
    local_nh.param<double>("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY, {M_PI});
    local_nh.param<double>("MAX_STEERING_ANGLE", MAX_STEERING_ANGLE, {20*M_PI/180});
    local_nh.param<double>("MAX_PITCH_ANGLE", MAX_PITCH_ANGLE, {M_PI / 12.0});
    local_nh.param<double>("MAX_ROLL_ANGLE", MAX_ROLL_ANGLE, {M_PI / 24.0});
    local_nh.param<double>("PITCH_OFFSET", PITCH_OFFSET, {0.0 * M_PI / 180.0});
    local_nh.param<double>("STEER_R_OFFSET", STEER_R_OFFSET, {3.25 * M_PI / 180.0});
    local_nh.param<double>("STEER_L_OFFSET", STEER_L_OFFSET, {0.0 * M_PI / 180.0});
    local_nh.param<double>("TREAD", TREAD, {0.5});

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

    // mosquitto_lib_init();
    // mosq = mosquitto_new(id, clean_session, NULL);
    // if(!mosq){
    //     // fprintf(stderr, "Cannot create mosquitto object\n");
    //     mosquitto_lib_cleanup();
    //     return;
    // }
    // mosquitto_connect_callback_set(mosq, on_connect);
    // mosquitto_disconnect_callback_set(mosq, on_disconnect);
    // mosquitto_publish_callback_set(mosq, on_publish);
    // mosquitto_message_callback_set(mosq, on_message);
    //
    // if(mosquitto_connect_bind(mosq, host, port, keepalive, NULL)){
    //     // fprintf(stderr, "failed to connect broker.\n");
    //     mosquitto_lib_cleanup();
    //     return;
    // }
    //
    // mosquitto_loop_start(mosq);

    ros::Rate loop_rate(50);
    while(ros::ok()){
        if(joy_subscribed){
            double v = 0.0;
            double w = 0.0;
            double direction = 0.0;
            double d_i = 0.0;
            double d_o = 0.0;
            double pitch = 0.0;
            double roll = 0.0;
            if(joy.buttons[CROSS])
            {
                mode = -1;
            }
            else if(joy.buttons[CIRCLE]){
                mode = 0;
            }else if(joy.buttons[SQUARE]){
                mode = 1;
            }
            std::cout << "mode: " << mode << std::endl;
            if(mode == -1)
            {
                v = 0.0;
                w = 0.0;
                d_o = 0.0;
                d_i = 0.0;
            }
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
                    direction = stick_angle / 2.0;
                }else if(mode == 2){
                    v = cmd_vel_.linear.x;
                    w = cmd_vel_.angular.z;

                    // v = joy.axes[L_STICK_V] * MAX_VELOCITY;
                    // w = joy.axes[L_STICK_H] * MAX_ANGULAR_VELOCITY;
                    // if(joy.buttons[L2] && !joy.buttons[R2]){
                    //     // w = 0.0;
                    //     direction = MAX_STEERING_ANGLE * (1.0 - (joy.axes[L2_STICK] + 1.0) * 0.5);
                    // }else if(joy.buttons[R2] && !joy.buttons[L2]){
                    //     // w = 0.0;
                    //     direction = -MAX_STEERING_ANGLE * (1.0 - (joy.axes[R2_STICK] + 1.0) * 0.5);
                    // }else if(joy.buttons[R2] && joy.buttons[L2]){
                    //     // std::cout << "brake" << std::endl;
                    //     v = 0.0;
                    //     w = 0.0;
                    // }
                    // pitch = joy.axes[R_STICK_V] * MAX_PITCH_ANGLE;
                    // pitch = std::max(-MAX_PITCH_ANGLE, std::min(MAX_PITCH_ANGLE, pitch));
                    // roll = joy.axes[R_STICK_H] * MAX_ROLL_ANGLE;
                    // roll = std::max(-MAX_ROLL_ANGLE, std::min(MAX_ROLL_ANGLE, roll));
                }

                // if(mode == 1)
                // {
                //     v = cmd_vel_.linear.x;
                //     w = cmd_vel_.angular.z;
                // }

                direction = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, direction));
                if(abs(w) > 1e-2){
                    double r = abs(v / w);
                    d_i = atan(r * sin(direction) / (r * cos(direction) - TREAD / 2.0));
                    d_o = atan(r * sin(direction) / (r * cos(direction) + TREAD / 2.0));
                    if(w < 0.0){
                        double buf = d_i;
                        d_i = d_o;
                        d_o = buf;
                    }
                }else{
                    d_i = direction;
                    d_o = direction;
                }
            }else{
                if(mode == 1)
                {
                    v = cmd_vel_.linear.x;
                    w = cmd_vel_.angular.z;
                }

                direction = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, direction));
                if(abs(w) > 1e-2){
                    double r = abs(v / w);
                    d_i = atan(r * sin(direction) / (r * cos(direction) - TREAD / 2.0));
                    d_o = atan(r * sin(direction) / (r * cos(direction) + TREAD / 2.0));
                    if(w < 0.0){
                        double buf = d_i;
                        d_i = d_o;
                        d_o = buf;
                    }
                }else{
                    d_i = direction;
                    d_o = direction;
                }
                // std::cout << "press L1 to move" << std::endl;
            }
            std::cout << "v: " << v << ", " << "w: " << w << ", " << "direction: " << direction << std::endl;
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
            // mosquitto_publish(mosq, NULL, "cmd_vel", sizeof(vd), (void*)&vd, 0, 0);

            // CcvServoStructure servo_command{0, 0, 0};
            // servo_command.command_position[servo::STRR] = -d_o;
            // servo_command.command_position[servo::STRL] = -d_i;
            // servo_command.command_position[servo::FORE] = -pitch + PITCH_OFFSET;
            // servo_command.command_position[servo::REAR] = pitch + PITCH_OFFSET;
            // servo_command.command_position[servo::ROLL] = -roll;
            ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos;
            if(mode!=1)
            {
                cmd_pos.steer_r = -d_o+STEER_R_OFFSET;
                cmd_pos.steer_l = -d_i+STEER_L_OFFSET;
            }
            else
            {
                cmd_pos.steer_r = -cmd_pos_.steer_r+STEER_R_OFFSET;
                cmd_pos.steer_l = -cmd_pos_.steer_l+STEER_L_OFFSET;
            }
            cmd_pos.fore = -pitch + PITCH_OFFSET;
            cmd_pos.rear = pitch + PITCH_OFFSET;
            cmd_pos.roll = -roll;
            // mosquitto_publish(mosq, NULL, servo::topic_write, sizeof(servo_command), (void*)&servo_command, 0, 0);
            // std::cout << "servo states: " << std::endl;
            // servo_command.print_command();
            pub_cmd_pos_.publish(cmd_pos);
            // std::cout << "send command of dynamixels:\n" << cmd_pos <<std::endl;
            std::cout<<"steer_l: "<<cmd_pos.steer_l/M_PI*180<<" ,steer_r: "<<cmd_pos.steer_r/M_PI*180<<std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // mosquitto_destroy(mosq);
    // mosquitto_lib_cleanup();
}

void CCV2Teleoperator::joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
    joy = *msg;
    joy_subscribed = true;
}

void CCV2Teleoperator::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel_ = *msg;
}

void CCV2Teleoperator::cmd_pos_callback(const ccv_dynamixel_msgs::CmdPoseByRadian::ConstPtr &msg)
{
    cmd_pos_ = *msg;
}
