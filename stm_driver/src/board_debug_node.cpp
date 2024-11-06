#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include "config_robot.h"
#include "serialstm.h"
#include <geometry_msgs/Twist.h>
using namespace std;

double LOOPTIME = 500;
double speed_req = 0;
double angular_speed_req = 0;
double speed_req_left = 0;
double speed_req_right = 0;
double l_rpm = 0;
double r_rpm = 0;
robot rbot;

void cmd_handle(const geometry_msgs::Twist& cmd_vel)
{
    speed_req = cmd_vel.linear.x;
    angular_speed_req = cmd_vel.angular.z;

    speed_req_left = trunc(speed_req - (angular_speed_req * (rbot.Track / 2)));
    speed_req_right = trunc(speed_req + (angular_speed_req * (rbot.Track / 2)));
    l_rpm = (speed_req_left / (M_PI * rbot.wheelDia)) * 60;
    r_rpm = (speed_req_right / (M_PI * rbot.wheelDia)) * 60;
}

void allTopicPublish(SerialSTM* pb, recvMessage* receive)
{
    pb->speedPublish(receive, LOOPTIME);
    pb->IMUPublish(receive);
    pb->distancePublish(receive);
    pb->bumpPublish(receive);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"my_serial_port");
    ros::NodeHandle n;
    SerialSTM serial("/dev/ttyUSB0", 115200);
    recvMessage recv;
    Hostmessage hostmsg;

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_handle);
  	std::string data, result;
    ros::Rate loop_rate(60);
    ros::Time last_publish_time = ros::Time::now();
    while( ros::ok() )
    {
    
        uint8_t bufferArray[42];
        serial.notopen(result);
        if(result.length()==42){
            for(int i=0;i<42;i++){
                bufferArray[i] = result[i];
            }
        }

        if((ros::Time::now() - last_publish_time).toSec() * 1000 >= LOOPTIME)
        {
            allTopicPublish(&serial, &recv);
            last_publish_time = ros::Time::now();
        }
        
        hostmsg.Hleftspeed = -speed_req_left;
        hostmsg.Lleftspeed = -speed_req_left;
        hostmsg.Hrightspeed = speed_req_right;
        hostmsg.Lrightspeed = speed_req_right;
        ROS_INFO("l_rpm: %f, r_rpm: %f", l_rpm, r_rpm);
        serial.putSpeed(&hostmsg);
        if (rbot.checksum(bufferArray, 42)){
            serial.readSpeed(&recv, bufferArray);
        }
        ROS_INFO("speed_req_left : %f", speed_req_left);
        ROS_INFO("speed_req_right: %f", speed_req_right);
        ROS_INFO("rbot.Track: %f", rbot.Track);
       
        // Output the result
        // cout << "lID: " << " " << recv.HleftID << endl;
        // cout << "rID: " << " " << recv.HrightID << endl;
        // cout << "Hleftspeed: " << " " << recv.Hleftspeed << endl;
        // cout << "Hrightspeed: " << " " << recv.Hrightspeed << endl;
        // cout << "Lleftspeed: " << " " << recv.Lleftspeed << endl;
        // cout << "Lrightspeed: " << " " << recv.Lrightspeed << endl;
        // cout << "accel_x: " << " " << recv.acc_x << endl;
        // cout << "accel_y: " << " " << recv.acc_y << endl;
        // cout << "accel_z: " << " " << recv.acc_z << endl;
        // cout << "gyro_x: " << " " << recv.gyro_x << endl;
        // cout << "gyro_y: " << " " << recv.gyro_y << endl;
        // cout << "gyro_z: " << " " << recv.gyro_z << endl;
        // cout << "q1: " << " " << recv.Q0 << endl;
        // cout << "q2: " << " " << recv.Q1 << endl;
        // cout << "q3: " << " " << recv.Q2 << endl;
        // cout << "q4: " << " " << recv.Q3 << endl;
        // cout << "sensor1: " << " " << recv.sensor1 << endl;
        // cout << "sensor2: " << " " << recv.sensor2 << endl;
        // cout << "d80nk1: " << " " << recv.d80nk1 << endl;
        // cout << "d80nk2: " << " " << recv.d80nk2 << endl;
        // cout << "d80nk3: " << " " << recv.d80nk3 << endl;
        // cout << "d80nk4: " << " " << recv.d80nk4 << endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
