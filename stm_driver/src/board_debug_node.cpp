#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include "my_board_debug.h"
#include "config_robot.h"
#include "serialstm.h"
#include <geometry_msgs/Twist.h>
using namespace std;

robot myrobot;

double speed_req = 0;
double angular_speed_req = 0;
double speed_req_left = 0;
double speed_req_right = 0;

void cmd_handle(const geometry_msgs::Twist& cmd_vel)
{
    speed_req = cmd_vel.linear.x;
    angular_speed_req = cmd_vel.angular.z;
    speed_req_left = speed_req - angular_speed_req*(myrobot.wheelBase/2);
    speed_req_right = speed_req + angular_speed_req*(myrobot.wheelBase/2);
    // ROS_INFO("speed_req_left : %f", speed_req_left);
    // ROS_INFO("speed_req_right : %f", speed_req_right);
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

    while( ros::ok() )
    {
    
        uint8_t bufferArray[28];
        serial.serial_read(result);
        if(result.length()==28){
            for(int i=0;i<28;i++){
                bufferArray[i] = result[i];
            }
        }
        ros::Rate loop_rate(3);
        serial.putSpeed(&hostmsg);
        if (checksum(bufferArray, 28)){
            serial.readSpeed(&recv, bufferArray);
        }
        hostmsg.leftspeed = speed_req_left * 50;
        hostmsg.rightspeed = speed_req_right * 50;
        ROS_INFO("hostmsg.leftspeed : %f", hostmsg.rightspeed);
        ROS_INFO("hostmsg.rightspeed : %f", hostmsg.rightspeed);
        // Output the result
        // cout << "lID: " << " " << recv.leftID << endl;
        // cout << "rID: " << " " << recv.rightID << endl;
        // cout << "Lspeed: " << " " << recv.leftspeed << endl;
        // cout << "Rspeed: " << " " << recv.rightspeed << endl;
        // cout << "accel_x: " << " " << recv.acc_x << endl;
        // cout << "accel_y: " << " " << recv.acc_y << endl;
        // cout << "accel_z: " << " " << recv.acc_z << endl;
        // cout << "gyro_x: " << " " << recv.gyro_x << endl;
        // cout << "gyro_y: " << " " << recv.gyro_y << endl;
        // cout << "gyro_z: " << " " << recv.gyro_z << endl;
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
