#ifndef SERIALSTM_H
#define SERIALSTM_H
#include<iostream>
#include <serial/serial.h>
#include "my_board_debug.h"
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h> 
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8MultiArray.h>
using namespace std;

struct Hostmessage 
{
    uint8_t leftID = 0x01;
    uint8_t rightID = 0x02;
    int leftspeed = 0;
    int rightspeed = 0;
};

struct recvMessage
{
    uint8_t leftID = 0x01;
    uint8_t rightID = 0x02;
    int16_t leftspeed = 0;
    int16_t rightspeed = 0;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z; 
    int16_t sensor1;
    int16_t sensor2;
    uint8_t d80nk1;
    uint8_t d80nk2;
    uint8_t d80nk3;
    uint8_t d80nk4;
};

class SerialSTM {
    private:
        string port = "COM4";
        int baud = 115200;
        serial::Serial ser;
        ros::NodeHandle n_ser;
        geometry_msgs::Vector3Stamped speed_msgs;
        sensor_msgs::Range front_dist;
        sensor_msgs::Range back_dist;
        sensor_msgs::Imu imu_msgs;
        std_msgs::Int8MultiArray wsad;
        ros::Publisher  ser_pub;
        ros::Publisher  front_pub;
        ros::Publisher  back_pub;
        ros::Publisher  imu_pub;
        ros::Publisher  wsad_pub;

    public:
        SerialSTM(string port, int baud);
        void readSpeed(recvMessage* recvmsg, uint8_t* bufferArray);
        void putSpeed(Hostmessage* hostmsg);
        uint8_t getcrc(uint8_t* Bytecode, int len);
        int serial_read(std::string &result);

};



#endif

