#ifndef SERIALSTM_H
#define SERIALSTM_H
#include<iostream>
#include <serial/serial.h>
#include "my_board_debug.h"
using namespace std;

struct Hostmessage 
{
    uint8_t leftID = 0x01;
    uint8_t rightID = 0x02;
    int leftspeed = 50;
    int rightspeed = 50;
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
    int16_t sensor3;
    int16_t sensor4;
};

class SerialSTM {
    private:
        string port = "COM4";
        int baud = 115200;
        serial::Serial ser;

    public:
        SerialSTM(string port, int baud);
        void readSpeed(recvMessage* recvmsg, uint8_t* bufferArray);
        void putSpeed(Hostmessage* hostmsg);
        uint8_t getcrc(uint8_t* Bytecode, int len);
        int serial_read(std::string &result);

};



#endif

