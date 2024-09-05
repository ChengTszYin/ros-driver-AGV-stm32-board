#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include "my_board_debug.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>



using namespace std;

uint8_t lID = 0;
uint8_t rID = 0;
int16_t Lspeed = 0;
int16_t Rspeed = 0;
int16_t accel_x = 0;
int16_t accel_y = 0;
int16_t accel_z = 0;
int16_t gyro_x = 0;
int16_t gyro_y = 0;
int16_t gyro_z = 0;
int16_t sensor1 = 0;
int16_t sensor2 = 0;
int16_t sensor3 = 0;
int16_t sensor4 = 0;

int16_t lspeed = 0;
int16_t rspeed = 0;


int serial_read(serial::Serial &ser, std::string &result)
{
    result = ser.read( ser.available() );
    return 0;
}

uint8_t getcrc(uint8_t* Bytecode, int len)
{
    uint8_t sum = 0;
    for(int i=0; i<len; i++)
    {
        sum += Bytecode[i];
    }
    return sum;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv,"my_serial_port");
    ros::NodeHandle n;

    //create an object of class serial
   serial::Serial ser;

    //初始化串口相关设置
   ser.setPort("/dev/ttyUSB0");         //set com port
   ser.setBaudrate(115200);                //set baudrate
   serial::Timeout to = serial::Timeout::simpleTimeout(1000);           //declare the timeout
   ser.setTimeout(to);                           //set serial timeout

    //打开串口
    try
    {
        ser.open();         //打开串口
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");           
        return -1;
    }

    //check com port open
    if( ser.isOpen() )
    { 
        ROS_INFO_STREAM("Serial Port initialized. \n");         
    }
    else
    {
        return -1;
    }


    ros::Rate loop_rate(50);
	
	//data 为发送数据
    //result 为接收数据
  	std::string data, result;

    while( ros::ok() )
    {
        uint8_t sendByte[8];
        sendByte[0] = 0x00;
        sendByte[1] = 0x01;
        sendByte[2] = (lspeed >> 8) & 0xFF;
        sendByte[3] = lspeed & 0xFF;
        sendByte[4] = 0x02;
        sendByte[5] = (rspeed >> 8) & 0xFF;
        sendByte[6] = rspeed & 0xFF;
        sendByte[7] = getcrc(sendByte, 7);
        ser.write(sendByte, sizeof(sendByte));
        cout << "sendByte: " << sendByte << endl;

        uint8_t bufferArray[28];
        serial_read(ser, result);
        if(result.length()==28){
            for(int i=0;i<28;i++){
                bufferArray[i] = result[i];
            }
        }
        ros::Rate loop_rate(5);
        loop_rate.sleep();
        if (checksum(bufferArray, 28)){
            lID = bufferArray[1] & 0xFF;
            Lspeed = ((bufferArray[2] << 8) & 0xFF) | (bufferArray[3] & 0xFF);
            rID = bufferArray[4] & 0xFF;
            Rspeed = ((bufferArray[5] << 8) & 0xFF) | (bufferArray[6] & 0xFF);
            accel_x = ((bufferArray[7] << 8) & 0xFF) | (bufferArray[8] & 0xFF);
            accel_y = ((bufferArray[9] << 8) & 0xFF) | (bufferArray[10] & 0xFF);
            accel_z = ((bufferArray[11] << 8) & 0xFF) | (bufferArray[12] & 0xFF);
            gyro_x = ((bufferArray[13] << 8) & 0xFF) | (bufferArray[14] & 0xFF);
            gyro_y = ((bufferArray[15] << 8) & 0xFF) | (bufferArray[16] & 0xFF);
            gyro_z = ((bufferArray[17] << 8) & 0xFF) | (bufferArray[18] & 0xFF);
            sensor1 = ((bufferArray[19] << 8) & 0xFF) | (bufferArray[20] & 0xFF);
            sensor2 = ((bufferArray[21] << 8) & 0xFF) | (bufferArray[22] & 0xFF);
            sensor3 = ((bufferArray[23] << 8) & 0xFF) | (bufferArray[24] & 0xFF);
            sensor4 = ((bufferArray[25] << 8) & 0xFF) | (bufferArray[26] & 0xFF);
        }
        // Output the result
        // cout << "lID: " << " " << lID << endl;
        // cout << "rID: " << " " << rID << endl;
        // cout << "Lspeed: " << " " << Lspeed << endl;
        // cout << "Rspeed: " << " " << Rspeed << endl;
        // cout << "accel_x: " << " " << accel_x << endl;
        // cout << "accel_y: " << " " << accel_y << endl;
        // cout << "accel_z: " << " " << accel_z << endl;
        // cout << "gyro_x: " << " " << gyro_x << endl;
        // cout << "gyro_y: " << " " << gyro_y << endl;
        // cout << "gyro_z: " << " " << gyro_z << endl;
        // cout << "sensor1: " << " " << sensor1 << endl;
        // cout << "sensor2: " << " " << sensor2 << endl;
        // cout << "sensor3: " << " " << sensor3 << endl;
        // cout << "sensor4: " << " " << sensor4 << endl;
    }

	ser.close();
    return 0;
}
