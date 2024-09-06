#include <ros/ros.h>
#include <std_msgs/String.h>
#include "my_board_debug.h"
#include "serialstm.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

uint8_t checksum(uint8_t data[], int len)
{
	int16_t crc = 0;
	for(int i = 0; i < len; i++)
	{
		crc = (crc + data[i]) & 0xFF;
	}
	return crc;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"my_serial_port");
    ros::NodeHandle n;

    SerialSTM serial("/dev/ttyUSB0", 115200);
    recvMessage recv;
    Hostmessage hostmsg;
    hostmsg.leftspeed = 0;
    hostmsg.rightspeed = 0;

    ros::Rate loop_rate(50);
	
	//data 为发送数据
    //result 为接收数据
  	std::string data, result;

    while( ros::ok() )
    {
        serial.putSpeed(&hostmsg);
        cout << "sent" << endl;

        uint8_t bufferArray[28];
        serial.serial_read(result);
        if(result.length()==28){
            for(int i=0;i<28;i++){
                bufferArray[i] = result[i];
            }
        }
        ros::Rate loop_rate(5);
        loop_rate.sleep();
        if (checksum(bufferArray, 28)){
            serial.readSpeed(&recv, bufferArray);
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
    return 0;
}
