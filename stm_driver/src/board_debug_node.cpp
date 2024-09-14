#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include "my_board_debug.h"
#include "config_robot.h"
#include "serialstm.h"
#include <std_msgs/String.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv,"my_serial_port");
    ros::NodeHandle n;

    

    SerialSTM serial("/dev/ttyUSB0", 115200);
    recvMessage recv;
    Hostmessage hostmsg;
    hostmsg.leftspeed = 10;
    hostmsg.rightspeed = 10;
    robot myrobot;
    ros::Rate loop_rate(50);
	
	//data 为发送数据
    //result 为接收数据
  	std::string data, result;

    while( ros::ok() )
    {
        serial.putSpeed(&hostmsg);
        //cout << "sent" << endl;

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
        cout << "lID: " << " " << recv.leftID << endl;
        cout << "rID: " << " " << recv.rightID << endl;
        cout << "Lspeed: " << " " << recv.leftspeed << endl;
        cout << "Rspeed: " << " " << recv.rightspeed << endl;
        cout << "accel_x: " << " " << recv.acc_x << endl;
        cout << "accel_y: " << " " << recv.acc_y << endl;
        cout << "accel_z: " << " " << recv.acc_z << endl;
        cout << "gyro_x: " << " " << recv.gyro_x << endl;
        cout << "gyro_y: " << " " << recv.gyro_y << endl;
        cout << "gyro_z: " << " " << recv.gyro_z << endl;
        cout << "sensor1: " << " " << recv.sensor1 << endl;
        cout << "sensor2: " << " " << recv.sensor2 << endl;
        cout << "d80nk1: " << " " << recv.d80nk1 << endl;
        cout << "d80nk2: " << " " << recv.d80nk2 << endl;
        cout << "d80nk3: " << " " << recv.d80nk3 << endl;
        cout << "d80nk4: " << " " << recv.d80nk4 << endl;
    }
    return 0;
}
