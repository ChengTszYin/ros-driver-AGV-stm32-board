#include "serialstm.h"

SerialSTM::SerialSTM(string port, int baud) : port(port), baud(baud)
{   
    ser.setPort(port);
    ser.setBaudrate(baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    try
    {
        ser.open();
    }

    catch(const std::exception& e )
    {
        cout << "Unable to open port: " << e.what() << endl;
        throw;
    }
    ser_pub = n_ser.advertise<geometry_msgs::Vector3Stamped> ("speed", 1000);
    front_pub = n_ser.advertise<sensor_msgs::Range> ("front_dist", 1000);
    back_pub = n_ser.advertise<sensor_msgs::Range> ("back_dist", 1000);
    imu_pub = n_ser.advertise<sensor_msgs::Imu> ("imu", 1000);
    wsad_pub = n_ser.advertise<std_msgs::Int8MultiArray> ("WSAD", sizeof(wsad));
}

uint8_t SerialSTM::getcrc(uint8_t* Bytecode, int len)
{
    uint8_t sum = 0;
    for(int i=0; i<len; i++)
    {
        sum += Bytecode[i];
    }
    return sum;
}

int SerialSTM::serial_read(std::string &result)
{
    if(!ser.isOpen())
    {
        cout << "serial port is not opened" << endl;
        return 0;
    }
    result = ser.read( ser.available() );
    return 0;
}


void SerialSTM::readSpeed(recvMessage* recvmsg, uint8_t* bufferArray)
{   
    if(!ser.isOpen())
    {
        cout << "serial port is not opened" << endl; 
        return;
    }
    recvmsg->leftID = bufferArray[1];
    recvmsg->leftspeed = (int16_t)((bufferArray[2] << 8) | (bufferArray[3] & 0xFF));
    recvmsg->rightID = bufferArray[4] & 0xFF;
    recvmsg->rightspeed = (int16_t)((bufferArray[5] << 8) | (bufferArray[6] & 0xFF));
    recvmsg->acc_x = ((bufferArray[7] << 8) &  0xFF) | (bufferArray[8] & 0xFF);
    recvmsg->acc_y = ((bufferArray[9] << 8) &  0xFF) | (bufferArray[10] & 0xFF);
    recvmsg->acc_z = ((bufferArray[11] << 8) &  0xFF) | (bufferArray[12] & 0xFF);
    recvmsg->gyro_x = ((bufferArray[13] << 8) &  0xFF) | (bufferArray[14] & 0xFF);
    recvmsg->gyro_y = ((bufferArray[15] << 8) &  0xFF) | (bufferArray[16] & 0xFF);
    recvmsg->gyro_z = ((bufferArray[17] << 8) &  0xFF) | (bufferArray[18] & 0xFF);
    recvmsg->sensor1 = ((bufferArray[19] << 8) & 0xFF) | (bufferArray[20] & 0xFF);
    recvmsg->sensor2 = ((bufferArray[21] << 8) & 0xFF) | (bufferArray[22] & 0xFF);
    recvmsg->d80nk1 = (bufferArray[23]);
    recvmsg->d80nk2 = (bufferArray[24]);
    recvmsg->d80nk3 = (bufferArray[25]);
    recvmsg->d80nk4 = (bufferArray[26]);

    speed_msgs.header.stamp = ros::Time::now();
    int l_speed = recvmsg -> leftspeed;
    int r_speed = recvmsg -> rightspeed;
    speed_msgs.vector.x = (l_speed/myrobot.wheelRadius) * (60/M_PI);
    speed_msgs.vector.y = (r_speed/myrobot.wheelRadius) * (60/M_PI);
    ser_pub.publish(speed_msgs);

    front_dist.min_range = 20.0;
    front_dist.max_range = 720.0;
    front_dist.range = recvmsg -> sensor1;
    front_dist.header.stamp = ros::Time::now();
    front_pub.publish(front_dist);

    back_dist.min_range = 20.0;
    back_dist.max_range = 720.0;
    back_dist.range = recvmsg -> sensor2;
    back_dist.header.stamp = ros::Time::now();
    back_pub.publish(back_dist);

    imu_msgs.linear_acceleration.x = recvmsg -> acc_x;
    imu_msgs.linear_acceleration.y = recvmsg -> acc_y;
    imu_msgs.linear_acceleration.z = recvmsg -> acc_z;
    imu_msgs.angular_velocity.x = recvmsg -> gyro_x;
    imu_msgs.angular_velocity.y = recvmsg -> gyro_y;
    imu_msgs.angular_velocity.z = recvmsg -> gyro_z;
    imu_msgs.header.stamp = ros::Time::now();
    imu_pub.publish(imu_msgs);

    wsad.data = {recvmsg->d80nk1 - 48, recvmsg->d80nk2 - 48, recvmsg->d80nk3 - 48, recvmsg->d80nk4 - 48};
    wsad_pub.publish(wsad);
}

void SerialSTM::putSpeed(Hostmessage* hostmsg)
{
    if(!ser.isOpen())
    {
        cout << "serial port is not opened" << endl;
        return;
    }
    uint8_t sendByte[8];
    sendByte[0] = 0x00;
    sendByte[1] = hostmsg->leftID;
    sendByte[2] = (hostmsg->leftspeed >> 8) & 0xFF;
    sendByte[3] = hostmsg->leftspeed & 0xFF;
    sendByte[4] = hostmsg->rightID;
    sendByte[5] = (hostmsg->rightspeed >> 8) & 0xFF;
    sendByte[6] = hostmsg->rightspeed & 0xFF;
    sendByte[7] = getcrc(sendByte, 7);
    ser.write(sendByte, sizeof(sendByte));
}

