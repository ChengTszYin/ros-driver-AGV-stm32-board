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
    recvmsg->leftspeed = ((bufferArray[2] << 8) &  0xFF) | (bufferArray[3] & 0xFF);
    recvmsg->rightID = bufferArray[4] & 0xFF;
    recvmsg->rightspeed = ((bufferArray[5] << 8) &  0xFF) | (bufferArray[6] & 0xFF);
    recvmsg->acc_x = ((bufferArray[7] << 8) &  0xFF) | (bufferArray[8] & 0xFF);
    recvmsg->acc_y = ((bufferArray[9] << 8) &  0xFF) | (bufferArray[10] & 0xFF);
    recvmsg->acc_z = ((bufferArray[11] << 8) &  0xFF) | (bufferArray[12] & 0xFF);
    recvmsg->gyro_x = ((bufferArray[13] << 8) &  0xFF) | (bufferArray[14] & 0xFF);
    recvmsg->gyro_y = ((bufferArray[15] << 8) &  0xFF) | (bufferArray[16] & 0xFF);
    recvmsg->gyro_z = ((bufferArray[17] << 8) &  0xFF) | (bufferArray[18] & 0xFF);
    recvmsg->sensor1 = ((bufferArray[19] << 8) & 0xFF) | (bufferArray[20] & 0xFF);
    recvmsg->sensor2 = ((bufferArray[21] << 8) & 0xFF) | (bufferArray[22] & 0xFF);
    recvmsg->sensor3 = ((bufferArray[23] << 8) & 0xFF) | (bufferArray[24] & 0xFF);
    recvmsg->sensor4 = ((bufferArray[25] << 8) & 0xFF) | (bufferArray[26] & 0xFF);
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

