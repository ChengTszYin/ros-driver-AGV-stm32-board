#ifndef CONFIG_ROBOT_H
#define CONFIG_ROBOT_H
#include <iostream>
#include <sstream>
#include <sstream>
#include <fstream>
#include <vector>

class robot
{        
    public:
        std::string baseType;
        float wheelDia;
        float wheelBase;
        float Track;
        uint8_t checksum(uint8_t data[], int len);
        robot();
};

#endif