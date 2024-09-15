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
        float wheelRadius;
        float wheelBase;
        robot();
};

#endif