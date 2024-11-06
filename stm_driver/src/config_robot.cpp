#include "config_robot.h"

uint8_t robot::checksum(uint8_t data[], int len)
{
	int16_t crc = 0;
	for(int i = 0; i < len; i++)
	{
		crc = (crc + data[i]) & 0xFF;
	}
	return crc;
}

robot::robot()
{
   std::ifstream file_("/home/dllm/bme_ws/src/stm_driver/cfg/config.txt"); //absolute directory 
   if(file_.is_open())
   {
      std::vector<std::string> _val;
      std::string lines;
      while (std::getline(file_, lines)) 
      {
         //std::cout << lines << std::endl; // Log each line
         std::istringstream iss(lines);

         std::string key, value;
         if(std::getline(iss, key, '=') && (std::getline(iss, value)))
         {
            //std::cout << "key:" << key << std::endl;
            //std::cout << "value:" << value << std::endl;
            _val.push_back(value);
         }

      }
      file_.close();

      if(_val.size() == 4)
      {
         try
         {
            baseType = _val[0];
            wheelDia = std::stof(_val[1]);
            wheelBase = std::stof(_val[2]);
            Track = std::stof(_val[3]);
            std::cout << baseType << " drive is initiated " << std::endl;
         }

         catch(const std::exception& e)
         {
            std::cout << "error: " << e.what() << std::endl;
         }
      }

   }
   else {
            std::cerr << "Error opening file for reading!" << std::endl;
        }
}

