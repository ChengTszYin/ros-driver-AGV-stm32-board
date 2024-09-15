#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
using namespace std;


double speed_act_left = 0.0;
double speed_act_right = 0.0;
double speed_dt = 0.0;
ros::Time speed_time(0.0);

void handle_speed(const geometry_msgs::Vector3Stamped& speed)
{
    speed_act_left = trunc(speed.vector.x * 100)/100;
    speed_act_right = trunc(speed.vector.y * 100)/100;
    speed_dt = speed.vector.z;
    speed_time = speed.header.stamp;
    ROS_INFO("speed left : %f", speed_act_left);
    ROS_INFO("speed right : %f", speed_act_right);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("speed", 50, handle_speed);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 
    tf::TransformBroadcaster broadcaster;
    char base_link[] = "/base_link";
    char odom[] = "/odom";

    while(ros::ok())
    {
        ros::spinOnce();
    }
}