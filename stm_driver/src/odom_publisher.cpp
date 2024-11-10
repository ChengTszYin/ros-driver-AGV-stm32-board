#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cmath>
#include <ros/ros.h>

using namespace std;

double wheelDia = 0.1007; // Convert mm to meters
double wheelBase = 0.240;          // Wheelbase in meters
double Track = 0.280;               // Track width in meters

double dx_pos_ = 0.0;
double dy_pos_ = 0.0;
double dheading_ = 0.0;
double linear_velocity_x_ = 0.0;
double linear_velocity_y_ = 0.0;
double angular_velocity_z_ = 0.0;

double wheel_circumference_ = wheelDia * M_PI;

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
ros::Time current_time;
ros::Time speed_time(0.0);

void handle_speed(const geometry_msgs::Vector3Stamped& vel)
{
    current_time = ros::Time::now();
    double dt = (current_time - speed_time).toSec(); // Get the actual elapsed time

    linear_velocity_x_ = vel.vector.x * (wheelDia* M_PI / 2); // m/s
    linear_velocity_y_ = vel.vector.y; // m/s
    angular_velocity_z_ = (vel.vector.z * wheel_circumference_) / ((wheelBase / 2) + (Track / 2)); // rad/s

    // Calculate deltas based on velocities and time
    double delta_heading = angular_velocity_z_ * dt; // radians
    double delta_x = (linear_velocity_x_ * cos(dheading_) - linear_velocity_y_ * sin(dheading_)) * dt; // m
    double delta_y = (linear_velocity_x_ * sin(dheading_) + linear_velocity_y_ * cos(dheading_)) * dt; // m

    // Update current position of the robot
    dx_pos_ = delta_x;
    dy_pos_ = delta_y;
    dheading_ += delta_heading;

    // Update global position
    x_pos += dx_pos_;
    y_pos += dy_pos_;

    // Normalize theta to the range [-pi, pi]
    if (dheading_ > M_PI) dheading_ -= 2 * M_PI;
    if (dheading_ < -M_PI) dheading_ += 2 * M_PI;

    speed_time = current_time; // Update the last speed time
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");

    // Parameters can be set through launch files or parameter server
    n.param<double>("wheelDia", wheelDia, 0.1007); // Default to 100.7 mm
    n.param<double>("wheelBase", wheelBase, 0.240); // Default to 240 mm
    n.param<double>("Track", Track, 0.280);         // Default to 280 mm

    ros::Subscriber sub = n.subscribe("velocity", 50, handle_speed);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster broadcaster;

    double rate = 100; // Default rate in Hz
    nh_private_.getParam("publish_rate", rate);

    ros::Rate r(rate);
    while (ros::ok())
    {
        ros::spinOnce();

        // Prepare and publish the odometry transformation
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(dheading_);
        geometry_msgs::TransformStamped t;

        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_pos;
        t.transform.translation.y = y_pos;
        t.transform.translation.z = 0.0;
        t.transform.rotation = odom_quat;
        t.header.stamp = current_time;
        broadcaster.sendTransform(t);

        // Prepare the odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        // Fill in the twist data
        odom_msg.twist.twist.linear.x = linear_velocity_x_;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_velocity_z_;

        // Publish the odometry message
        odom_pub.publish(odom_msg);
        r.sleep();
    }

    return 0;
}