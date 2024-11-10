#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <stm_driver/Wheel.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
using namespace std;
int PUBLISH_RATE = 80;

double wheelDia = 0.1007;
double wheelBase = 0.240;
double Track = 0.280;

double speed_act_upper_left = 0.0;
double speed_act_upper_right = 0.0;
double speed_act_lower_left = 0.0;
double speed_act_lower_right = 0.0;
double speed_act_left = 0.0;
double speed_act_right = 0.0;
double speed_dt = 0.0;
double two_pi = 6.28319;
double dx_pos = 0.0;
double dy_pos_ = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

double wheel_cir = (wheelDia * M_PI) / 60;
double dheading_ = 0.0;
double heading_ = 0.0; 
double linear_velocity_x_ = 0.0;
double linear_velocity_y_ = 0.0;
double angular_velocity_z_ = 0.0;
double vel_dt_ = 0.0;
tf2::Quaternion odom_quat;

void handle_speed(const geometry_msgs::Vector3Stamped vel)
{
    ros::Time current_time = ros::Time::now();
    linear_velocity_x_ = vel.vector.x * (wheelDia* M_PI / 2);
    linear_velocity_y_ = vel.vector.y;
    angular_velocity_z_ = (vel.vector.z * (wheelDia * M_PI / 2)) / ((Track / 2) + ( wheelBase/ 2));
    vel_dt_ = 0.1;
    // ROS_INFO("vel_dt_: %f", vel_dt_);
    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(dheading_) - linear_velocity_y_ * sin(dheading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(dheading_) + linear_velocity_y_ * cos(dheading_)) * vel_dt_; //m

    //calculate current position of the robot
    dx_pos = delta_x;
    dy_pos_ = delta_y;
    dheading_ += delta_heading;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    ros::Subscriber sub = n.subscribe("speed", 50, handle_speed);
    geometry_msgs::TransformStamped t;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 
    tf::TransformBroadcaster broadcaster;
    double rate = PUBLISH_RATE;
    bool publish_tf = true;
    double dt = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double dth = 0.0;
    double dxy = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;
    double linear_scale_positive = 1.0;
    double linear_scale_negative = 1.0;
    double angular_scale_positive = 1.0;
    double angular_scale_negative = 1.0;
    char base_link[] = "/base_link";
    char odom[] = "/odom";
    nh_private_.getParam("publish_rate", rate);
    nh_private_.getParam("publish_tf", publish_tf);
    nh_private_.getParam("linear_scale_positive", linear_scale_positive);
    nh_private_.getParam("linear_scale_negative", linear_scale_negative);
    nh_private_.getParam("angular_scale_positive", angular_scale_positive);
    nh_private_.getParam("angular_scale_negative", angular_scale_negative);

    ros::Rate r(rate);
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        

        x_pos += dx_pos;
        y_pos += dy_pos_;
        heading_ += dheading_;
        // heading_ += delta_heading;

        odom_quat.setRPY(0,0,heading_);

        ROS_INFO("x_pos: %f", x_pos);
        ROS_INFO("y_pos: %f", y_pos);
        ROS_INFO("speed_dt: %f", speed_dt);
        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        // geometry_msgs::Quaternion empty_quat = tf::createQuaternionMsgFromYaw(0);

        t.header.frame_id = odom;
        t.child_frame_id = base_link;
        t.transform.translation.x = x_pos;
        t.transform.translation.y = y_pos;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = odom_quat.x();
        t.transform.rotation.y = odom_quat.y();
        t.transform.rotation.z = odom_quat.z();
        t.transform.rotation.w = odom_quat.w();
        t.header.stamp = current_time;

        broadcaster.sendTransform(t);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();;
        odom_msg.header.frame_id = odom;
        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = odom_quat.x();
        odom_msg.pose.pose.orientation.y = odom_quat.y();
        odom_msg.pose.pose.orientation.z = odom_quat.z();
        odom_msg.pose.pose.orientation.w = odom_quat.w();
        odom_msg.pose.covariance[0] = 0.001;
        odom_msg.pose.covariance[7] = 0.001;
        odom_msg.pose.covariance[35] = 0.001;
        //linear speed from encoders
        odom_msg.twist.twist.linear.x = linear_velocity_x_;
        odom_msg.twist.twist.linear.y = linear_velocity_y_;
        odom_msg.twist.twist.linear.z = 0.0;

        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        //angular speed from encoders
        odom_msg.twist.twist.angular.z = angular_velocity_z_;
        odom_msg.twist.covariance[0] = 0.0001;
        odom_msg.twist.covariance[7] = 0.0001;
        odom_msg.twist.covariance[35] = 0.0001;
    
        odom_pub.publish(odom_msg);
        r.sleep();
    }
}