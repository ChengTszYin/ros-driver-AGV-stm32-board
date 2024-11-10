#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <stm_driver/Wheel.h>
#include <cmath>
using namespace std;

double wheelDia = 100.7;
double wheelBase = 240.0;
double Track = 280.0;

double speed_act_upper_left = 0.0;
double speed_act_upper_right = 0.0;
double speed_act_lower_left = 0.0;
double speed_act_lower_right = 0.0;
double speed_act_left = 0.0;
double speed_act_right = 0.0;
double speed_dt = 0.0;
double two_pi = 6.28319;

double dx_pos_ = 0.0;
double dy_pos_ = 0.0;
double dheading_ = 0.0;
double linear_velocity_x_ = 0.0;
double linear_velocity_y_ = 0.0;
double angular_velocity_z_ = 0.0;
double vel_dt_ = 0.0;

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
double wheel_cir = (wheelDia * M_PI) / 60;
ros::Time current_time;
ros::Time speed_time(0.0);

// void handle_speed(const stm_driver::Wheel speed)
// {
//     speed_act_upper_left = speed.TopLeftWheel;
//     speed_act_upper_right = speed.TopRightWheel;
//     speed_act_lower_left = speed.BottomLeftWheel;
//     speed_act_lower_right = speed.BottomRightWheel;
//     speed_dt = speed.time;
//     speed_time = speed.header.stamp;
//     speed_act_left = (speed_act_upper_left + speed_act_lower_left) * wheel_cir/ 2;
//     speed_act_right = -1*(speed_act_upper_right + speed_act_lower_right) * wheel_cir/ 2;
// }

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
    dx_pos_ = delta_x;
    dy_pos_ = delta_y;
    dheading_ += delta_heading;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    n.param<double>("wheelDia", wheelDia, 0.01007);
    n.param<double>("wheelBase", wheelBase, 0.240);
    n.param<double>("Track", Track, 0.280);
    // ros::Subscriber sub = n.subscribe("speed", 50, handle_speed);
    ros::Subscriber sub = n.subscribe("velocity", 50, handle_speed);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 
    tf::TransformBroadcaster broadcaster;
    double rate = 100;
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
    // nh_private_.getParam("publish_rate", rate);
    nh_private_.getParam("publish_tf", publish_tf);
    nh_private_.getParam("linear_scale_positive", linear_scale_positive);
    nh_private_.getParam("linear_scale_negative", linear_scale_negative);
    nh_private_.getParam("angular_scale_positive", angular_scale_positive);
    nh_private_.getParam("angular_scale_negative", angular_scale_negative);

    ros::Rate r(rate);
    while(ros::ok())
    {
        ros::spinOnce();

        current_time = speed_time;
        // dt = speed_dt;					//Time in s
        // ROS_INFO("dt : %f", dt);
        // dxy = (speed_act_left+speed_act_right)*dt/2;
        // ROS_INFO("dxy : %f", dxy);
        // dth = ((speed_act_right-speed_act_left)*dt)/Track;
        // ROS_INFO("dth : %f", dth);
        // if (dth > 0) dth *= angular_scale_positive;
        // if (dth < 0) dth *= angular_scale_negative;
        // if (dxy > 0) dxy *= linear_scale_positive;
        // if (dxy < 0) dxy *= linear_scale_negative;

        // dx = cos(dth) * dxy;
        // dy = sin(dth) * dxy;

        // x_pos += (cos(theta) * dx - sin(theta) * dy);
        // y_pos += (sin(theta) * dx + cos(theta) * dy);
        // theta += dth;
        x_pos += dx_pos_ * 100;
        y_pos += dy_pos_ * 100;
        theta = dheading_ * 100;
        ROS_INFO("x_pos : %f", x_pos);
        ROS_INFO("y_pos : %f", y_pos);
        ROS_INFO("theta : %f", theta);

        if(theta >= two_pi) theta -= two_pi;
        if(theta <= -two_pi) theta += two_pi;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        
        geometry_msgs::TransformStamped t;
        
        t.header.frame_id = odom;
        t.child_frame_id = base_link;
        t.transform.translation.x = x_pos;
        t.transform.translation.y = y_pos;
        t.transform.translation.z = 0.0;
        t.transform.rotation = odom_quat;
        t.header.stamp = current_time;
        broadcaster.sendTransform(t);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = odom;
        odom_msg.child_frame_id = base_link;
        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;
        // if (speed_act_left == 0 && speed_act_right == 0){
        // odom_msg.pose.covariance[0] = 1e-9;
        // odom_msg.pose.covariance[7] = 1e-3;
        // odom_msg.pose.covariance[8] = 1e-9;
        // odom_msg.pose.covariance[14] = 1e6;
        // odom_msg.pose.covariance[21] = 1e6;
        // odom_msg.pose.covariance[28] = 1e6;
        // odom_msg.pose.covariance[35] = 1e-9;
        // odom_msg.twist.covariance[0] = 1e-9;
        // odom_msg.twist.covariance[7] = 1e-3;
        // odom_msg.twist.covariance[8] = 1e-9;
        // odom_msg.twist.covariance[14] = 1e6;
        // odom_msg.twist.covariance[21] = 1e6;
        // odom_msg.twist.covariance[28] = 1e6;
        // odom_msg.twist.covariance[35] = 1e-9;
        // }
        // else{
        // odom_msg.pose.covariance[0] = 1e-3;
        // odom_msg.pose.covariance[7] = 1e-3;
        // odom_msg.pose.covariance[8] = 0.0;
        // odom_msg.pose.covariance[14] = 1e6;
        // odom_msg.pose.covariance[21] = 1e6;
        // odom_msg.pose.covariance[28] = 1e6;
        // odom_msg.pose.covariance[35] = 1e3;
        // odom_msg.twist.covariance[0] = 1e-3;
        // odom_msg.twist.covariance[7] = 1e-3;
        // odom_msg.twist.covariance[8] = 0.0;
        // odom_msg.twist.covariance[14] = 1e6;
        // odom_msg.twist.covariance[21] = 1e6;
        // odom_msg.twist.covariance[28] = 1e6;
        // odom_msg.twist.covariance[35] = 1e3;
        // }
        // vx = (dt == 0)?  0 : (speed_act_left+speed_act_right)/2;
        // vth = (dt == 0)? 0 : (speed_act_right-speed_act_left)/Track;
        odom_msg.twist.twist.linear.x = linear_velocity_x_;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_velocity_z_;

        odom_pub.publish(odom_msg);
        r.sleep();
    }
}