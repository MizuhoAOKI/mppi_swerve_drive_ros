#include "joy_controller/joy_controller.hpp"

namespace operation
{

//constructor
JoyController::JoyController()
    : nh_(""), private_nh_("~")
{
    // load parameters 

    //// joy stick axes
    private_nh_.param<int>("joy_left_stick_x_idx", joy_left_stick_x_idx, 0);
    private_nh_.param<int>("joy_left_stick_y_idx", joy_left_stick_y_idx, 1);
    private_nh_.param<int>("joy_right_stick_x_idx", joy_right_stick_x_idx, 3);
    private_nh_.param<int>("joy_right_stick_y_idx", joy_right_stick_y_idx, 4);

    //// max command values
    private_nh_.param<float>("abs_max_linear_vel_x", abs_max_linear_vel_x, 3.0f);
    private_nh_.param<float>("abs_max_linear_vel_y", abs_max_linear_vel_y, 3.0f);
    private_nh_.param<float>("abs_max_angular_vel_z", abs_max_angular_vel_z, 1.0f);

    //// subscribing topic names
    std::string joy_topic;
    private_nh_.param<std::string>("joy_topic", joy_topic, "/joy");

    //// publishing topic names
    std::string control_cmd_vel_topic;
    private_nh_.param<std::string>("control_cmd_vel_topic", control_cmd_vel_topic, "/cmd_vel");

    // initialize subscribers
    sub_joy_ = nh_.subscribe(joy_topic, 1, &JoyController::joyCallback, this);

    // initialize publishers
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(control_cmd_vel_topic, 10);
}

// destructor
JoyController::~JoyController()
{
    // No Contents
}

// /joy topic callback
void JoyController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    // parse received joy message
    float val_left_stick_x = -1.0f * msg->axes[joy_left_stick_x_idx];
    float val_left_stick_y = msg->axes[joy_left_stick_y_idx];
    float val_right_stick_x = msg->axes[joy_right_stick_x_idx];
    float val_right_stick_y = msg->axes[joy_right_stick_y_idx];

    // [for debug] annouce received joy message
    ROS_DEBUG("Received Joy Command: Lstick_x = %+5.1f, Lstick_y = %+5.1f, Rstick_x = %+5.1f, Rstick_y = %+5.1f", val_left_stick_x, val_left_stick_y, val_right_stick_x, val_right_stick_y);

    // calculate Twist message to publish
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x =  abs_max_linear_vel_x * val_left_stick_y; // forward : positive, backward : negative
    cmd_vel.linear.y = -abs_max_linear_vel_y * val_left_stick_x; // left    : positive, right    : negative
    cmd_vel.linear.z =  0.0f;
    cmd_vel.angular.x = 0.0f;
    cmd_vel.angular.y = 0.0f;
    cmd_vel.angular.z = abs_max_angular_vel_z * val_right_stick_x; // left    : positive, right    : negative

    // [for debug] annouce publishing Twist message
    ROS_DEBUG("Send Twist Command: linear_x = %+5.1f, linear_y = %+5.1f, angular_z = %+5.1f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // publish Twist message
    pub_cmd_vel_.publish(cmd_vel);
}

} // namespace operation