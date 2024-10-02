#include "world_make/world_make.hpp"

namespace gazebo
{

// constructor
WorldMake::WorldMake()
    : nh_(""), private_nh_("~")
{
    // register SIGINT handler
    signal(SIGINT, WorldMake::sigintHandler);
}

// destructor
WorldMake::~WorldMake()
{
    // No Contents
}

// add signal handler of Ctrl+C
void WorldMake::sigintHandler(int sig)
{
    ROS_WARN("Ctrl+C is pressed. kill gazebo process.");
    int result_kill_gazebo = system("killall -9 gzserver gzclient&");
    ros::shutdown();
}

// timer callback
void WorldMake::timerCallback(const ros::TimerEvent& event)
{
    // Not using this timer callback now...
    ROS_INFO("timer callback in world_make node.");
}

} // namespace gazebo