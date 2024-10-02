#pragma once

#include <ros/ros.h>
#include <signal.h>

namespace gazebo
{
    class WorldMake
    {
        public:
            WorldMake();
            ~WorldMake();
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Timer timer_;
            static void sigintHandler(int sig);
            void timerCallback(const ros::TimerEvent& event);
    };
}