#include "world_make/world_make.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "world_make");
    gazebo::WorldMake world_make;
    ros::spin();
    return 0;
};
