
#include <dcsp/dcsp.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dcsp_agent");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    dcsp_exploration::Agent dcspAgent(nh, nh_private);

    ROS_INFO("DCSP Agent Started.");
    ros::spin();
    return 0;
}
