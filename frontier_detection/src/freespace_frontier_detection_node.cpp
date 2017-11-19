#include <frontier_detection/freespace_frontier_extractor.h>
#include <frontier_detection/freespace_frontier_representative.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "freespace_frontier_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    octomap_frontiers3d::FreeSpaceFrontierExtractor extractor(nh, nh_private);

    ROS_INFO("Frontier Detection Node Started.");
    ros::spin();
    return 0;
}
