
#include <dcsp/dcsp.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dcsp_agent");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    dcsp_exploration::Agent dcspAgent(nh, nh_private);

    ROS_INFO("DCSP Agent Started.");
    /*
    double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC; ////////
    ROS_INFO("******************Duration: %f *****************************", duration);
    if(duration >= atof(srv_time_out)){
        ROS_INFO("Publish my topics");
    }
    */
    ros::spin();
    return 0;
}
