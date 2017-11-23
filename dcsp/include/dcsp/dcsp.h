
#ifndef _DCSP_
#define _DCSP_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <dcsp/dcsp_srv.h>
#include <dcsp/customPoint.h>
#include <dcsp/agentViewElement.h>
#include <dcsp/dcsp_msg.h>

#include <list>
#include <vector>
#include <math.h>
#include <sstream>
#include <ctime>
#include <limits>

namespace dcsp_exploration
{

class Agent
{
    struct point
    {
        double x;
        double y;
        double z;
        double yaw;
    };

    struct agent_view_element
    {
        std::string agent_identifier;
        point value;
    };

    /* TEMP struct defintions in place of service request and response */
    struct req
    {
        std::string init_agent_identifier;
        point init_point;
        std::vector<std::string> agents;
        std::vector<point> my_domain;
        std::string ok_agent_identifier;
        point ok_value;
        std::vector<agent_view_element> no_goods;
        bool init;
        bool ok;
        bool nogood;
    };

    struct res
    {
        double x;
        double y;
        double z;
    };

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher dcsp_pub_1;
    ros::Publisher dcsp_pub_2;
    ros::Publisher dcsp_pub_3;

    ros::Subscriber listen_topic;

    ros::ServiceServer dcsp_service;

    std::string server_name;
    std::string srv_time_out;
    std::string my_msg_name;
    std::string my_srv_name;
    
    std::vector<std::string> higher_priority_agents;
    std::vector<std::string> lower_priority_agents;

    std::string my_agent_identifier;

    point my_current_point = {0.0, 0.0, 0.0, 0.0};

    std::vector<point> my_domain;
    std::vector<point> my_point_domain; //temporary because we have to change the my_domain to be of point type vector
    
    std::vector<agent_view_element> agent_view;
    std::vector<agent_view_element> no_goods_list;
    
    bool is_agent_initialized = false;

  public:
    Agent(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    virtual ~Agent();

    void agent_init(std::vector<std::string> &agents, const std::string &init_agent_identifier, const point &init_value, const std::vector<point> &init_my_domain);
    
    void recieve_ok_msg(const std::string &agent_identifier, const point &value);
    void receive_nogood_msg(const std::vector<agent_view_element> &no_goods);
    
    void send_ok_msg(Agent &agent);
    void send_nogood_msg(const std::vector<agent_view_element> &no_goods, const std::string lowest_priority_agent_identifier);
    
    void check_agent_view();
    bool check_consistency();
    bool check_compatibility();
    
    bool same_points(const point &point1, const point &point2);
    
    void backtrack();
    void select_best_point();

    void listenerCallback(const dcsp::dcsp_msg &req);
    bool dcspServiceCallback(dcsp::dcsp_srv::Request &req, dcsp::dcsp_srv::Response &res);

    void dcsp_msg_2_recieve_ok();
    void dcsp_msg_3_recieve_ok();
};
}

#endif
