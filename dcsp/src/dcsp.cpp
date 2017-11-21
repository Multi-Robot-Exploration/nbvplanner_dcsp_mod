
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <dcsp/dcsp_srv.h>
#include <dcsp/customPoint.h>
#include <dcsp/agentViewElement.h>
#include <dcsp/dcsp_msg.h>
#include <dcsp/dcsp.h>

#include <list>
#include <vector>
#include <math.h>
#include <sstream>
#include <ctime>
#include <limits>

using namespace dcsp_exploration;

Agent::Agent(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh),
                                                                             nh_private_(nh_private)
{

    std::string ns = ros::this_node::getName();

    if (!ros::param::get(ns + "/srv_name", server_name))
    {
        ROS_WARN("No server name specified. Looking for %s.",
                 (ns + "/srv_name").c_str());
    }

    if (server_name == "dcsp_server_1")
    {
        my_msg_name = "dcsp_msg_1";
        my_srv_name = "firefly1/dcsp_srv";
    }
    else if (server_name == "dcsp_server_2")
    {
        my_msg_name = "dcsp_msg_2";
        my_srv_name = "firefly2/dcsp_srv";
    }
    else if (server_name == "dcsp_server_3")
    {
        my_msg_name = "dcsp_msg_3";
        my_srv_name = "firefly3/dcsp_srv";
    }

    ROS_INFO("DCSP Agent started with, name: %s, msgtop: %s, srvtop: %s",
             server_name.c_str(),
             my_msg_name.c_str(),
             my_srv_name.c_str());

    dcsp_pub_1 = nh_.advertise<dcsp::dcsp_msg>("dcsp_msg_1", 10);
    dcsp_pub_2 = nh_.advertise<dcsp::dcsp_msg>("dcsp_msg_2", 10);
    dcsp_pub_3 = nh_.advertise<dcsp::dcsp_msg>("dcsp_msg_3", 10);

    dcsp_service = nh_.advertiseService(my_srv_name, &dcsp_exploration::Agent::dcspServiceCallback, this);

    listen_topic = nh_.subscribe(my_msg_name, 10, &dcsp_exploration::Agent::listenerCallback, this);
}

Agent::~Agent()
{
    //destructor
}

//service callback
bool Agent::dcspServiceCallback(dcsp::dcsp_srv::Request &req, dcsp::dcsp_srv::Response &res)
{
    ROS_INFO("Inside the service callback of dcsp_agent");
    
    ros::Time startTime = ros::Time::now();

    std::vector<std::string> agentsVector;
    for (int i = 0; i < req.agents_names.size(); i++)
    {
        agentsVector.push_back(req.agents_names[i]);
    }

    std::string myName = req.my_name;

    point initialPoint;
    initialPoint.x = req.origin.x;
    initialPoint.y = req.origin.y;
    initialPoint.z = req.origin.z;

    std::vector<point> myDomainVector;
    for (int i = 0; i < req.my_domain.size(); i++)
    {
        point p;
        p.x = req.my_domain[i].x;
        p.y = req.my_domain[i].y;
        p.z = req.my_domain[i].z;

        myDomainVector.push_back(p);
    }

    agent_init(agentsVector, myName, initialPoint, myDomainVector);

    ros::Duration interval = (ros::Time::now() - startTime);
    while( interval >= ros::Duration(3.0)){
        ros::Duration(0.5).sleep();
    }
    // wait before return below
    res.x = my_current_point.x;
    res.y = my_current_point.y;
    res.z = my_current_point.z;

    return true;
}

void Agent::listenerCallback(const dcsp::dcsp_msg &msg)
{
    ROS_INFO("Inside the listener callback of dcsp_agent");

    if (msg.ok)
    {
        std::string ok_agent_identifier_string = msg.ok_agent_identifier;
        point p;
        p.x = msg.ok_value.x;
        p.y = msg.ok_value.y;
        p.z = msg.ok_value.z;
        recieve_ok_msg(ok_agent_identifier_string, p);
    }
    else if (msg.nogood)
    {
        std::vector<agent_view_element> no_goods_vector;
        for (int i = 0; i < msg.no_goods.size(); i++)
        {
            agent_view_element a;
            a.agent_identifier = msg.no_goods[i].agent_identifier;
            point p;
            p.x = msg.no_goods[i].value.x;
            p.y = msg.no_goods[i].value.y;
            p.z = msg.no_goods[i].value.z;

            a.value = p;

            no_goods_vector.push_back(a);
        }
        receive_nogood_msg(no_goods_vector);
    }
}

void Agent::agent_init(std::vector<std::string> &agents, const std::string &init_agent_identifier, const point &init_value, const std::vector<point> &init_my_domain)
{

    my_agent_identifier = init_agent_identifier;
    my_current_point = init_value;
    my_domain = init_my_domain;

    ROS_INFO("DCSP SERVICE 1 INSIDE agent init");

    std::vector<std::string>::iterator it;

    for (it = agents.begin(); it != agents.end(); ++it)
    {
        if (my_agent_identifier.compare(*it) > 0)
        {
            higher_priority_agents.push_back(*it);
        }
        else if (my_agent_identifier.compare(*it) < 0)
        {
            lower_priority_agents.push_back(*it);
        }
    }

    // select the point giving the best euclidean distance
    select_best_point();
    // send ok message to all the elements in the lower_priority_agents

    ROS_INFO("Selected point x value: %f", my_current_point.x);
    ROS_INFO("Selected point y value: %f", my_current_point.y);
    ROS_INFO("Selected point z value: %f", my_current_point.z);
    
    is_agent_initialized = true;

    if (my_agent_identifier == "firefly_1")
    {
        ROS_INFO("Firefly_1 call the service of Firefly_2.");
        dcsp_msg_2_recieve_ok();
        ROS_INFO("Firefly_1 call the service of Firefly_3.");
        dcsp_msg_3_recieve_ok();
    }
    else if (my_agent_identifier == "firefly_2")
    {
        dcsp_msg_3_recieve_ok();
    }
}

void Agent::recieve_ok_msg(const std::string &agent_identifier, const point &value)
{
    ROS_INFO("%s: OK RECEIVED.",my_agent_identifier.c_str());

    bool is_agent_in_agent_view = false;

    std::vector<agent_view_element>::iterator it;

    for (it = agent_view.begin(); it != agent_view.end(); ++it)
    {
        if (it->agent_identifier == agent_identifier)
        {
            is_agent_in_agent_view = true;
        }
    }

    if (!is_agent_in_agent_view)
    {
        agent_view_element new_element;
        new_element.agent_identifier = agent_identifier;
        new_element.value = value;
        agent_view.push_back(new_element);
    }

    check_agent_view();
}

void Agent::receive_nogood_msg(const std::vector<agent_view_element> &no_goods)
{
    ROS_INFO("%s: NOGOOD RECEIVED.",my_agent_identifier.c_str());

    no_goods_list.insert(no_goods_list.end(), no_goods.begin(), no_goods.end());

    check_agent_view();
}

void Agent::check_agent_view()
{
    if (!check_consistency())
    {
        ROS_INFO("%s: Not consistent",my_agent_identifier.c_str());
        bool is_new_value_consistent = false;

        std::vector<point>::iterator it;

        for (it = my_domain.begin(); it != my_domain.end(); ++it)
        {
            if (!same_points(*it, my_current_point))
            {
                std::vector<agent_view_element>::iterator it2;

                for (it2 = agent_view.begin(); it2 != agent_view.end(); ++it2)
                {
                    if (same_points(it2->value, *it))
                    {
                        is_new_value_consistent = false;
                        break;
                    }
                    else
                    {
                        is_new_value_consistent = true;
                    }
                }
            }

            if (is_new_value_consistent)
            {
                ROS_INFO("%s: new consistent value found",my_agent_identifier.c_str());
                my_current_point = *it;

                std::vector<std::string>::iterator it3;

                for (it3 = lower_priority_agents.begin(); it3 != lower_priority_agents.end(); ++it3)
                {
                    std::string lower_priority_agent = *it3;

                    dcsp::dcsp_msg msg;

                    msg.ok_agent_identifier = my_agent_identifier;

                    dcsp::customPoint c;
                    c.x = my_current_point.x;
                    c.y = my_current_point.y;
                    c.z = my_current_point.z;

                    msg.ok_value = c;

                    msg.ok = true;
                    msg.nogood = false;

                    if (lower_priority_agent == "firefly_1")
                    {
                        dcsp_pub_1.publish(msg);
                    }
                    else if (lower_priority_agent == "firefly_2")
                    {
                        dcsp_pub_2.publish(msg);
                    }
                    else if (lower_priority_agent == "firefly_3")
                    {
                        dcsp_pub_3.publish(msg);
                    }
                }
                break;
            }
        }

        if (!is_new_value_consistent)
        {
            backtrack();
        }
    }
    else
    {
        ROS_INFO("%s: value is consistent.",my_agent_identifier.c_str());
    }
}

bool Agent::check_consistency()
{
    bool is_consistent = false;
    bool is_agent_view_consistent = true;

    std::vector<agent_view_element>::iterator it;

    for (it = agent_view.begin(); it != agent_view.end(); ++it)
    {
        if (same_points(it->value, my_current_point))
        {
            is_agent_view_consistent = false;
        }
    }

    bool is_compatible = check_compatibility();

    if (is_agent_view_consistent && (!is_compatible))
    {
        is_consistent = true;
    }
    else
    {
        is_consistent = false;
    }

    return is_consistent;
}

bool Agent::check_compatibility()
{
    bool is_compatible = false;

    std::vector<agent_view_element>::iterator it;

    for (it = no_goods_list.begin(); it != no_goods_list.end(); ++it)
    {
        if (it->agent_identifier == my_agent_identifier)
        {
            if (same_points(it->value, my_current_point))
            {
                is_compatible = true;
                continue;
            }
            else
            {
                is_compatible = false;
                break;
            }
        }

        std::vector<agent_view_element>::iterator agent_view_it;

        for (agent_view_it = agent_view.begin(); agent_view_it != agent_view.end(); ++agent_view_it)
        {
            if (same_points(agent_view_it->value, it->value))
            {
                is_compatible = true;
                continue;
            }
            else
            {
                is_compatible = false;
                break;
            }
        }

        if (is_compatible)
        {
            continue;
        }
        else
        {
            break;
        }
    }
    return is_compatible;
}

void Agent::backtrack()
{
    // Add inconsistent subset of agent_view to a new vector called no_goods
    std::vector<agent_view_element> no_goods;

    std::vector<agent_view_element>::iterator it;

    for (it = agent_view.begin(); it != agent_view.end(); ++it)
    {
        if (same_points(it->value, my_current_point))
        {
            no_goods.push_back(*it);
        }
    }

    if (no_goods.empty())
    {
        ROS_INFO("%s: No Solution found.",my_agent_identifier.c_str());
    }
    else
    {
        agent_view_element first_agent_view_element = no_goods.front();
        std::string lowest_priority_agent_identifier = first_agent_view_element.agent_identifier;

        std::vector<agent_view_element>::iterator it2;

        for (it2 = agent_view.begin(); it2 != agent_view.end(); ++it2)
        {
            if ((lowest_priority_agent_identifier.compare(it2->agent_identifier)) < 0)
            {
                lowest_priority_agent_identifier = it2->agent_identifier;
            }
        }
        send_nogood_msg(no_goods, lowest_priority_agent_identifier);
    }
}

void Agent::send_nogood_msg(const std::vector<agent_view_element> &no_goods, const std::string lowest_priority_agent_identifier)
{
    std::vector<std::string>::iterator it;

    for (it = higher_priority_agents.begin(); it != higher_priority_agents.end(); ++it)
    {
        if (*it == lowest_priority_agent_identifier)
        {
            dcsp::dcsp_msg msg;

            std::vector<dcsp::agentViewElement> agentViewElementVector;
            for (int i = 0; i < no_goods.size(); i++)
            {
                dcsp::agentViewElement a;
                a.agent_identifier = no_goods[i].agent_identifier;

                dcsp::customPoint c;
                c.x = no_goods[i].value.x;
                c.y = no_goods[i].value.y;
                c.z = no_goods[i].value.z;
                a.value = c;

                agentViewElementVector.push_back(a);
            }
            msg.no_goods = agentViewElementVector;
            msg.ok = false;
            msg.nogood = true;

            if (lowest_priority_agent_identifier == "firefly_1")
            {
                dcsp_pub_1.publish(msg);
            }
            else if (lowest_priority_agent_identifier == "firefly_2")
            {
                dcsp_pub_2.publish(msg);
            }
            else if (lowest_priority_agent_identifier == "firefly_3")
            {
                dcsp_pub_3.publish(msg);
            }
        }
    }
}

void Agent::select_best_point()
{
    ROS_INFO("select the best point in dcsp_server_1.");
    double closest_euclidean_distance = std::numeric_limits<double>::infinity();

    std::vector<point>::iterator it;

    for (it = my_domain.begin(); it != my_domain.end(); ++it)
    {
        double x = it->x - my_current_point.x;
        double y = it->y - my_current_point.y;
        double z = it->z - my_current_point.z;
        double dist;

        dist = pow(x, 2) + pow(y, 2) + pow(z, 2);

        double euclidean_distance = sqrt(dist);

        if (euclidean_distance < closest_euclidean_distance)
        {
            closest_euclidean_distance = euclidean_distance;
            my_current_point = *it;
            ROS_INFO("BETS POINT IS SELECTED IN DCSP_SERVER_1");
            ROS_INFO("BEST POINT OF DCSP_SERVER_1 X, Y, Z: %f, %f, %f", my_current_point.x, my_current_point.y, my_current_point.z);
        }
    }
}

bool Agent::same_points(const Agent::point &point1, const Agent::point &point2)
{
    if ((point1.x == point2.x) && (point1.y == point2.y) && (point1.z == point2.z))
    {
        return true;
    }
    return false;
}

void Agent::dcsp_msg_2_recieve_ok()
{
    dcsp::dcsp_msg msg;

    msg.ok_agent_identifier = my_agent_identifier;

    dcsp::customPoint c;
    c.x = my_current_point.x;
    c.y = my_current_point.y;
    c.z = my_current_point.z;

    msg.ok_value = c;

    msg.ok = true;
    msg.nogood = false;

    dcsp_pub_2.publish(msg);
}

void Agent::dcsp_msg_3_recieve_ok()
{
    dcsp::dcsp_msg msg;

    msg.ok_agent_identifier = my_agent_identifier;

    dcsp::customPoint c;
    c.x = my_current_point.x;
    c.y = my_current_point.y;
    c.z = my_current_point.z;

    msg.ok_value = c;

    msg.ok = true;
    msg.nogood = false;

    dcsp_pub_3.publish(msg);
}
