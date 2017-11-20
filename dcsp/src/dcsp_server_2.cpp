//
//  main.cpp
//  DCSPNode
//
//  Created by Dulitha Dabare on 11/10/17.
//  Copyright © 2017 Dulitha Dabare. All rights reserved.
//

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dcsp/dcsp_srv.h"
#include "dcsp/customPoint.h"
#include "dcsp/agentViewElement.h"
#include "dcsp/dcsp_msg.h"

#include <list>
#include <vector>
#include <math.h>
#include <sstream>
#include <ctime>
#include <limits>

ros::Publisher dcsp_pub_1;
ros::Publisher dcsp_pub_2;
ros::Publisher dcsp_pub_3;

class Agent {


    struct point {
        double x;
        double y;
        double z;
        double yaw;
    };

    struct agent_view_element {
        std::string agent_identifier;
        point value;
    };

    /* TEMP struct defintions in place of service request and response */
    struct req {

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

    struct res {

        double x;
        double y;
        double z;


    };



private:

    std::vector<std::string> higher_priority_agents;
    std::vector<std::string> lower_priority_agents;

    std::string my_agent_identifier;
    //std::string my_current_point;
    // point my_current_point;
    point my_current_point = {
            0.0,
            0.0,
            0.0,
            0.0
    };
    std::vector<point> my_domain;
    std::vector<point> my_point_domain; //temporary because we have to change the my_domain to be of point type vector
    std::vector<agent_view_element> agent_view;
    std::vector<agent_view_element> no_goods_list;
    bool is_agent_initialized = false;


public:
    //  Agent(std::vector<Agent> &agents,const std::string &init_agent_identifier,const point &init_value,const std::vector<point> &init_my_domain , point &init_point);
    void agent_init(std::vector<std::string> &agents,const std::string &init_agent_identifier,const point &init_value,const std::vector<point> &init_my_domain);
    void recieve_ok_msg(const std::string &agent_identifier,const point &value);
    void receive_nogood_msg(const std::vector<agent_view_element> &no_goods);
    void send_ok_msg(Agent &agent);
    void send_nogood_msg(const std::vector<agent_view_element> &no_goods, const std::string lowest_priority_agent_identifier);
    void check_agent_view();
    bool check_consistency();
    bool check_compatibility ();
    bool same_points(const point &point1, const point &point2);
    void backtrack();
    void select_best_point();
    void select_method(const dcsp::dcsp_msg& req);
    void dcsp_msg_2_recieve_ok();
    void dcsp_msg_3_recieve_ok();

};
/*
Agent::Agent(std::vector<Agent> &agents,const std::string &init_agent_identifier,const point &init_value,const std::vector<point> &init_my_domain , point &init_point){

    my_agent_identifier = init_agent_identifier;
    my_current_point = init_value;
    my_domain = init_my_domain;
    my_current_point = init_point;

    std::vector<Agent>::iterator it;

    for (it = agents.begin(); it != agents.end(); ++it){

        if (my_agent_identifier.compare(it->my_agent_identifier) > 0) {

            higher_priority_agents.push_back(*it);

        }else if (my_agent_identifier.compare(it->my_agent_identifier) < 0) {

            lower_priority_agents.push_back(*it);
        }

        // what if the agent identifier is the same as the my_agent_identifier?


    }
    // select the point giving the best euclidean distance

    // point best_point = select_best_point();
    // send ok message to all the elements in the lower_priority_agents




}
*/

void Agent::agent_init(std::vector<std::string> &agents,const std::string &init_agent_identifier,const point &init_value,const std::vector<point> &init_my_domain){

    my_agent_identifier = init_agent_identifier;
    my_current_point = init_value;
    my_domain = init_my_domain;
    //my_current_point = init_point;


    ROS_INFO("DCSP SERVICE 1 INSIDE agent init");


    std::vector<std::string>::iterator it;

    for (it = agents.begin(); it != agents.end(); ++it){

        if (my_agent_identifier.compare(*it) > 0) {

            higher_priority_agents.push_back(*it);

        }else if (my_agent_identifier.compare(*it) < 0) {

            lower_priority_agents.push_back(*it);
        }

        // what if the agent identifier is the same as the my_agent_identifier?


    }
    // select the point giving the best euclidean distance

    select_best_point();

    // send ok message to all the elements in the lower_priority_agents

    ROS_INFO("Selected point x value: %f", my_current_point.x);
    ROS_INFO("Selected point y value: %f", my_current_point.y);
    ROS_INFO("Selected point z value: %f", my_current_point.z);
    is_agent_initialized = true;

    if(my_agent_identifier == "firefly_1"){
        ROS_INFO("Firefly_1 call the service of Firefly_2.");
        dcsp_msg_2_recieve_ok();
        ROS_INFO("Firefly_1 call the service of Firefly_3.");
        dcsp_msg_3_recieve_ok();
    }else if(my_agent_identifier == "firefly_2"){
        dcsp_msg_3_recieve_ok();
    }
}

void Agent::recieve_ok_msg(const std::string &agent_identifier, const point &value){
    ROS_INFO("DCSP_SERVER_2 INSIDE RECEIVE OK MESSAGE.");
    bool is_agent_in_agent_view = false;

    std::vector<agent_view_element>::iterator it;


    for (it = agent_view.begin(); it != agent_view.end(); ++it){

        if(it->agent_identifier == agent_identifier){

            is_agent_in_agent_view = true;
        }

    }

    if (!is_agent_in_agent_view) {
        agent_view_element new_element;
        new_element.agent_identifier = agent_identifier;
        new_element.value = value;
        agent_view.push_back(new_element);
    }

    check_agent_view();

}

void Agent::receive_nogood_msg(const std::vector<agent_view_element> &no_goods){
    ROS_INFO("DCSP_SERVER_2 NOGOOD RECEIVED.");
    no_goods_list.insert(no_goods_list.end(), no_goods.begin(),no_goods.end());
    check_agent_view();


}

void Agent::check_agent_view(){

    ROS_INFO("DCSP_SERVER_2 INSIDE CHECK_AGENT_VIEW.");

    if (!check_consistency()) {

        ROS_INFO("not consistent");
        bool is_new_value_consistent = false;

        std::vector<point>::iterator it;

        for (it = my_domain.begin(); it != my_domain.end(); ++it){

            if (!same_points(*it, my_current_point)) {

                std::vector<agent_view_element>::iterator it2;


                for (it2 = agent_view.begin(); it2 != agent_view.end(); ++it2){

                    if(same_points(it2->value, *it)){
                        is_new_value_consistent = false;
                        break;

                    }else{
                        is_new_value_consistent = true;
                    }

                }
            }

            if (is_new_value_consistent) {

                ROS_INFO("new consistent value found");
                my_current_point = *it;

                std::vector<std::string>::iterator it3;


                for (it3 = lower_priority_agents.begin(); it3 != lower_priority_agents.end(); ++it3){

                    std::string lower_priority_agent = *it3;
                    //lower_priority_agent.recieve_ok_msg(my_agent_identifier, my_current_point);

                    /* Call the corresponding firefly node's service. Example : for firefly1 call dcspe_srv_1 service */


//                     ros::NodeHandle n;
//                     ros::ServiceClient client = n.serviceClient<dcsp::dcsp_msg>(service_name);
                    dcsp::dcsp_msg msg;
                    // srv.request.init_agent_identifier = ""NULL"";
                    // srv.request.init_point = NULL;
                    // srv.request.agents = NULL;
                    // srv.request.my_domain = NULL;
                    // srv.request.ok_agent_identifier = my_agent_identifier;
                    msg.ok_agent_identifier = my_agent_identifier;

                    dcsp::customPoint c;
                    c.x = my_current_point.x;
                    c.y = my_current_point.y;
                    c.z = my_current_point.z;
                    msg.ok_value = c;
                    // srv.request.no_goods = NULL;
                    msg.init = false;
                    msg.ok = true;
                    msg.nogood = false;


                    if (lower_priority_agent == "firefly_1") {
                        dcsp_pub_1.publish(msg);
                    }else if (lower_priority_agent == "firefly_2") {
                        dcsp_pub_2.publish(msg);
                    }else if (lower_priority_agent == "firefly_3") {
                        dcsp_pub_3.publish(msg);
                    }

                }

                break;

            }
        }

        if (!is_new_value_consistent) {
            backtrack();
        }

    }else{
        ROS_INFO("FIREFLY 1 value is consistent");
    }

}

bool Agent::check_consistency(){

    bool is_consistent = false;
    bool is_agent_view_consistent = true;

    std::vector<agent_view_element>::iterator it;

    for (it = agent_view.begin(); it != agent_view.end(); ++it){

        if (same_points(it->value, my_current_point)) {
            is_agent_view_consistent = false;
        }

    }

    bool is_compatible = check_compatibility();

    if (is_agent_view_consistent && (!is_compatible)) {
        is_consistent = true;
    }else{
        is_consistent = false;
    }

    return is_consistent;

}

bool Agent::check_compatibility(){

    bool is_compatible = false;

    std::vector<agent_view_element>::iterator it;

    for (it = no_goods_list.begin(); it != no_goods_list.end(); ++it){

        if (it->agent_identifier == my_agent_identifier) {

            if (same_points(it->value, my_current_point)) {
                is_compatible = true;
                continue;
            }else{

                is_compatible = false;
                break;
            }
        }


        std::vector<agent_view_element>::iterator agent_view_it;

        for (agent_view_it = agent_view.begin(); agent_view_it != agent_view.end(); ++agent_view_it){

            if (same_points(agent_view_it->value, it->value)) {
                is_compatible = true;
                continue;
            }else{

                is_compatible = false;
                break;
            }
        }

        if (is_compatible) {
            continue;
        }else{
            break;
        }


    }

    return is_compatible;
}


void Agent::backtrack(){

    // Add inconsistent subset of agent_view to a new vector called no_goods


    std::vector<agent_view_element> no_goods;

    std::vector<agent_view_element>::iterator it;

    for (it = agent_view.begin(); it != agent_view.end(); ++it){

        if (same_points(it->value, my_current_point) ) {

            no_goods.push_back(*it);
        }

    }

    if (no_goods.empty()) {
        puts("No Solution found.");
    }else{

        agent_view_element first_agent_view_element = no_goods.front();
        std::string lowest_priority_agent_identifier = first_agent_view_element.agent_identifier;

        std::vector<agent_view_element>::iterator it2;

        for (it2 = agent_view.begin(); it2 != agent_view.end(); ++it2){

            if ((lowest_priority_agent_identifier.compare(it2->agent_identifier)) < 0) {

                lowest_priority_agent_identifier = it2->agent_identifier;

            }

        }

        send_nogood_msg(no_goods, lowest_priority_agent_identifier);

    }

}

void Agent::send_nogood_msg(const std::vector<agent_view_element> &no_goods, const std::string lowest_priority_agent_identifier){

    std::vector<std::string>::iterator it;

    for (it = higher_priority_agents.begin(); it != higher_priority_agents.end(); ++it){

        if (*it == lowest_priority_agent_identifier) {

            //   it->receive_nogood_msg(no_goods);

            /* Call the corresponding firefly node's service. Example : for firefly1 call dcspe_srv_1 service */



            /*   ROS SERVICE CALL TO THE CORRESPONDING FIREFLY DCSP SERVICE with NOGOOD flag */

//            ros::NodeHandle n;
//            ros::ServiceClient client = n.serviceClient<dcsp::dcsp_msg>(service_name);
            dcsp::dcsp_msg msg;
            // srv.request.init_agent_identifier = NULL;
            // srv.request.init_point = NULL;
            // srv.request.agents = NULL;
            // srv.request.my_domain = NULL;
            // srv.request.ok_agent_identifier = NULL;
            // srv.request.ok_value = NULL;
            std::vector<dcsp::agentViewElement> agentViewElementVector;
            for (int i=0; i<no_goods.size(); i++){
                dcsp::agentViewElement a;
                a.agent_identifier = no_goods[i].agent_identifier ;

                dcsp::customPoint c;
                c.x = no_goods[i].value.x;
                c.y = no_goods[i].value.y;
                c.z = no_goods[i].value.z;
                a.value = c;

                agentViewElementVector.push_back(a);
            }
            msg.no_goods = agentViewElementVector;
            msg.init = false;
            msg.ok = false;
            msg.nogood = true;


            if (lowest_priority_agent_identifier == "firefly_1") {
                dcsp_pub_1.publish(msg);
            }else if (lowest_priority_agent_identifier == "firefly_2") {
                dcsp_pub_2.publish(msg);
            }else if (lowest_priority_agent_identifier == "firefly_3") {
                dcsp_pub_3.publish(msg);
            }


        }

    }

}

void Agent::select_best_point(){

    ROS_INFO("select the best point in dcsp_server_2.");
    double closest_euclidean_distance = std::numeric_limits<double>::infinity();;

    std::vector<point>::iterator it;

    for (it = my_domain.begin(); it != my_domain.end(); ++it){

        double x = it->x - my_current_point.x;
        double y = it->y - my_current_point.y;
        double z = it->z - my_current_point.z;
        double dist;

        dist = pow(x,2)+pow(y,2)+pow(z,2);

        double euclidean_distance = sqrt(dist);
        ROS_INFO("BETS POINT IS SELECTED IN DCSP_SERVER_2");
        ROS_INFO("BEST POINT OF DCSP_SERVER_1 X, Y, Z: %f, %f, %f", my_current_point.x, my_current_point.y, my_current_point.z);
        if (euclidean_distance<closest_euclidean_distance) {
            closest_euclidean_distance = euclidean_distance;
            my_current_point = *it;
            }

    }
}

bool Agent::same_points(const Agent::point &point1, const Agent::point &point2){

    bool is_same_point = false;

    if((point1.x == point2.x) && (point1.y == point2.y) ){
        is_same_point = true;
    }

    return is_same_point;

}

void Agent::dcsp_msg_2_recieve_ok(){

//    ros::NodeHandle n;
//    ros::ServiceClient client = n.serviceClient<dcsp::dcsp_msg>("dcsp_msg_2");

    dcsp::dcsp_msg msg;
    // srv.request.init_agent_identifier = NULL;
    // srv.request.init_point = NULL;
    // srv.request.agents = NULL;
    // srv.request.my_domain = NULL;
    msg.ok_agent_identifier = my_agent_identifier;

    dcsp::customPoint c;
    c.x = my_current_point.x;
    c.y = my_current_point.y;
    c.z = my_current_point.z;
    msg.ok_value = c;

    // srv.request.no_goods = NULL;
    msg.init = false;
    msg.ok = true;
    msg.nogood = false;

//    if (client.call(srv))
//    {
//    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
//    }
//    else
//    {
//    //  ROS_ERROR("Failed to call service add_two_ints");
//    //  return 1;
//    }
    dcsp_pub_2.publish(msg);
}

void Agent::dcsp_msg_3_recieve_ok(){

//        ros::NodeHandle n;
//        ros::ServiceClient client = n.serviceClient<dcsp::dcsp_msg>("dcsp_msg_3");
    dcsp::dcsp_msg msg;
    // srv.request.init_agent_identifier = NULL;
    // srv.request.init_point = NULL;
    // srv.request.agents = NULL;
    // srv.request.my_domain = NULL;
    msg.ok_agent_identifier = my_agent_identifier;

    dcsp::customPoint c;
    c.x = my_current_point.x;
    c.y = my_current_point.y;
    c.z = my_current_point.z;
    msg.ok_value = c;
    // srv.request.no_goods = NULL;
    msg.init = false;
    msg.ok = true;
    msg.nogood = false;

//        if (client.call(srv))
//        {
//        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
//        }
//        else
//        {
//        //  ROS_ERROR("Failed to call service add_two_ints");
//        //  return 1;
//        }
    dcsp_pub_3.publish(msg);
}
void Agent::select_method(const dcsp::dcsp_msg& req){

    //Agent::point point;
    //Agent::req request;
    //Agent::req response;
    ROS_INFO("Inside the select_method of dcsp_server_1");

    if(req.init){
        std::vector<std::string> agentsVector;
        for (int i=0; i<req.agents.size(); i++){
            agentsVector.push_back(req.agents[i]);
        }

        std::string init_agent_identifier_variable = req.init_agent_identifier;
        point pt;
        pt.x = req.init_point.x;
        pt.y = req.init_point.y;
        pt.z = req.init_point.z;

        std::vector<point> my_domain_vector;
        for (int i=0; i<req.my_domain.size(); i++){
            point p;
            p.x = req.my_domain[i].x;
            p.y = req.my_domain[i].y;
            p.z = req.my_domain[i].z;

            my_domain_vector.push_back(p);
        }
        agent_init(agentsVector, init_agent_identifier_variable ,pt ,my_domain_vector);


    }else if (req.ok){
        ROS_INFO("DCSP_SERVER_2 RECIEVED A OK MESSAGE");
        std::string ok_agent_identifier_string = req.ok_agent_identifier;
        point p;
        p.x = req.ok_value.x;
        p.y = req.ok_value.y;
        p.z = req.ok_value.z;
        recieve_ok_msg(ok_agent_identifier_string, p);

    }else if (req.nogood){
        ROS_INFO("DCSP_SERVER_2 RECIEVED A NOGOOD MESSAGE");
        std::vector<agent_view_element> no_goods_vector;
        for (int i=0; i<req.no_goods.size(); i++){
            agent_view_element a;
            a.agent_identifier = req.no_goods[i].agent_identifier;
            point p;
            p.x = req.no_goods[i].value.x;
            p.y = req.no_goods[i].value.y;
            p.z = req.no_goods[i].value.z;

            a.value = p;

            no_goods_vector.push_back(a);
        }
        receive_nogood_msg(no_goods_vector);
    }
//    res.x = my_current_point.x;
//    res.y = my_current_point.y;
//    res.z = my_current_point.z;
}

int main(int argc, char** argv)
{
    std::clock_t start; ///////////
    double duration; /////////////


    ros::init(argc, argv, "dcsp_server_2");
    ros::NodeHandle n;
    Agent agent_temp;

    start = std::clock(); ///////////////////

    //ros::ServiceServer service = n.advertiseService("dcsp_mag_1", &Agent::select_method, &agent_temp);

    ROS_INFO("DCSP SERVICE 2 STARTED");

    dcsp_pub_1 = n.advertise<dcsp::dcsp_msg>("dcsp_msg_1", 1000);
    dcsp_pub_2 = n.advertise<dcsp::dcsp_msg>("dcsp_msg_2", 1000);
    dcsp_pub_3 = n.advertise<dcsp::dcsp_msg>("dcsp_msg_3", 1000);

    ros::Rate loop_rate(0.1);

    ros::Subscriber sub = n.subscribe("dcsp_msg_2", 1000, &Agent::select_method, &agent_temp);

    while(n.ok()){

        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC; ////////

        if(duration >= 0.1){
            ROS_INFO("Publish my topics");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

