#include <iostream>
#include "ros/ros.h"
#include "dcsp/dcsp_srv.h"
#include "dcsp/customPoint.h"
#include "dcsp/agentViewElement.h"

#include <list>
#include <vector>
#include <math.h>

int main(int argc, char **argv)
   {
       int c1 = 0.2, c2 = 2.5, c3 = 1.1;
      ros::init(argc, argv, "dcsp_server_3_client");

  //    if (argc != 3)
  //    {
  //     ROS_INFO("usage: dcsp_server_1_client X Y");
  //     return 1;
  //   }
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<dcsp::dcsp_srv>("dcsp_srv_3");
     dcsp::dcsp_srv srv;
       
     srv.request.init_agent_identifier = "firefly_3";

     dcsp::customPoint cp;
     cp.x = 1.2;
     cp.y = 9.3;
     cp.z = 2.5;
     srv.request.init_point = cp;
       
     std::vector<std::string> agents_vector;
     agents_vector.push_back("firefly_1");
     agents_vector.push_back("firefly_2");
     srv.request.agents = agents_vector;

     std::vector<dcsp::customPoint> my_domain_vector;

     for (int i = 0; i < 6 ; ++i) {
         dcsp::customPoint c;
           c.x = 1.0 + c1;
           c.y = 9.0 + c2;
           c.z = 7.1 + c3;
           my_domain_vector.push_back(c);
           c1 += 2.0;
           c2 += 1.9;
           c3 += 5.4;
       }
       srv.request.my_domain = my_domain_vector;

       std::string ok_agent_identifier_variable = "test";
       srv.request.ok_agent_identifier  = ok_agent_identifier_variable;
       dcsp::customPoint ok_value_point;
       ok_value_point.x = 3.5;
       ok_value_point.y = 0.3;
       ok_value_point.z = 1.9;
       srv.request.ok_value = ok_value_point;

       std::vector<dcsp::agentViewElement> no_goods_vector;

           dcsp::agentViewElement agnt1;
           agnt1.agent_identifier = "firefly_1";
           agnt1.value.x = 8.8;
           agnt1.value.y = 0.8;
           agnt1.value.z = 1.3;
       no_goods_vector.push_back(agnt1);

       dcsp::agentViewElement agnt2;
       agnt2.agent_identifier = "firefly_2";
       agnt2.value.x = 1.1;
       agnt2.value.y = 4.5;
       agnt2.value.z = 0.9;
       no_goods_vector.push_back(agnt2);

       dcsp::agentViewElement agnt3;
       agnt3.agent_identifier = "firefly_1";
       agnt3.value.x = 5.5;
       agnt3.value.y = 8.0;
       agnt3.value.z = 2.0;
       no_goods_vector.push_back(agnt3);

       srv.request.no_goods = no_goods_vector;

       srv.request.init = true;
       srv.request.ok = false;
       srv.request.nogood = false;

     if (client.call(srv))
    {
       ROS_INFO("Message received.");
     }
     else
     {
       ROS_ERROR("Failed to call dcsp service.");
       return 1;
     }
   
     return 0;
   }
