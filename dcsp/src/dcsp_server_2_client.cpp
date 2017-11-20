#include <iostream>
#include "ros/ros.h"
#include "dcsp/dcsp_msg.h"
#include "dcsp/customPoint.h"
#include "dcsp/agentViewElement.h"

#include <list>
#include <vector>
#include <math.h>

int main(int argc, char **argv)
   {
     int c1 = 0, c2 = 0, c3 = 0;
      ros::init(argc, argv, "dcsp_server_2_client");

  //    if (argc != 3)
  //    {
  //     ROS_INFO("usage: dcsp_server_1_client X Y");
  //     return 1;
  //   }
   
     ros::NodeHandle n;
       ros::Publisher dcsp_pub_2 = n.advertise<dcsp::dcsp_msg>("dcsp_msg_2", 1000);
     dcsp::dcsp_msg msg;
       
     msg.init_agent_identifier = "firefly_2";

     dcsp::customPoint cp;
//     cp.x = 1.2;
//     cp.y = 1.0;
//     cp.z = 2.5;
    cp.x = 1.0;
    cp.y = 3.2;
    cp.z = 1.5;
     msg.init_point = cp;
       
     std::vector<std::string> agents_vector;
     agents_vector.push_back("firefly_1");
     agents_vector.push_back("firefly_3");
     msg.agents = agents_vector;

     std::vector<dcsp::customPoint> my_domain_vector;

//     for (int i = 0; i < 6 ; ++i) {
//         dcsp::customPoint c;
//           c.x = 1.4 + c1;
//           c.y = 3.0 + c2;
//           c.z = 7.1 + c3;
//           my_domain_vector.push_back(c);
//           c1 += 2.0;
//           c2 += 1.9;
//           c3 += 7.4;
//       }
  for (int i = 0; i < 1 ; ++i) {
    dcsp::customPoint c;
    c.x = 2.4 + c1;
    c.y = 5.7 + c2;
    c.z = 1.1 + c3;
    my_domain_vector.push_back(c);
    c1 += 2.1;
    c2 += 1.2;
    c3 += 3.4;
  }
       msg.my_domain = my_domain_vector;

       std::string ok_agent_identifier_variable = "test";
       msg.ok_agent_identifier  = ok_agent_identifier_variable;
       dcsp::customPoint ok_value_point;
       ok_value_point.x = 1.2;
       ok_value_point.y = 0.3;
       ok_value_point.z = 2.9;
       msg.ok_value = ok_value_point;

       std::vector<dcsp::agentViewElement> no_goods_vector;

           dcsp::agentViewElement agnt1;
           agnt1.agent_identifier = "firefly_3";
           agnt1.value.x = 1.8;
           agnt1.value.y = 4.8;
           agnt1.value.z = 1.3;
       no_goods_vector.push_back(agnt1);

       dcsp::agentViewElement agnt2;
       agnt2.agent_identifier = "firefly_1";
       agnt2.value.x = 1.1;
       agnt2.value.y = 3.5;
       agnt2.value.z = 7.3;
       no_goods_vector.push_back(agnt2);

       dcsp::agentViewElement agnt3;
       agnt3.agent_identifier = "firefly_3";
       agnt3.value.x = 9.5;
       agnt3.value.y = 6.0;
       agnt3.value.z = 2.5;
       no_goods_vector.push_back(agnt3);

       msg.no_goods = no_goods_vector;

       msg.init = true;
       msg.ok = false;
       msg.nogood = false;


       ROS_INFO("Init : %d",msg.init );
       ros::Rate loop_rate(0.1);
       while(n.ok()){
           dcsp_pub_2.publish(msg);
           ros::spinOnce();
           loop_rate.sleep();
       }


       return 0;
   }
