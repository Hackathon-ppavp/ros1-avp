#include "rqt_plugin_cpp/ros_interface.h"



void RosNode::opCallback(std_msgs::Float32 msg)
{
  //TODO::fill me up

}

void RosNode::park()
{
  std_msgs::String route;
  route.data = "park.csv";
  pub_cmd.publish(route);
}

void RosNode::summon()
{
  std_msgs::String route;
  route.data = "summon.csv";
  pub_cmd.publish(route);
}

void RosNode::abort()
{
  std_msgs::String route;
  route.data = "abort.csv";
  pub_cmd.publish(route);
}
