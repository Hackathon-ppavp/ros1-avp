#include <mutex>
#include <dbw_pid.h>
// ros
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// autoware specific
#include "autoware_msgs/VehicleCmd.h"



// main
int main(int argc, char **argv)
{

  // ros setup
  ros::init(argc, argv, "twist_vehicle_pid_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  DbwPid dbw_controller(nh, private_nh, 200);
  return 0;
}
