/*
        This node is used in conjunction with joy stick twist message to drive the vehicle manually
        It map v directly to % throttle  (mapping MAX v twist to MAX % throttle as defined)
               w directly to % steering  (mapping MAX w twist to MAX % steering as defined)
*/

// ros
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

// autoware specific
#include "autoware_msgs/VehicleCmd.h"

// ros subscribe, publisher
ros::Subscriber sub_twist;
ros::Publisher pub_vehicle;
autoware_msgs::VehicleCmd vehicle_cmd;
int counter=0;


// twist, torque, steering config
float throttle_max;    // max throttle output %
float steering_max;    // max steering output %
float twist_v_max;      // teleop max v
float twist_w_max;      // teleop max w




// Twist callback function: Sub Pub Process
void twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
  try
  {
    // get data
    double v = msg.get()->linear.x;
    double w = msg.get()->angular.z;
    // linear mapping twist cmd into throttle and steering command
    // v -0.7 to 0.7 mapped to throttle -50 to 50 %
    // w -0.4 to 0.4 mapped to steering -30 to 30 %
    int req_throttle = int(v * throttle_max/twist_v_max);
    int req_steering = int(w * steering_max/twist_w_max);

    ROS_INFO("request throttle: [%d]", req_throttle);
    ROS_INFO("request steering: [%d]", req_steering);
    // publish throttle, steering command
    vehicle_cmd.header.seq = counter;
    vehicle_cmd.header.stamp = ros::Time::now();
    vehicle_cmd.steer_cmd.steer = req_steering;
    vehicle_cmd.accel_cmd.accel = req_throttle;
    pub_vehicle.publish(vehicle_cmd);
    counter++;
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Could not convert a message to a car command!");
  }
}


// main
int main(int argc, char **argv)
{
  // ros setup
  ros::init(argc, argv, "twist_vehicle_mapper_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  // param
  private_nh.param<float>("throttle_max", throttle_max, 50.0);
  private_nh.param<float>("steering_max", steering_max, 40.0);

  // get other node rosparam if there is any, otherwise get from launch the file
  if( !private_nh.getParam("/teleop_twist_joy/scale_linear", twist_v_max) )
    private_nh.param<float>("twist_v_max", twist_v_max, 2.0);
  if( !private_nh.getParam("/teleop_twist_joy/scale_angular", twist_w_max) )
    private_nh.param<float>("twist_w_max", twist_w_max, 2.0);

  // init subscriber/publisher
  sub_twist = nh.subscribe("/cmd_vel", 10, twistCallback);   // target v,w
  pub_vehicle = nh.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);

  ros::spin();

	return 0;
}
