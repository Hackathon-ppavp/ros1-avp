
#ifndef dbw_pid_h__
#define dbw_pid_h__

#include <mutex>
// ros
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

// autoware specific
#include "autoware_msgs/VehicleCmd.h"
// service call
#include "twist_to_control/brake.h"



//TODO::reconsider private/public functions- set ros callbacks are to private/and other interface function
//to public
class DbwPid
{
public:
  DbwPid(ros::NodeHandle& nh_com, ros::NodeHandle& nh_param, ros::Rate rate);
  ~DbwPid();
  void twistCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  void pidThrottleCallback(const std_msgs::Float64& msg);
  void pidSteerCallback(const std_msgs::Float64& msg);
  void currentSpeedCallback(const geometry_msgs::TwistStamped& msg);
  void brakeCmdCallback(const std_msgs::Bool& msg);
  void brakeEmergencyCallback(const std_msgs::Bool& msg);
  bool setBrakeService(twist_to_control::brake::Request& req, twist_to_control::brake::Response& res);
  int32_t  getGear(double target_velocity);

private:
  void startPid();
  void stopPid();
  void run();


private:
  // ros node & messages
  ros::NodeHandle nh;
  ros::Rate loop_rate;
  autoware_msgs::VehicleCmd vehicle_cmd;
  std_msgs::Bool pid_enable;
  bool start_pid = false;

  // ros subscribe
  ros::Subscriber sub_twist_ctr;
  ros::Subscriber sub_current_speed;
  ros::Subscriber sub_current_pose;
  ros::Subscriber sub_throttle_pid;
  ros::Subscriber sub_steer_pid;
  ros::Subscriber sub_brake_cmd;
  ros::Subscriber sub_brake_emergency;

  // ros publisher
  ros::Publisher pub_vehicle;                            // Vehicle CANBus cmd
  ros::Publisher pub_setpoint_pid_v;                     // pid set point velocity
  ros::Publisher pub_setpoint_pid_w;                     // pid set point steering
  ros::Publisher pub_pid_enable;                         // start/stop pid request
  ros::Publisher pub_state_pid_v;                        // current v
  ros::Publisher pub_state_pid_w;                        // current steering

  // ros services
  ros::ServiceServer brake_service;                      // service call brake on/off


  // local variables: twist, torque, steering config
  int counter=0;
  std::mutex vehicle_cmd_mutex;
  std::mutex twist_cmd_mutex;
  int throttle_max = 30;            // limited max throttle output %
  int steering_max = 30;            // limited max steering output %
  int brake_max = 30;               // limited max brake output %
  double gearchange_speedlimit = 3.0; // gear won't change above this speed limit m/s
  double v_max = 10;                // max speed (forward +)
  double v_min = -1;                // min speed (reverse -)
  double w_max = 0.4;               // max speed 20 degree/sec ~ 0.4 rad/s
  double w_min = -0.4;              // min speed for yaw controller
  double pid_throttle_effort = 0;   // feedback from pid controller
  double pid_steer_effort = 0;      // feedback from pid controller
  int req_throttle = 0;             // CAN vehicle
  int req_steering = 0;             // CAN vehicle
  std_msgs::Float64 pid_current_v, pid_current_w;
  std_msgs::Float64 pid_setpoint_v, pid_setpoint_w;

  // Steering control configuration
  double wheel_base;                // default 1.691 m
  double steering_ratio;            // default 14.8
  double max_steer_angle;           // defualt 25 degree

  // Brake command
  bool brake_on = false;            // activate brake
  bool emergency_brake = false;     // emergency brake from sensors- master brake (overwrites any request)
  bool operational_brake = false;   // normal driving start, stop brake

  // Gear command (Forward/Backward/Neutral)
  enum GearState {REVERSE=-1, NEUTRAL=0, FORWARD=1};
  int32_t current_gear_state = static_cast<int32_t>(GearState::NEUTRAL);

};

#endif
