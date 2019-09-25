
#ifndef kinematics_vehicle_pid_h_
#define kinematics_vehicle_pid_h_

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
#include "autoware_msgs/ControlCommandStamped.h"


class KinematicPid
{
public:
  KinematicPid(ros::NodeHandle& nh_com, ros::NodeHandle& nh_param, ros::Rate rate);
  ~KinematicPid();
  void controlCallback(const autoware_msgs::ControlCommandStampedConstPtr& msg);
  void pidThrottleCallback(const std_msgs::Float64& msg);
  void currentSpeedCallback(const geometry_msgs::TwistStamped& msg);
  void brakeCmdCallback(const std_msgs::Bool& msg);
  void brakeEmergencyCallback(const std_msgs::Bool& msg);
  void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
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
  ros::Subscriber sub_ctrl_cmd;
  ros::Subscriber sub_current_speed;
  ros::Subscriber sub_current_pose;
  ros::Subscriber sub_throttle_pid;
  ros::Subscriber sub_brake_cmd;
  ros::Subscriber sub_brake_emergency;

  // ros publisher
  ros::Publisher pub_vehicle;           // Vehicle CANBus cmd
  ros::Publisher pub_setpoint_pid_v;    // pid set point velocity
  ros::Publisher pub_pid_enable;        // start/stop pid request
  ros::Publisher pub_state_pid_v;       // current v

  // local variables: twist, torque, steering config
  int counter=0;
  std::mutex vehicle_cmd_mutex;
  std::mutex control_cmd_mutex;
  int throttle_max = 30;                // limited max throttle output %
  int steering_max = 30;                // limited max steering output %
  int brake_max = 30;                   // limited max braking output %
  double gearchange_speedlimit = 3.0; // gear won't change above this speed limit m/s
  double v_max = 10;                    // max speed (forward +)
  double v_min = -1;                    // min speed (reverse -)
  double pid_throttle_effort = 0;       // feedback from pid controller
  double ctrl_steer_effort = 0;         // feedback from mpc controller
  int req_throttle = 0;                 // CAN vehicle
  int req_steering = 0;                 // CAN vehicle
  std_msgs::Float64 pid_current_v;
  std_msgs::Float64 pid_setpoint_v;

  // Steering control configuration
  double wheel_base;                    // default 1.691 m
  double steering_ratio;                // default 14.8
  double max_steer_angle;               // defualt 25 degree
  double steering_effort_ratio;         // effort/rad

  // Brake command
  bool brake_on = false;                // activate brake
  bool emergency_brake = false;         // emergency brake from sensors- master brake (overwrites any request)
  bool operational_brake = false;       // normal driving start, stop brake

  // Gear command (Forward/Backward/Neutral)
  enum GearState {REVERSE=-1, NEUTRAL=0, FORWARD=1};
  int32_t current_gear_state = static_cast<int32_t>(GearState::NEUTRAL);

};

#endif
