/*
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Parkopedia. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * # Author: Punnu Phairatt, Parkopedia
 * # Brief: Use in conjunction with ROS PID Controller for a motion feedback control
 *          of throttle and steering controls
 */

#include "kinematics_vehicle_pid.h"

KinematicPid::KinematicPid(ros::NodeHandle& nh_com, ros::NodeHandle& nh_param, ros::Rate rate):
  nh(nh_com), loop_rate(rate)
{
  // vehicle control param
  nh_param.param<int>("throttle_max", throttle_max, 50);  // limited % throttle power
  nh_param.param<int>("steering_max", steering_max, 50);  // limited % steering power
  nh_param.param<int>("brake_max", brake_max, 50);  // limited % braking power
  nh_param.param<double>("gearchange_speedlimit", gearchange_speedlimit, 3.0);  // gear won't change above this speed
  nh_param.param<double>("steering_effort_ratio", steering_effort_ratio, 83.35); // % steer effort/ rad

  // yaw controller param
  nh_param.param<double>("wheel_base", wheel_base, 1.686);
  nh_param.param<double>("steering_ratio", steering_ratio, 1.0);
  nh_param.param<double>("linear_v_max", v_max, 10.0); // m/s - Forward velocity with + sign
  nh_param.param<double>("linear_v_min", v_min, -3.0); // m/s - Reverse velocity with - sign
  nh_param.param<double>("max_steer_angle", max_steer_angle, 0.6);

  // Read other topic param
  std::string control_command_topic, sd_throttle_pid_topic, current_velocity_topic, \
              setpoint_linear_topic, vehicle_command_topic, pid_enable_topic, \
              state_linear_topic, current_pose_topic, brake_cmd_topic, brake_emergency_topic;
  nh_param.param<std::string>("control_command_topic", control_command_topic, "/ctrl_cmd");
  nh_param.param<std::string>("sd_throttle_pid_topic", sd_throttle_pid_topic, "/sd_throttle/control_effort");
  nh_param.param<std::string>("current_velocity_topic", current_velocity_topic, "/estimated_twist");
  nh_param.param<std::string>("setpoint_linear_topic", setpoint_linear_topic, "/setpoint/linear_velocity");
  nh_param.param<std::string>("pid_enable_topic", pid_enable_topic, "/pid_enable");
  nh_param.param<std::string>("state_linear_topic", state_linear_topic, "/sd_throttle/state");                    // meaning current v (NOT Throttle)
  nh_param.param<std::string>("vehicle_command_topic", vehicle_command_topic, "/vehicle_cmd");
  nh_param.param<std::string>("current_pose_topic", current_pose_topic, "/current_pose");
  nh_param.param<std::string>("brake_cmd_topic", brake_cmd_topic, "/final_waypoints_reach");                      // Goal reach TODO: accept cmd from Mission controller instead
  nh_param.param<std::string>("brake_emergency_topic", brake_emergency_topic, "/us/us_alarm");                    // emergency stop from Ultrasonic and Lidar safety
  // subscribers
  sub_ctrl_cmd = nh.subscribe(control_command_topic, 10, &KinematicPid::controlCallback, this);                   // setpoint target v,w
  sub_throttle_pid = nh.subscribe(sd_throttle_pid_topic, 10, &KinematicPid::pidThrottleCallback, this);           // relay to vehicle output
  sub_current_speed = nh.subscribe(current_velocity_topic, 10, &KinematicPid::currentSpeedCallback, this);        // current speed
  sub_brake_cmd = nh.subscribe(brake_cmd_topic, 10, &KinematicPid::brakeCmdCallback, this);                       // brake on when goal reach
  sub_brake_emergency = nh.subscribe(brake_emergency_topic, 10, &KinematicPid::brakeEmergencyCallback, this);     // brake on upon ultrasonic or Lidar safety
  //sub_current_pose = nh.subscribe(current_pose_topic, 10, &KinematicPid::currentPoseCallback, this);              // current vehicle pose

  // publishers
  pub_setpoint_pid_v = nh.advertise<std_msgs::Float64>(setpoint_linear_topic, 10);
  pub_vehicle = nh.advertise<autoware_msgs::VehicleCmd>(vehicle_command_topic, 10);
  pub_state_pid_v = nh.advertise<std_msgs::Float64>(state_linear_topic, 10);
  pub_pid_enable = nh.advertise<std_msgs::Bool>(pid_enable_topic, 10);

  // ros message init
  pid_setpoint_v.data = 0.0;
  pid_current_v.data = 0.0;

  // start Vehicle command publishing loop
  this->run();
}

KinematicPid::~KinematicPid()
{

}


// main run
void KinematicPid::run()
{
  while(ros::ok())
  {
    vehicle_cmd_mutex.lock();
      // publish throttle, steering command to the vehicle
      req_throttle = int(pid_throttle_effort);
      req_steering = int(ctrl_steer_effort);
    vehicle_cmd_mutex.unlock();

    // sign +,- and bound check
    req_throttle = std::min(throttle_max, std::max(-throttle_max, req_throttle));

    // sign +,- and bound check
    req_steering = std::min(steering_max, std::max(-steering_max, req_steering));

    // publish vehicle command
    vehicle_cmd.header.seq = counter;
    vehicle_cmd.header.stamp = ros::Time::now();
    vehicle_cmd.steer_cmd.steer = req_steering;
    // condition throttle based on brake cmd and reset the PID
    if(brake_on)
    {
      // in case of emergency brake or final destination reach
      // put the brake on, steering 0, switch gear to neutral, and pid off
      vehicle_cmd.accel_cmd.accel = -abs(brake_max);
      vehicle_cmd.steer_cmd.steer = 0;
      vehicle_cmd.gear = static_cast<int32_t>(GearState::NEUTRAL);
      current_gear_state = static_cast<int32_t>(GearState::NEUTRAL);

      // stop the pid if it is true (still running)
      if(start_pid)
      {
        this->stopPid();
      }

    }
    else
    {
      // start the pid if it is false (not start yet)
      if(!start_pid)
      {
        this->startPid();
      }
      vehicle_cmd.accel_cmd.accel = req_throttle;
      vehicle_cmd.gear = current_gear_state;
    }

    // put the emergency brake state to the vehicle comand field
    vehicle_cmd.emergency = (emergency_brake ? 1:0);
    // send the vehicle command to the vehicle dbw interface
    pub_vehicle.publish(vehicle_cmd);

    // publish current and setpint v,w to pid controller
    // vehicle is in a running state
    if(start_pid)
    {
      control_cmd_mutex.lock();
        pub_setpoint_pid_v.publish(pid_setpoint_v);
        pub_state_pid_v.publish(pid_current_v);
      control_cmd_mutex.unlock();
    }
    // vehicle is in a stopped state (publish messages to reset a PID state)
    else
    {
        std_msgs::Float64 reset_pid_state;
        reset_pid_state.data = 0.0;
        pub_setpoint_pid_v.publish(reset_pid_state);
        pub_state_pid_v.publish(reset_pid_state);
    }
    counter++;

    // sleep rate
    loop_rate.sleep();
    ros::spinOnce();
  }
}

/*
*  ROS message callback functions
*/
// Control callback function: turning control command v, steering angle to % unit throttle/steering angle for the vehicle
void KinematicPid::controlCallback(const autoware_msgs::ControlCommandStampedConstPtr& msg)
{
  try
  {
    // update current and setpoint pid from twist messages
    // publish a new setpoint to pid controllers (v and steering angle setpoint)
    double v_target = msg.get()->cmd.linear_velocity;
    double steering_target = msg.get()->cmd.steering_angle;
    // compute setpoint from v,w and vehicle kinematic
    // Max speed we want to go
    // Check +,- sign and bound (note: for now we set v_min=0 so we don't reverse backward)
    v_target = std::fmin(v_max, std::fmax(v_min, v_target));

    // Check +,- sign and bound
    steering_target = std::fmin(max_steer_angle, std::fmax(-max_steer_angle, steering_target));

    // update pid data with gear setting (for Forward/Reverse drive)
    control_cmd_mutex.lock();
      // setting gear from the velocity
      current_gear_state = getGear(v_target);
      // setting speed (v alway positive for PID controller)
      pid_setpoint_v.data = fabs(v_target);
      // setting steering angle
      ctrl_steer_effort = steering_target * steering_effort_ratio;
    control_cmd_mutex.unlock();
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Publishing setpoint to pid error");
  }
}

// current estimated velocity
void KinematicPid::currentSpeedCallback(const geometry_msgs::TwistStamped& msg)
{
  try
  {
    control_cmd_mutex.lock();
      pid_current_v.data = std::fabs(msg.twist.linear.x);
    control_cmd_mutex.unlock();
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Updating current speed callback error");
  }
}


// Update pid throttle call back
void KinematicPid::pidThrottleCallback(const std_msgs::Float64& msg)
{
  try
  {
    vehicle_cmd_mutex.lock();
      pid_throttle_effort = msg.data;
    vehicle_cmd_mutex.unlock();
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Updating throttle pid callback error");
  }
}

void KinematicPid::brakeCmdCallback(const std_msgs::Bool& msg)
{
  // this is a start-stop braking. we don't overwrite the emergency brake if it activated
  // meaning that we can control the brake on-off normally if the emergency brake is not ativated (ON)
  try
  {
      operational_brake = msg.data;
      brake_on = operational_brake | emergency_brake;
      ROS_INFO("Command Set Brake: [%d]", brake_on);
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Brake command request callback error");
  }
}

void KinematicPid::brakeEmergencyCallback(const std_msgs::Bool& msg)
{
  // this is the master brake- if it is activated- we need to reset the whole thing
  // in order to get the car going again for safety reason
  try
  {
      brake_on = emergency_brake = msg.data;
      ROS_INFO("Emergency Set Brake: [%d]", brake_on);
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Brake emergency request callback error");
  }
}


/*
* Class member functions
*/
// enable PID node
void KinematicPid::startPid()
{
  // start pid controller
  pid_enable.data = true;
  pub_pid_enable.publish(pid_enable);
  start_pid = true;
}

// stop pid node
void KinematicPid::stopPid()
{
  // stop pid controller
  pid_enable.data = false;
  pub_pid_enable.publish(pid_enable);
  start_pid = false;
}


// checking conditions if we can go backward
int32_t KinematicPid::getGear(double target_velocity)
{
  // determine a gear from velocity +,-
  int32_t gear_output;
  if(target_velocity >= 0.0)
  {
    gear_output = static_cast<int32_t>(GearState::FORWARD);
  }
  else if(target_velocity < 0.0)
  {
    gear_output = static_cast<int32_t>(GearState::REVERSE);
  }
  else
  {
    gear_output = static_cast<int32_t>(GearState::NEUTRAL);
  }

  // make sure we don't change gear/direction at high speed e.g. 10 mph to - 10 mph and vice versa
  // this will break the motors if we switch direction while a vehicle is in motion
  if(current_gear_state != gear_output)  // gear change request
  {
    // check current speed not in motion
    if(fabs(pid_current_v.data) > gearchange_speedlimit) // still in motion (m/s)
    {
      // not ok to change gear: carry on with the current gear
      gear_output = current_gear_state;
      ROS_WARN("Cannot change gear: vehicle is still in motion !!!");
    }
  }
  return gear_output;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_vehicle_pid_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  KinematicPid kinematic_controller(nh, private_nh, 200);
  return 0;
}
