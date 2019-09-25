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
 * # Brief: Tansform current vehicle velocity from WORLD to BASE_LINK, 
 *          Optional using LowPassFilter on the velocity
 */

#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/VehicleStatus.h>

// pose and velocity in the GLOBAL frame
ros::Subscriber sub_current_pose;
// estimated velocity in the vehicle BASE frame
ros::Publisher pub_base_velocity;
// vehicle status (twist and wheel angle in the vehicle frame)
ros::Publisher pub_vehicle_status;

// variables
geometry_msgs::PoseStamped previous_msg_pose;
geometry_msgs::TwistStamped vehicle_twist;
bool first_pose_recv = false;
double duration = 0.02;

// ROS params
std::string current_pose_topic, estimated_twist_topic, vehicle_status_topic;
std::string right_wheel_axel_tf, left_wheel_axel_tf, vehicle_frame_tf;

// type definition
struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};
static pose current_pose, previous_pose;


// copy idea from ndt_matching autoware
tf::Transform convertPoseIntoRelativeCoordinate(const geometry_msgs::PoseStamped& target_pose, const geometry_msgs::PoseStamped& reference_pose)
{
  // tf target_pose in world
  tf::Quaternion target_q(target_pose.pose.orientation.x,
                          target_pose.pose.orientation.y,
                          target_pose.pose.orientation.z,
                          target_pose.pose.orientation.w);
  tf::Vector3 target_v(target_pose.pose.position.x,
                       target_pose.pose.position.y,
                       target_pose.pose.position.z);
  tf::Transform target_tf(target_q, target_v);

  // tf reference (old pose) in world
  tf::Quaternion reference_q(reference_pose.pose.orientation.x,
                             reference_pose.pose.orientation.y,
                             reference_pose.pose.orientation.z,
                             reference_pose.pose.orientation.w);
  tf::Vector3 reference_v(reference_pose.pose.position.x,
                          reference_pose.pose.position.y,
                          reference_pose.pose.position.z);
  tf::Transform reference_tf(reference_q, reference_v);

  // tf target and reference frame
  tf::Transform trans_target_tf = reference_tf.inverse() * target_tf;
  return trans_target_tf;
}

// Pose with +- a velocity in the vehicle frame
void currentPoseCallback(const geometry_msgs::PoseStamped& pose)
{
  try
  {
    // check first message has arrived before for computing
    if(!first_pose_recv)
    {
      previous_msg_pose = pose;
      first_pose_recv = true;
    }
    else
    {
      // update only every N cycle eg. 50 hz
      double dt = pose.header.stamp.toSec() - previous_msg_pose.header.stamp.toSec();
      if(dt > duration)
      {
        // *                                                  * //
        // * Linear and Angular velocity in the vehicle frame * //
        // *                                                  * //

        // computing angular velocity: yaw-z axis
        double current_roll, current_pitch, current_yaw;
        double prev_roll, prev_pitch, prev_yaw;
        tf::Quaternion current_q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        tf::Quaternion prev_q(previous_msg_pose.pose.orientation.x, previous_msg_pose.pose.orientation.y, previous_msg_pose.pose.orientation.z, previous_msg_pose.pose.orientation.w);
        tf::Matrix3x3 m2(current_q);
        tf::Matrix3x3 m1(prev_q);
        m2.getRPY(current_roll, current_pitch, current_yaw);
        m1.getRPY(prev_roll, prev_pitch, prev_yaw);
        double w = (current_yaw-prev_yaw)/dt;

        // computing linear velocity from diff position
        double dx = pose.pose.position.x - previous_msg_pose.pose.position.x;
        double dy = pose.pose.position.y - previous_msg_pose.pose.position.y;
        double dz = pose.pose.position.z - previous_msg_pose.pose.position.z;
        double v  = (sqrt(dx*dx + dy*dy + dz*dz))/dt;

        // check +,- velocity by frame transformation between current and previous pose in X direction
        tf::Transform trans_current_pose = convertPoseIntoRelativeCoordinate(pose, previous_msg_pose);
        v =  (trans_current_pose.getOrigin().getX() >= 0) ? v : -v;

        // publish twist message
        geometry_msgs::TwistStamped twist;
        twist.header = pose.header;
        twist.header.frame_id = "base_link";
        twist.twist.linear.x = v;
        twist.twist.angular.z = w;
        pub_base_velocity.publish(twist);
        // record data
        previous_msg_pose = pose;
        vehicle_twist = twist;
      }

    }

  }
  catch (ros::Exception& e)
  {
    ROS_ERROR("Updating current pose callback error");
  }
}




int main(int argc, char **argv)
{
  // ros setup
  ros::init(argc, argv, "twist_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  // ros params
  nh_param.param<std::string>("current_pose_topic", current_pose_topic, "/current_pose");
  nh_param.param<std::string>("estimated_twist_topic", estimated_twist_topic, "/estimated_twist");
  nh_param.param<std::string>("vehicle_status_topic", vehicle_status_topic, "/vehicle_status");
  nh_param.param<std::string>("right_wheel_axle_tf", right_wheel_axel_tf, "/fr_axle");
  nh_param.param<std::string>("left_wheel_axle_tf", left_wheel_axel_tf, "/fl_axle");
  nh_param.param<std::string>("vehicle_frame_tf", vehicle_frame_tf, "/chassis");


  // subscribers
  sub_current_pose = nh.subscribe(current_pose_topic, 10, currentPoseCallback);

  // publishers
  pub_base_velocity = nh.advertise<geometry_msgs::TwistStamped>(estimated_twist_topic, 10);
  pub_vehicle_status = nh.advertise<autoware_msgs::VehicleStatus>(vehicle_status_topic, 10);

  // transform
  tf::TransformListener listener;
  tf::StampedTransform transform_left_wheel;
  tf::StampedTransform transform_right_wheel;

  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    // * ---------------------------------- *//
    // * Compute Wheel angle from tf frames *//
    // * ---------------------------------- *//
    try
    {
      listener.waitForTransform(vehicle_frame_tf, left_wheel_axel_tf, ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform(vehicle_frame_tf, left_wheel_axel_tf, ros::Time(0), transform_left_wheel);
      listener.waitForTransform(vehicle_frame_tf, right_wheel_axel_tf, ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform(vehicle_frame_tf, right_wheel_axel_tf, ros::Time(0), transform_right_wheel);
    }
    catch (tf::TransformException ex)
    {
      //ROS_ERROR("%s",ex.what());
    }
    tf::Quaternion lq = transform_left_wheel.getRotation();
    tf::Quaternion rq = transform_right_wheel.getRotation();
    double left_yaw = tf::getYaw(lq);
    double right_yaw = tf::getYaw(rq);
    double mean_yaw = (left_yaw + right_yaw)/2.0;
    autoware_msgs::VehicleStatus vehicle_status;
    vehicle_status.angle = mean_yaw;                           // radian
    vehicle_status.speed = 3.6*vehicle_twist.twist.linear.x;   // kmph
    pub_vehicle_status.publish(vehicle_status);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
