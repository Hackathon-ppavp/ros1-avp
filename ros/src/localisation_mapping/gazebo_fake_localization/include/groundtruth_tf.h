#ifndef groundtruth_tf_h__
#define groundtruth_tf_h__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class BasePoseGroundTruth
{
public:
  BasePoseGroundTruth(ros::NodeHandle& nh_private, ros::NodeHandle& nh_param);
  ~BasePoseGroundTruth();
  void groundtruthCallback(const nav_msgs::OdometryConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub_groundtruth_pose;

};

#endif
