
#include <groundtruth_tf.h>

BasePoseGroundTruth::BasePoseGroundTruth(ros::NodeHandle& nh_private, ros::NodeHandle& nh_param):
  nh(nh_private)
{
  // subscriber
  sub_groundtruth_pose = nh.subscribe("/base_pose_ground_truth", 10, &BasePoseGroundTruth::groundtruthCallback, this);
}


BasePoseGroundTruth::~BasePoseGroundTruth()
{

}

void BasePoseGroundTruth::groundtruthCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // build tf
  tf::Vector3 v(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Transform msg_to_tf(q, v);
  // build tf stamped
  tf::StampedTransform trans(msg_to_tf, msg->header.stamp, "/map", "/base_link");
  // broadcast to tf
  tf::TransformBroadcaster br;
  br.sendTransform(trans);
  ROS_INFO("publish tf /map /base_link");
}
