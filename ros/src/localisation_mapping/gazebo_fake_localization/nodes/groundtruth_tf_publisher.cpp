#include <groundtruth_tf.h>

// main
int main(int argc, char **argv)
{

  // ros setup
  ros::init(argc, argv, "groundtruth_tf_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  BasePoseGroundTruth base_pose_tf(nh, private_nh);
  ros::spin();
  return 0;
}
