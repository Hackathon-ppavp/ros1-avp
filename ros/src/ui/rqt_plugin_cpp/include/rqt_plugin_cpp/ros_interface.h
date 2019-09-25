#ifndef ROS_API_H_
#define ROS_API_H_

#include <rqt_gui_cpp/plugin.h>
#include <rqt_plugin_cpp/ui_my_plugin.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosbag/recorder.h>
class Record: public rosbag::Recorder
{
public:
  Record(ros::NodeHandle& nh,rosbag::RecorderOptions const& options);

};



class RosNode: public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  RosNode(Ui::MyPluginWidget& ui):ui_(ui)
  {
    sub_state = nh.subscribe("/op_state", 1, &RosNode::opCallback, this);
    pub_cmd = nh.advertise<std_msgs::String>("/route_csv", 1);
  }
  void park();
  void summon();
  void abort();

private:
  void opCallback(std_msgs::Float32 msg);


private:
  Ui::MyPluginWidget ui_;

private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Subscriber sub_state;
  ros::Publisher pub_cmd;
};


#endif
