/*
  Copyright Parkopedia 2019
  Author: Punnu Phairatt
*/
#ifndef RQT_EXAMPLE_CPP_MY_PLUGIN_H
#define RQT_EXAMPLE_CPP_MY_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <rqt_plugin_cpp/ui_my_plugin.h>
#include <QWidget>
#include <QThread>
#include <QStorageInfo>
#include <QMessageBox>
#include <cmath>

// experiment with additional ros node
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "rqt_plugin_cpp/ros_interface.h"


namespace rqt_plugin_cpp
{

class MyPlugin: public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  MyPlugin();
  ~MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:
  virtual void parkAVP();
  virtual void summonAVP();
  virtual void abortAVP();
  virtual void exitAVP();


private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
  QThread* thread_;
  //ros::Subscriber sub_trigger_hz;
  RosNode* pRosNode;
};

}  // namespace rqt_plugin_cpp
#endif  // RQT_EXAMPLE_CPP_MY_PLUGIN_H
