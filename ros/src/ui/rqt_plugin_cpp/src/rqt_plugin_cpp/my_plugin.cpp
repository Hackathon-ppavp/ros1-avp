/*
  Parkopedia 2019
  Author: Punnu Phairatt
*/


#include "rqt_plugin_cpp/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QtGui>
#include <iostream>
#include <QTimer>
#include <ctime>
#include <boost/filesystem.hpp>

namespace rqt_plugin_cpp
{
/*
 *     Main UI
*/

MyPlugin::MyPlugin(): rqt_gui_cpp::Plugin(), widget_(0)
{
  // Constructor is called first before initPlugin function
  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

MyPlugin::~MyPlugin()
{
  delete pRosNode;
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{

  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // icon banner
  std::string file = getenv("HOME") + std::string("/ppavp-datacapture/ros_ws/src/rqt_plugin/rqt_plugin_cpp/resources/images/parkopediacom-wide.jpg");
  QString filepath = QString::fromStdString(file);
  QPixmap pix(filepath);
  ui_.label_logo->setPixmap(pix);

  // add widget to the user interface
  context.addWidget(widget_);
  // this will make toggle work
  ui_.parkAVP->setCheckable(false);
  ui_.summonAVP->setCheckable(false);
  ui_.abortAVP->setCheckable(false);
  ui_.exitAVP->setCheckable(false);
  // add gui event connection
  connect(ui_.parkAVP, SIGNAL(clicked(bool)), this, SLOT(parkAVP()));
  connect(ui_.summonAVP, SIGNAL(clicked(bool)), this, SLOT(summonAVP()));
  connect(ui_.abortAVP, SIGNAL(clicked(bool)), this, SLOT(abortAVP()));
  connect(ui_.exitAVP, SIGNAL(clicked(bool)), this, SLOT(exitAVP()));

  // self-contained ros node (Experiment only, will be changed to a smart pointer)
  pRosNode = new RosNode(ui_);

}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

void MyPlugin::parkAVP()
{
  QProcess process;
  // confirm user cmd
  if (QMessageBox::Yes == QMessageBox::question(widget_,
                                                tr("AVP-DEMO"),
                                                tr("Do you want to PARK?")))
  {
    // publish request
    pRosNode->park();
    // display ui state
    ui_.deviceState->document()->setPlainText("PARKING");
  }
}

void MyPlugin::summonAVP()
{
  QProcess process;
  // confirm user cmd
  if (QMessageBox::Yes == QMessageBox::question(widget_,
                                                tr("AVP-DEMO"),
                                                tr("Do you want to SUMMON?")))
  {
    // publish request
    pRosNode->summon();
    // display ui state
    ui_.deviceState->document()->setPlainText("SUMMONING");
  }
}

void MyPlugin::abortAVP()
{
  QProcess process;
  // confirm user cmd
  if (QMessageBox::Yes == QMessageBox::question(widget_,
                                                tr("AVP-DEMO"),
                                                tr("Do you want to ABORT?")))
  {
    // publish request
    pRosNode->abort();
    // display ui state
    ui_.deviceState->document()->setPlainText("ABORT");
  }
}


void MyPlugin::exitAVP()
{
  if (QMessageBox::Yes == QMessageBox::question(widget_,
                                                tr("AVP-DEMO"),
                                                tr("Do you want to EXIT?")))
  {
    //display ui state
    ui_.deviceState->document()->setPlainText("EXIT..");
    //stop devices and all nodes
    system("rosnode kill -a");
    //stop roscore
    system("pkill -f /opt/ros");       // gentle way
  }
}


}  // namespace rqt_plugin_cpp


PLUGINLIB_DECLARE_CLASS(rqt_plugin_cpp, MyPlugin, rqt_plugin_cpp::MyPlugin, rqt_gui_cpp::Plugin) //kinetic
//PLUGINLIB_EXPORT_CLASS(rqt_plugin_cpp::MyPlugin, rqt_gui_cpp::Plugin) //melodic
