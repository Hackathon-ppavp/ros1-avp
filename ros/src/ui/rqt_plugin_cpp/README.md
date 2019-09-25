## USAGE

Start roscore
```
$ roscore
```

Source the workspace otherwise the plugin won't be loaded!
```
$ source devel/setup.bash
```

Start rqt_gui plugin
```
$ rosrun rqt_gui rqt_gui --perspective-file "folder-to-perspective-file/gui_layout_lidarcam.perspective"
$ rosrun rqt_gui rqt_gui --perspective-file "/home/punnu/catkin_ws_qt/src/rqt_plugin_cpp/resources/avp-demo.perspective"
```
