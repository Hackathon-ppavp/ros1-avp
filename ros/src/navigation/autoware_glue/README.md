## lane_way_points_loader 
**1.0 Function**   
Read x,y,z,yaw,v from a csv file  

```
-79.1842799595	-49.3224293647	0	0	2.78
     x               y          z  yaw  speed

```

Note: 
1- x,y,z,yaw is in the map coordinate, v is a vehicle speed in the vehicle frame  
2- At the moment, Autoware purepursuit does not use the orientation (yaw), so set to any numbers you want in a csv file.   
3- Prior to reaching the final waypoint, it is recommend to set the velocity that gradually reduced to zero to prevent a sudden stop motion.  	

```
-79.1842799595	-49.3224293647	0	0	2.78
-79.2062233956	-48.3017339202	0	0	2.78
-79.2281740678	-47.2811012592	0	0	2.78
-79.250172332	-46.258299579	0	0	2.78
-79.272186264	-45.2350817203	0	0	2.78
-79.2941934641	-44.2120615232	0	0	2.78
-79.3162111177	-43.1887839036	0	0	2.78
-79.3382658912	-42.1636742349	0	0	2.78
-79.3603135051	-41.1388861695	0	0	2.78
-79.3823375881	-40.1154195728	0	0	2.5
-79.4043321993	-39.0932321279	0	0	2
-79.4263225763	-38.0712364045	0	0	1.5
-79.4483606607	-37.0471797964	0	0	1
-79.4704254709	-36.0221559792	0	0	0
```

4- Config Autoware PurePursuit to use this set of speeds at each waypoint.  


**2.0 Node**  
2.1 Subscribe topics  
wp_enable (std_msgs/Bool)  
	Message callback that publish the lane waypoints array   

2.2 Publish topics  
lane_waypoints_array (autoware_msgs/LaneArray)  
	Autoware lane waypoints array that contains x,y,z,yaw,v   

2.3 Parameters  
path  
	csv file path  

velocity (**Deprecate**)  
	

## track_waypoints  
**1.0 Function**
Publish the next waypoints in front of the vehicle's current location. To speed up the search and to avoid a jump in cross paths, the search only looks for a portion of waypoints within +- of lookahead waypoints for the next closest waypoint from the current waypoint index.   

This node also tracks the final waypoint. It reports the end of waypoint follower if the goal reach.   

**2.0 Node**  
2.1 Subscribe topics  
current_pose (geometry_msgs/PoseStamped)   
	Current vehicle position and orientation  

lane_waypoints_array (autoware_msgs/LaneArray)  
	Global path- Autoware lane waypoints array message  

2.2 Publish topics  
final_waypoints (autoware_msgs/Lane)  
	Waypoints from the current vehicle location onward  

final_waypoints_reach (std_msg/Bool)  
	Final waypoint is reached

2.3 Parameters  
lookahead_wp  
	Number of lookahead waypoints  



## autoware_translator
**1.0 Overview**
Convert autoware /vehicle_cmd message to /prius for the Gazebo simulator.  
**2.0 Node**  
2.1 Subscribe topics  

2.2 Publish topics  

2.3 Parameters  

## repeater
// TODO


