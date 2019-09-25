#!/usr/bin/python2

'''
   The node loads csv wp to publish once lane waypoint array for a path follower e.g. purepursuit
   P.Phairatt 20/03/19 Parkopedia
'''

import os
import math
import csv
import numpy as np
from autoware_msgs.msg import LaneArray, Lane, Waypoint
import matplotlib.pyplot as plt
import rospy
import tf
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, String



CSV_HEADER = ['X', 'Y', 'Z', 'YAW', 'V']

class WaypointLoader(object):
    def __init__(self):
        # init ros
        rospy.init_node('waypoint_loader', log_level=rospy.INFO)
        self.lane_waypoints_topic = rospy.get_param('~lane_waypoints_topic', "/lane_waypoints_array")
        self.file_path = rospy.get_param('~path')
        self.pub = rospy.Publisher(self.lane_waypoints_topic, LaneArray, queue_size=1, latch=True)
        self.sub_wp_enable = rospy.Subscriber("/route_csv", String, self.wp_callback)
        # spin loop
        rospy.spin()

    # request to publish lane waypoints when it is loaded
    def wp_callback(self, message):
        # load/publish lane wp array
        if(str(message.data)):
            file = str(self.file_path) + "/" + str(message.data)
            print file
            self.new_waypoint_loader(file)
            self.publish(self.waypoints)
        else:
            rospy.logerr("Filename empty!")


    def new_waypoint_loader(self, path):
        if os.path.isfile(path):
            self.waypoints = self.load_waypoints(path)
            rospy.loginfo('Waypoint Loaded: %s', path)
        else:
            rospy.logerr('%s is not a file', path)

    def quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0., 0., yaw)

    def load_waypoints(self, fname):
        waypoints = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                p = Waypoint()
                # filling pose
                p.pose.pose.position.x = float(wp['X'])
                p.pose.pose.position.y = float(wp['Y'])
                p.pose.pose.position.z = float(wp['Z'])
                # filling yaw in Quaternion
                q = self.quaternion_from_yaw(float(wp['YAW']))
                p.pose.pose.orientation = Quaternion(*q)
                # filling linear v
                p.twist.twist.linear.x = float(wp['V'])
                # filling direction forward=1 (v >= 0), reverse=0 (v < 0)
                p.direction = 1 if p.twist.twist.linear.x >= 0 else 0
                # make waypoints[] for LaneArray
                waypoints.append(p)
        return waypoints

    def publish(self, waypoints):
        # make 1 lane
        lane = Lane()
        lane.header.frame_id = 'map'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        # append a lane
        lane_array = LaneArray()
        lane_array.lanes.append(lane)
        self.pub.publish(lane_array)


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint loader node.')
