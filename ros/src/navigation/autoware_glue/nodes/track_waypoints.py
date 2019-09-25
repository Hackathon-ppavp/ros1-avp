#!/usr/bin/python2
'''
   This node is used in conjunction with waypoint follower e.g. purepursuit or mpc
   It tracks the current vehicle location to extarct the final path the follower as mentioned above.
   In addition, it supports the path with going forward and backward direction in the route by dividing the path
   into segments. This is due to the MPC need the path in forward or backward one at the time for optimisation,
   where as the purepursuit in ok with the mixing. The implementation provides the consistency platform for both.
   P.Phairatt 20/03/19 Parkopedia
'''

import math
import rospy
import numpy as np
from enum import IntEnum
import time
from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import LaneArray, Lane
from std_msgs.msg import Bool, Int32



class TrackingState(IntEnum):
    INIT_WPS = 0
    IN_TRACKING = 1
    REACH_TRACKING = 2
    FINISH_TRACKING = 3


class TrackWaypoints:
    def __init__(self):
        # params
        self.current_pose = None
        self.front_wp_index = 0
        self.closest_wp_index = 0
        self.lookahead_wp = rospy.get_param('~lookahead_wp', 50)
        self.current_pose_topic = rospy.get_param('~current_pose_topic', "/current_pose")
        self.lane_waypoints_topic = rospy.get_param('~lane_waypoints_topic', "/lane_waypoints_array")
        self.final_waypoints_topic = rospy.get_param('~final_waypoints_topic', "/final_waypoints")
        self.base_waypoints_topic = rospy.get_param('~base_waypoints_topic', "/base_waypoints")
        self.final_reach_topic = rospy.get_param('~final_reach_topic', "/final_waypoints_reach")
        self.final_waypoints = Lane()
        #self.base_waypoints = Lane()
        self.lane = Lane()
        self.closest_wp_msg = Int32()
        self.lane_segments = []
        self.current_segment_id = 0
        self.base_waypoints_array = []
        self.current_tracking_state = TrackingState.INIT_WPS

        # sub/pub
        self.sub_current_pose = rospy.Subscriber(self.current_pose_topic, PoseStamped, self.pose_callback)
        self.sub_lane_waypoints = rospy.Subscriber(self.lane_waypoints_topic, LaneArray, self.lane_callback)
        self.pub_final_waypoints = rospy.Publisher(self.final_waypoints_topic, Lane, queue_size=10)
        self.pub_base_waypoints = rospy.Publisher(self.base_waypoints_topic, Lane, queue_size=10)
        self.pub_final_reach = rospy.Publisher(self.final_reach_topic, Bool, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1. / 50.), self.timer_callback)


    def timer_callback(self, event):
        # Entrypoint: do nothing until get the current pose and lane waypoints
        if( (self.current_pose is None) or (len(self.lane.waypoints) == 0) ):
            return

        # initilse the first waypoint if not done so, and request brake off
        if(self.current_tracking_state is TrackingState.INIT_WPS):
            self.pub_final_reach.publish(False)
            self.make_lane_segments()
            self.update_lane_segments(self.current_segment_id)
            self.closest_wp_index = self.find_closest_wp(0, len(self.lane.waypoints))
            self.current_tracking_state = TrackingState.IN_TRACKING

        # Filter final waypoints in the current segment until no more (the final destination reach)
        if(self.current_tracking_state is TrackingState.IN_TRACKING):
            self.final_waypoints = self.front_waypoints()
            if len(self.final_waypoints.waypoints) != 0:
                # pub final waypoints for the motion controller
                self.pub_final_waypoints.publish(self.final_waypoints)

        # reach the segment final waypoint - continue or finish
        if(self.current_tracking_state is TrackingState.REACH_TRACKING):
            # move onto the next segment
            self.current_segment_id += 1
            # reach the destination or keep going
            if(self.current_segment_id > len(self.lane_segments)-1):
                # when reach the destination
                # brake on and stop a purepursuit
                self.pub_final_reach.publish(True)
                self.current_tracking_state = TrackingState.FINISH_TRACKING
            else:
                # there is the next segment to follow
                # momentary stop before changing a direction of travelling e.g. signal to apply a brake on before moving reverse to forward as we normally do
                self.signal_pause(2)
                # load another segment waypoints
                self.update_lane_segments(self.current_segment_id)
                self.front_wp_index = 0
                self.closest_wp_index = 0
                self.current_tracking_state = TrackingState.IN_TRACKING

        # Following accomplish-
        if(self.current_tracking_state is TrackingState.FINISH_TRACKING):
            # reset all the motion controller pid/mpc/pp/zero steering/waypoints
            self.restart_tracking()
            # get back to the beginning - Waiting to restart another journey
            self.current_tracking_state = TrackingState.INIT_WPS

    def pose_callback(self, message):
        self.current_pose = message

    def lane_callback(self, message):
        # copy lane way points array e.g. lane[0], lane[1], lane[2] (we only do one for now)
        self.lane = message.lanes[0]

    def make_lane_segments(self):
        # extract segment waypoints from a 'direction' attribute in the lane wapoints
        direction = []
        for i in range(len(self.lane.waypoints)):
            direction.append(self.lane.waypoints[i].direction)
        # find path segments index before the sign change (direction) happens
        self.lane_segments = np.where(np.diff(direction))[0]
        # append the last length to the list
        self.lane_segments = np.append(self.lane_segments, len(direction)-1)
        # make base_waypoints_array
        start_index = 0
        for segment_index in self.lane_segments:
            base_waypoints = Lane()
            base_waypoints.header = self.lane.header
            end_index = segment_index+1 # note need to +1 to include the lase segment waypoint
            for i in range(start_index, end_index):
                    base_waypoints.waypoints.append(self.lane.waypoints[i])

            self.base_waypoints_array.append(base_waypoints)
            start_index = end_index


    def update_lane_segments(self, segment_id):
        self.pub_base_waypoints.publish(self.base_waypoints_array[segment_id])

    def restart_tracking(self):
        # set variable to initial
        self.front_wp_index = 0
        self.closest_wp_index = 0
        self.lane = Lane()
        self.final_waypoints = Lane()
        self.lane_segments = []
        self.current_segment_id = 0
        self.base_waypoints_array = []
        rospy.loginfo("RESTART TRACKING")

    def find_closest_wp(self, start_index, end_index):
        """
        Finding the closest waypoint index from Euclidean distance: partial and forward search and non-cyclic
        """
        # check bound wp index- no cyclic (only search forward to the end from the current location)
        wp_num = len(self.base_waypoints_array[self.current_segment_id].waypoints)
        start_index = min(wp_num-1, max(0, start_index))
        end_index = min(wp_num, max(0, end_index))

        # closest distance + closest index (path may be overlay because we go forward and backward on the same path)
        current_position = np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y])
        min_distance = 1e10
        min_index = self.closest_wp_index
        #max_jump = 2 # to prevent index jump e.g. 154 to 170 (it should has been 155) because the path is overlaid to each other and index 170 is the closest- don't want that
        for i in range(start_index, end_index):
            wp_index = i
            wp_position = np.array([self.base_waypoints_array[self.current_segment_id].waypoints[wp_index].pose.pose.position.x,
                                    self.base_waypoints_array[self.current_segment_id].waypoints[wp_index].pose.pose.position.y])
            dist_i = np.linalg.norm(current_position - wp_position)
            #diff_i = abs(i-self.closest_wp_index)
            #if (dist_i < min_distance and diff_i < max_jump):
            if (dist_i < min_distance):
                min_distance = dist_i
                min_index = i

        return min_index


    def front_waypoints(self):
        """
        Looking for the next nearest waypoint in front of the current position (The result is the nearest point + 1)
        We are only searching the waypoints around the current position (+ lookahead_wp)
        """
        final_waypoints = Lane()
        wp_num = len(self.base_waypoints_array[self.current_segment_id].waypoints)
        # tracking segmentation
        if(wp_num and (self.current_tracking_state is TrackingState.IN_TRACKING)):
            # find the current closest_wp_index
            self.closest_wp_index = self.find_closest_wp(self.closest_wp_index, self.closest_wp_index + self.lookahead_wp)
            # shift to the next wp target to follow if a vehicle reaches the current self.front_wp_index
            if(self.closest_wp_index >= self.front_wp_index):
                self.front_wp_index = self.closest_wp_index + 1

            rospy.loginfo("closest wp: %d front wp: %d segment wp: %d segment_id: %d", self.closest_wp_index, self.front_wp_index, wp_num-1, self.current_segment_id)
            # move to the next track segment if the vehicle hit the last waypoint in the current segment
            if(self.closest_wp_index >= wp_num-1):
                # update base_waypoints and reset the index to 0 prior to starting the new segment
                self.current_tracking_state = TrackingState.REACH_TRACKING
                return final_waypoints

            # extract final waypoint within this segments
            for i in range(self.front_wp_index, min(self.front_wp_index + self.lookahead_wp, wp_num)):
                final_waypoints.waypoints.append(self.base_waypoints_array[self.current_segment_id].waypoints[i])

        return final_waypoints

    def signal_pause(self, secs):
        self.pub_final_reach.publish(True)
        time.sleep(secs)
        self.pub_final_reach.publish(False)
        time.sleep(secs)





if __name__ == '__main__':
    rospy.init_node('track_waypoints')
    t = TrackWaypoints()
    rospy.spin()
