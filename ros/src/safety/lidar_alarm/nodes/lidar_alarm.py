#!/usr/bin/python2

import rospy
import roslib
import numpy as np
import matplotlib.pyplot as plt
import cv2

from nav_msgs.msg import OccupancyGrid

class Lidar_alarm:
    def __init__(self):
        self.resolution = rospy.get_param('~resolution', 0.1)      # meter
        self.cell_width = rospy.get_param('~cell_width', 500)      # unit
        self.cell_height = rospy.get_param('~cell_height', 300)    # unit
        self.safety_x = rospy.get_param('~safety_x', 3.0)          # lidar x-axis meter default    3.0 from the lidar origin axis-x (forward or rear depend on the moving direction)
        self.safety_y = rospy.get_param('~safety_y', 1.0)          # lidar y-axis meter default +- 1.0 from the lidar origin axis-y
        self.sub = rospy.Subscriber('/semantics/costmap_generator/occupancy_grid', OccupancyGrid, self.CostmapCallback)     # accept namespace mapping with ' ' topic

    def CostmapCallback(self, msg):
        if msg.info.resolution: self.resolution = msg.info.resolution
        if msg.info.width: self.cell_width = int(msg.info.width)
        if msg.info.height: self.cell_height = int(msg.info.height)
        np_arr = np.array(msg.data).reshape(self.cell_height, self.cell_width)
        # searching grid within safety_width x safety_height
        n_row = int(self.safety_x/ self.resolution)           # grid unit
        n_col = int(self.safety_y/ self.resolution)           # grid unit
        grid_center_row =  int(self.cell_width/2)            # lidar centre in rows
        grid_center_col =  int(self.cell_height/2)             # lidar centre in cols

        # TODO: Only look forward for now
        danger_zone = np_arr[grid_center_row-n_row:grid_center_row+n_row, grid_center_col:grid_center_col+n_col]
        obstruction = np.where(danger_zone > 0)
        #if(np.any(obstruction)): print("Brake Brake Brake")

        # display data with bounding box added
        img = 2.55*np_arr
        cv2.rectangle(img, (grid_center_row-n_row, grid_center_col-n_col), (grid_center_row+n_row, grid_center_col+n_col), 255, 2)
        cv2.namedWindow('OCCcostmap',cv2.WINDOW_NORMAL)
        cv2.imshow('OCCcostmap', img) # remap probability to 255 image scale
        cv2.waitKey(2)


if __name__ == '__main__':
    '''Initializes and cleanup ros node'''
    rospy.init_node('lidar_alarm_node', anonymous=True)
    ld = Lidar_alarm()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down lidar_alarm_node"
    cv2.destroyAllWindows()
