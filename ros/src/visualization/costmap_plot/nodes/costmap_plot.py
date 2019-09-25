#!/usr/bin/python2

import rospy
import roslib
import numpy as np
import matplotlib.pyplot as plt
import cv2

from nav_msgs.msg import OccupancyGrid

class Costmap_plot:
    def __init__(self):
        self.resolution = rospy.get_param('~resolution', 0.1)      # meter
        self.cell_width = rospy.get_param('~cell_width', 500)      # unit
        self.cell_height = rospy.get_param('~cell_height', 500)    # unit
        self.safety_x = 5.0  # meter default 5.0 from the lidar origin
        self.safety_y = 3.0  # meter default 3.0 from the lidar origin
        self.sub = rospy.Subscriber('realtime_cost_map', OccupancyGrid, self.CostmapCallback)     # accept namespace mapping with ' ' topic

    def CostmapCallback(self, msg):
        if msg.info.resolution: self.resolution = msg.info.resolution
        if msg.info.width: self.cell_width = int(msg.info.width)
        if msg.info.height: self.cell_height = int(msg.info.height)
        np_arr = np.array(msg.data).reshape(self.cell_width, self.cell_height)

        # searching grid within safety_width x safety_height
        row = int(self.safety_y/ self.resolution/2)
        col = int(self.safety_x/ self.resolution/2)
        grid_center_row =  int(self.cell_height/2)
        grid_center_col =  int(self.cell_width/2)

        danger_zone = np_arr[grid_center_row-row:grid_center_row+row, grid_center_col-col:grid_center_col+col]
        obstruction = np.where(danger_zone > 0)
        if(np.any(obstruction)): print("Brake Brake Brake")

        # display data with bounding box added
        img = 2.55*np_arr
        cv2.rectangle(img, (grid_center_col-col, grid_center_row-row), (grid_center_col+col, grid_center_row+row), 255, 2)
        cv2.namedWindow('OCCcostmap',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('OCCcostmap', 1280,1280)
        cv2.imshow('OCCcostmap', img) # remap probability to 255 image scale
        cv2.waitKey(2)


if __name__ == '__main__':
    '''Initializes and cleanup ros node'''
    rospy.init_node('costmap_plot_node', anonymous=True)
    cp = Costmap_plot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down costmap_plot_node"
    cv2.destroyAllWindows()
