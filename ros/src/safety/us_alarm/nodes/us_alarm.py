#!/usr/bin/python2

import rospy
from std_msgs.msg import Bool
from can_msgs.msg import Frame

class Us_alarm:
    def __init__(self):
        self.us_alarm_topic = rospy.get_param('~alarm_topic', "/us/us_alarm")
        self.sub = rospy.Subscriber('received_messages', Frame, self.UsCallback)
        self.pub_obstruction = rospy.Publisher('self.us_alarm_topic', Bool, queue_size=10)
        self.frontLeft = 0
        self.frontCentre = 0
        self.frontRight = 0
        self.right = 0
        self.left = 0
        self.rearLeft = 0
        self.rearCentre = 0
        self.rearRight = 0

    def UsCallback(self, msg):
        # extract USS CANBus
        if (msg.id == 0x402 ):
    	    tmp = bytearray(msg.data)
    	    self.right = tmp[5]
            self.frontRight = tmp[4]
            self.frontCentre = tmp[3]
            self.frontLeft = tmp[2]
            self.updated = 1
        elif (msg.id == 0x403 ):
    	    tmp = bytearray(msg.data)
            self.left = tmp[5]
            self.rearLeft = tmp[4]
            self.rearCentre = tmp[3]
            self.rearright = tmp[2]
            self.updated = 1

        # gauging within range: front 4 sensors
        if((self.frontCentre > 0 and self.frontCentre <255)):
            self.pub_obstruction.publish(True)
            rospy.loginfo("US Alarm: %d %d %d", self.frontLeft, self.frontCentre, self.frontRight )

if __name__ == '__main__':
    rospy.init_node('us_alarm', anonymous=True)
    u = Us_alarm()
    rospy.spin()
