#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    print("msg received")
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def HuskyHighlevelController():
    print("subscriber")
    rospy.init_node('laser_scan_node', anonymous=True)
    rospy.Subscriber('laser_scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    HuskyHighlevelController()