#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    dist_min = min(data.ranges)
    # rospy.loginfo(rospy.get_caller_id() + 'range_min %s', data.range_min)
    # rospy.loginfo(rospy.get_caller_id() + 'range_max %s', data.range_max)
    rospy.loginfo('distancia minima %s', dist_min)
    # if dist_min > data.range_min:
    #     rospy.loginfo('distancia minima %s', dist_min)
    # elif dist_min > data.range_max:
    #     rospy.loginfo('distancia minima %s', range_max)

def HuskyHighlevelController():
    # rospy.init_node('HuskyHighlevelController_node', anonymous=True)
    
    print("subscriber")

    global_param = rospy.get_param("from_launch", "NO Global parameter")
    param1 = rospy.get_param("~topic", "no_topic")
    param2 = rospy.get_param("~queue_size", "no")
    relative = rospy.get_param("from_launch_node", "NO relative parameter")

    print(global_param)
    print(relative)
    print(param1)
    print(param2)

    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    HuskyHighlevelController()