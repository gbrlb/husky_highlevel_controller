#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker

def callback(data):
    dist_min_index = np.argmin(data.ranges)
    dist_min = data.ranges[dist_min_index]
    angle = (360 - dist_min_index) * data.angle_increment
    angle_deg = np.rad2deg(angle)
    dist_x = np.cos(angle) * dist_min
    dist_y = -np.sin(angle) * dist_min

    print("===============================================")
    rospy.loginfo('distancia minima index %s', dist_min_index)
    rospy.loginfo('distancia minima %s', dist_min)
    rospy.loginfo('angle %s, %s', angle, np.rad2deg(angle))
    rospy.loginfo('distancia x %s', dist_x)
    rospy.loginfo('distancia y %s', dist_y)

    ## Marker
    scale = Vector3(3,2,1)
    pose = Pose(Point(x=dist_x, y=dist_y, z=0), Quaternion(x=0, y=0, z=0, w=1))
    print(pose)
    marker_publisher.publish(make_marker(Marker.CYLINDER, pose))


    ## Controlador Proporcional

    loop_rate = rospy.Rate(1000)
    print("===== go to ======")
    print('linear:', dist_min)
    print('angular:', np.rad2deg(angle))
    # rospy.loginfo("Moves from:[x={:.2f}, y={:.2f}] to Goal:[x={:.2f}, y={:.2f}]".format(x, y, x_goal, y_goal))
    if dist_min > data.range_min:
        print("range min:", data.range_min)
        cmd_vel_msg.linear.x = dist_min * kp_linear
        cmd_vel_msg.angular.z = angle * kp_angular
        print(cmd_vel_msg.linear.x)
        print(cmd_vel_msg.angular.z)
        vel_publisher.publish(cmd_vel_msg)
        loop_rate.sleep()
    else:
        print("estoy aqui!!!")

def make_marker(marker_type, pose):
    # make a visualization marker array for the occupancy grid
    marker = Marker()
    marker.header.frame_id = '/base_laser'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'marker_test_%d' % marker_type
    marker.id = 0
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = pose
    # marker.pose.position.x = 0
    # marker.pose.position.y = 0
    # marker.pose.position.z = 0
    # marker.pose.orientation.x = 0.0
    # marker.pose.orientation.y = 0.0
    # marker.pose.orientation.z = 0.0
    # marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.1
    marker.color.a = 1.0 # Don't forget to set the alpha!
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker


def HuskyHighlevelController():
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('HuskyHighlevelController_node', anonymous=True)
    topic = "/husky_velocity_controller/cmd_vel"
    queue_size = 10
    vel_publisher = rospy.Publisher(topic, Twist)
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    cmd_vel_msg = Twist()
    kp_linear = 10
    kp_angular = 1
    HuskyHighlevelController()