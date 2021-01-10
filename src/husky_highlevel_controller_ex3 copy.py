#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 1:
        pass

def callback(data):

    dist_min_index = np.argmin(data.ranges)
    dist_min = data.ranges[dist_min_index]
    angle = (360 - dist_min_index) * data.angle_increment
    angle_deg = np.rad2deg(angle)
    dist_x = np.cos(angle) * dist_min
    dist_y = - np.sin(angle) * dist_min

    print("===============================================")
    rospy.loginfo('distancia minima index %s', dist_min_index)
    rospy.loginfo('distancia minima %s', dist_min)
    rospy.loginfo('angle %s, %s', angle, np.rad2deg(angle))
    rospy.loginfo('distancia x %s', dist_x)
    rospy.loginfo('distancia y %s', dist_y)

    ## Marker
    colum_pose = Pose(Point(x=dist_x, y=dist_y, z=0), Quaternion(x=0, y=0, z=0, w=1))
    # colum_maker = make_marker(marker_type=Marker.CYLINDER, 
    #                         frame_id='/base_laser', 
    #                         pose=colum_pose,
    #                         scale=Vector3(x=.1, y=.1, z=.1),
    #                         color=std_msgs.msg.ColorRGBA(r=1, g=1, b=0, a=1)):
    # print(culum_pose)

    marker_colum_pub.publish(make_marker(Marker.CYLINDER, frame_id='/base_laser', pose=colum_pose, scale=[.5, .5, 1], color=[0, 1, 0, 1]))

    ## Controlador Proporcional

    loop_rate = rospy.Rate(500)
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
    

    # path = tfBuffer.lookup_transform('base_link', 'odom', rospy.Time())
    # marker.points.append(path.transform.translation)
    # pub_turtle_path.publish(marker)

def make_marker(marker_type=1, 
                frame_id='world', 
                pose=None,
                position=None,
                orientation=None,
                scale=[.1, .1, .1],
                color=[1, 1, 0, 1] # ['r','g','b','a']
                ):

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'marker_%d' % marker_type
    marker.id = 0
    marker.type = marker_type
    marker.action = Marker.ADD

    # pose ['position','orientation']
    if pose is not None:
        marker.pose = pose

    # marker orientaiton ['x','y','z','w']
    if orientation is not None:
        marker.pose.orientation = orientation

    # marker position ['x','y','z']
    if position is not None:
        marker.pose.position = position

    # maker scale Vector3 ['x','y','z']
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    # maker color ColorRGBA ['r','g','b','a']
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3] # Don't forget to set the alpha!
    return marker

def HuskyHighlevelController():
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('HuskyHighlevelController_node', anonymous=True)
    # wait_for_time()

    topic = "/husky_velocity_controller/cmd_vel"
    queue_size = 10

    vel_publisher = rospy.Publisher(topic, Twist)
    marker_colum_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    cmd_vel_msg = Twist()
    kp_linear = 2
    kp_angular = .5


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    maker_Husky_path_pub = rospy.Publisher('Husky_path', Marker, queue_size=5)

    HuskyHighlevelController()