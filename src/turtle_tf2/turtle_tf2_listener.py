#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
from visualization_msgs.msg import Marker

rospy.loginfo('Publishing example line')

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 1:
        pass

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

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')
    # wait_for_time()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    turtle_name = rospy.get_param('turtle', 'turtle2')
    spawner(4, 2, 0, turtle_name)

    # Publishers
    turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)


    # pose = geometry_msgs.msg.Pose(1, 2, 3)
    

    pub_turtle_path = rospy.Publisher('~turtle_path', Marker, queue_size=10)

    marker = make_marker(Marker.LINE_STRIP,
                         frame_id='world',
                         scale=[0.03, 0, 0],
                         color=[1, 1, 0, 1])

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        turtle_vel.publish(msg)

        # second point
        if msg.linear.x > 0.001:
            try:
                path = tfBuffer.lookup_transform('world', turtle_name, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            marker.points.append(path.transform.translation)
            pub_turtle_path.publish(marker)

        rate.sleep()