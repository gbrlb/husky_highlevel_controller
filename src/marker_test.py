#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
        type=Marker.TEXT_VIEW_FACING,
        id=0,
        lifetime=rospy.Duration(0),
        pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(w=1, x=0, y=0, z=0)),
        scale=Vector3(0.06, 0.06, 0.06),
        header=Header(frame_id='base_link'),
        color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
        text=text)
    marker_publisher.publish(marker)

def make_marker(marker_type, scale, position):
    # make a visualization marker array for the occupancy grid
    marker = Marker()
    marker.header.frame_id = '/base_laser'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'marker_test_%d' % marker_type
    marker.id = 0
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0 # Don't forget to set the alpha!
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

def main():
    rospy.init_node('MARKER')
    # wait_for_time()

    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)
    show_text_in_rviz(marker_publisher, 'Marker Demo')
    
    scale = Vector3(.1,.1,3)
    position = 
    # marker_publisher.publish(make_marker(Marker.SPHERE,   scale, 1, .5, .2, .3))
    marker_publisher.publish(make_marker(Marker.CYLINDER, scale))
    # marker_publisher.publish(make_marker(Marker.CUBE,     scale, .2, 1, .5, .3))
    # marker_publisher.publish(make_marker(Marker.ARROW,    scale, 1, 1, 1, .5))

if __name__ == '__main__':
    main()