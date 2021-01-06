#!/usr/bin/env python

import rospy

def param_ex1():

    print('========set/get parameters=======')

    rospy.set_param('a_string', 'baz')
    rospy.set_param('~private_int', 2)
    rospy.set_param('list_of_floats', [1., 2., 3., 4.])
    rospy.set_param('bool_True', True)
    rospy.set_param('gains', {'p': 1, 'i': 2, 'd': 3})

    a_string = rospy.get_param('a_string', 'ERROR')
    private_int = rospy.get_param('~private_int', 'ERROR' )
    list_of_floats = rospy.get_param('list_of_floats', 'ERROR' )
    bool_True = rospy.get_param('bool_True', 'ERROR' )
    gains = rospy.get_param('gains', 'ERROR' )

    rospy.loginfo('param %s = %s', rospy.resolve_name('a_string'), a_string)
    rospy.loginfo('param %s = %s', rospy.resolve_name('~private_int'), private_int)
    rospy.loginfo('param %s = %s', rospy.resolve_name('list_of_floats'), list_of_floats)
    rospy.loginfo('param %s = %s', rospy.resolve_name('bool_True'), bool_True)
    rospy.loginfo('param %s = %s', rospy.resolve_name('gains'), gains)


def testparam():

    print("======Parameters from diferent sources example======")

    global_param_from_launch = rospy.get_param("global_param_from_launch", "ERROR")
    reative_param_from_launch = rospy.get_param("~reative_param_from_launch", "ERROR")
    rospy.loginfo('Parameter %s = %s', rospy.resolve_name('global_param_from_launch'), 
    global_param_from_launch)
    rospy.loginfo('Parameter %s = %s', rospy.resolve_name('~reative_param_from_launch'), 
    reative_param_from_launch)

    # from *.yaml file
    yaml_param_golbal = rospy.get_param('yaml_param', "no_topic")
    yaml_param_relative = rospy.get_param('~yaml_param', "no_topic")
    rospy.loginfo('Parameter %s = %s', rospy.resolve_name('yaml_param_golbal'), yaml_param_golbal)
    rospy.loginfo('Parameter %s = %s', rospy.resolve_name('~yaml_param_relative'), yaml_param_relative)

    rospy.spin()

def print_param_list():
    print("======rosparam list======")
    param_names = rospy.get_param_names()
    for x in param_names:
        print(x)

if __name__ == '__main__':
    rospy.init_node('rosparam example')
    param_ex1()
    testparam()
    print_param_list()