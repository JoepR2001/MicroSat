#! /usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF

rospy.init_node("joint_name_reader")

urdf = URDF.from_parameter_server()

joint_name = [joint.name for joint in urdf.joints]

rospy.loginfo("Joint Names: %s", joint_name)

rospy.spin()
