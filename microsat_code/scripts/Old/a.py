#! /usr/bin/env python3

import rospy
import rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import numpy as np
from std_msgs.msg import Header
def my_publisher_control():
    
    rospy.init_node("Control_python_node")
   
    control_publisher = rospy.Publisher('/robot/arm/scaled_pos_traj_controller/command',JointTrajectory, queue_size = 10)
    
    msg = JointTrajectory()
    msg.header = Header()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = '' 
    msg.joint_names = ['robot_arm_elbow_joint', 'robot_arm_shoulder_lift_joint', 'robot_arm_shoulder_pan_joint', 'robot_arm_wrist_1_joint', 'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.8, 4.8, -1.8, -1.0 , 1.8]
    point.velocities = []
    point.accelerations = []
    point.effort = []
    point.time_from_start = rospy.Duration(1)
    
    msg.points.append(point)
    
    rospy.loginfo(msg)
    
    control_publisher.publish(msg)
    
if __name__ == '__main__':
    try:
    	my_publisher_control()
    except rospy.ROSInterruptException:
    	rospy.loginfo("error")
    	pass
    	

