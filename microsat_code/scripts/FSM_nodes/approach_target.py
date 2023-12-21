#!/usr/bin/env python

from FSM_controller import StateMachine
from base_pos_controller import publish_move_goal
from joint_controller import send_follow_joint_trajectory_action_goal
from data_handler import DataHandler
from actionlib_msgs.msg import GoalStatusArray
import rospy


"""
This file is the approach target node

It consists of the following functionalities:
- Estimate target position (X, Y)
- Move rover towards target positions
"""

handler = DataHandler()
handler.subscribe_to_topic('/robot/move/status', GoalStatusArray)

rospy.sleep(1)


def move_rover(x, y):
	input_base = {
		"goal" : 	[x, y, 0.0],
		"max_lin" : 	[0.5, 0.5, 0.0],
		"max_ang" :	[0.0, 0.0, 0.2]
	}
	publish_move_goal(input_base)
	rospy.sleep(1)
	
	while handler.status_text != "Goal has been reached":
		rospy.loginfo(f"ERROR, Current status: {handler.status_text}")
		rospy.sleep(1)
		
	return "POINTING ROBOT ARM"



if __name__ == "__main__":
	pass
