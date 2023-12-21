#!/usr/bin/env python

from FSM_controller import StateMachine

from FSM_nodes.initial_detection import initial_detection

import rospy


rospy.init_node('data_handler', anonymous=True)
rospy.sleep(1)


if __name__ == "__main__":
	m = StateMachine()
	m.add_state("INITIAL DETECTION", initial_detection)
	#m.add_state("APPROACH TARGET", approach_target)
	#m.add_state("POINTING ROBOT ARM", pointing_robot_arm)
	#m.add_state("APPROACH ROBOT ARM", approach_robot_arm)
	m.add_state("FINISH", None, end_state=1)
	
	m.set_start("INITIAL DETECTION")
	m.run()
