#!/usr/bin/env python


from FSM_controller import StateMachine
from arm_func.align_camera_balloon import calculate_movement
from data_handler import DataHandler
from actionlib_msgs.msg import GoalStatusArray
import rospy

"""
This file is the pointing robot arm node

It consists of the following functionalities:
- Allign camera with target
"""

handler = DataHandler()
handler.subscribe_to_topic('/robot/move/status', GoalStatusArray)

rospy.sleep(1)

#placeholder moving arm if ball is not detected anymore after moving base

def allign_arm():
    distance_to_move = 10
    #placeholder target location data input from balloon detector
    while distance_to_move < 0.01:
        target_location = calculate_movement(target_distance, target_center_x, target_center_y)
        arm_moving_angle = target_location[1]
        distance_to_move = target_location[2]
        #placeholder for moving arm in plane with a distance and angle as specified above
    rospy.sleep(1)

    while handler.status_text != "Goal has been reached":
        rospy.loginfo(f"ERROR, Current status: {handler.status_text}")
        rospy.sleep(1)

    return "APPROACH ROBOT ARM"


if __name__ == "__main__":
    pass
