#!/usr/bin/env python

from data_handler import DataHandler
from actionlib_msgs.msg import GoalStatusArray
import rospy

"""
This file is the approach robot arm node

It consists of the following functionalities:
- Move arm along axis to target
"""

handler = DataHandler()
handler.subscribe_to_topic('/robot/move/status', GoalStatusArray)

rospy.sleep(1)

def approach_arm():

    rospy.sleep(1)

    while handler.status_text != "Goal has been reached":
        rospy.loginfo(f"ERROR, Current status: {handler.status_text}")
        rospy.sleep(1)

    return "FINISH"


if __name__ == "__main__":
    pass
