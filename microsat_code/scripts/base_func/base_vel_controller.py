#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from robotnik_navigation_msgs.msg import MoveActionGoal, MoveGoal
from geometry_msgs.msg import Pose2D, Twist, Vector3

def publish_move_goal(input_base):
    rospy.init_node('move_goal_publisher')

    pub = rospy.Publisher('/robot/move/goal', MoveActionGoal, queue_size=10)


    rospy.sleep(0.5)

    move_action_goal = MoveActionGoal()

    # Set the header
    move_action_goal.header = Header()
    move_action_goal.header.stamp = rospy.Time.now()
    move_action_goal.header.frame_id = "base_link"  # Adjust as necessary

    # Set the goal ID
    move_action_goal.goal_id = GoalID()
    move_action_goal.goal_id.stamp = rospy.Time.now()
    move_action_goal.goal_id.id = "move_goal_" + str(rospy.Time.now())  # Unique ID for the goal

    # Set the MoveGoal
    move_goal = MoveGoal()

    # Set the goal position and orientation
    move_goal.goal = Pose2D(x=input_base["goal"][0], y=input_base["goal"][1], theta=input_base["goal"][2])  # Adjust x, y, theta as necessary

    # Set the maximum velocity
    move_goal.maximum_velocity = Twist()
    move_goal.maximum_velocity.linear = Vector3(x=input_base["max_lin"][0], y=input_base["max_lin"][1], z=input_base["max_lin"][2])  # Adjust linear velocity as necessary
    move_goal.maximum_velocity.angular = Vector3(x=input_base["max_ang"][0], y=input_base["max_ang"][1], z=input_base["max_ang"][2])  # Adjust angular velocity as necessary

    move_action_goal.goal = move_goal

    # Publish the message
    pub.publish(move_action_goal)
    rospy.loginfo("Published MoveActionGoal")

if __name__ == '__main__':
    try:
        input_base = {"goal" : [-0.1, 0.0, 0.0],
        		"max_lin" : [0.5, 0.0, 0.0],
        		"max_ang" : [0.0, 0.0, 0.2]}
        publish_move_goal(input_base)
    except rospy.ROSInterruptException:
        pass

