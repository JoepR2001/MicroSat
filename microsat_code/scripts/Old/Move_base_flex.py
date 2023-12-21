#!/usr/bin/env python

import rospy
from robotnik_navigation_msgs.msg import RobotnikMoveBaseFlexActionGoal, RobotnikMoveBaseFlexGoal, RobotnikMoveBaseFlexGoalAction
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID

def publish_robotnik_goal():
	rospy.init_node('robotnik_goal_publisher')

	pub = rospy.Publisher('robot/robotnik_move_base_flex/goal', RobotnikMoveBaseFlexActionGoal, queue_size=10)

	#while pub.get_num_connections() == 0:
	rospy.sleep(0.5)

	action_goal = RobotnikMoveBaseFlexActionGoal()

	action_goal.header = Header()
	action_goal.header.stamp = rospy.Time.now()
	action_goal.header.frame_id = "base_link"

	action_goal.goal_id = GoalID()
	action_goal.goal_id.stamp = rospy.Time.now()
	action_goal.goal_id.id = "some_unique_id"

	flex_goal_action = RobotnikMoveBaseFlexGoalAction()

	goal_pose = PoseStamped()
	goal_pose.header.frame_id = "base_link"
	goal_pose.pose = Pose(Point(0.5, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
	flex_goal_action.goal_target_pose = [goal_pose]

	action_goal.goal = RobotnikMoveBaseFlexGoal()
	action_goal.goal.goal = flex_goal_action

	# Example controller, planner, and recovery_behaviors (customize as needed)
	#action_goal.goal.controller = ["controller_name"]
	#action_goal.goal.planner = ["planner_name"]
	#action_goal.goal.recovery_behaviors = ["recovery_behavior_name"]

	# Example goal_tolerance (customize as needed)
	tolerance_pose = Pose(Point(0.1, 0.1, 0.1), Quaternion(0.0, 0.0, 0.0, 1.0))
	#action_goal.goal.goal_tolerance = [tolerance_pose]

	# Example max_vel (customize as needed)
	action_goal.goal.goal.max_vel = 0.1

	# Publish the message
	pub.publish(action_goal)
	rospy.loginfo("Published RobotnikMoveBaseFlexActionGoal")

if __name__ == '__main__':
    try:
        publish_robotnik_goal()
    except rospy.ROSInterruptException:
        pass

