#!/usr/bin/env python

import rospy
import uuid
from actionlib_msgs.msg import GoalID
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

def send_follow_joint_trajectory_action_goal():
    # Initialize the ROS node
    rospy.init_node('follow_joint_trajectory_action_goal_sender', anonymous=True)

    # Create a publisher to send FollowJointTrajectoryActionGoal messages
    pub = rospy.Publisher('/robot/arm/scaled_pos_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

    # Wait for the publisher to connect to subscribers
    rospy.sleep(1)

    # Create a new FollowJointTrajectoryActionGoal message
    action_goal = FollowJointTrajectoryActionGoal()

    # Set the header
    action_goal.header = Header()
    action_goal.header.seq = 0  # Sequence number
    action_goal.header.stamp = rospy.Time.now()  # Current time
    action_goal.header.frame_id = ""  # Replace with your frame_id

    # Set the goal_id (unique identifier for this goal)
    action_goal.goal_id = GoalID()
    action_goal.goal_id.stamp = rospy.Time.now()
    action_goal.goal_id.id = str(uuid.uuid4())  # Unique ID for the goal

    # Set up the FollowJointTrajectoryGoal
    action_goal.goal = FollowJointTrajectoryGoal()

    # Create a JointTrajectory message
    trajectory = JointTrajectory()
    trajectory.header = Header()
    trajectory.header.seq = 0  # Sequence number for the trajectory
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = ""  # Replace with the appropriate frame_id

    # Set the joint names
    trajectory.joint_names = ['robot_arm_elbow_joint', 'robot_arm_shoulder_lift_joint', 'robot_arm_shoulder_pan_joint', 'robot_arm_wrist_1_joint', 'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint']  # Replace with actual joint names

    # Create and add JointTrajectoryPoints
    point = JointTrajectoryPoint()
    point.positions = [0.25, -1.88, 1.5, -1.38, 0.12 , 1.8]  # Replace with desired positions
    point.velocities = []  # Replace with desired velocities
    point.accelerations = []  # Replace with desired accelerations
    point.effort = []  # Replace with desired effort
    point.time_from_start = rospy.Duration(10)  # Time to reach this point
    trajectory.points.append(point)

    # Assign the trajectory to the action goal
    action_goal.goal.trajectory = trajectory

    # Define path and goal tolerances (optional, based on your application needs)
    # Example: action_goal.goal.path_tolerance = [JointTolerance(name='joint1', position=0.1, velocity=0.1, acceleration=0.1)]

    # Define goal time tolerance (optional)
    action_goal.goal.goal_time_tolerance = rospy.Duration(0.5)  # Example: 0.5 seconds

    # Send the action goal
    pub.publish(action_goal)
    rospy.loginfo("Sent FollowJointTrajectoryActionGoal message.")

if __name__ == '__main__':
    try:
        send_follow_joint_trajectory_action_goal()
    except rospy.ROSInterruptException:
        pass

