#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

def publish_simple_goal():
    rospy.init_node('simple_goal_publisher')

    pub = rospy.Publisher('/robot/robotnik_move_base_flex/simple_goal', PoseStamped, queue_size=10)

    #while pub.get_num_connections() == 0:
    rospy.sleep(0.5)

    # Create the PoseStamped message
    goal_pose_stamped = PoseStamped()

    # Set the header (adjust frame_id and stamp as necessary)
    goal_pose_stamped.header = Header()
    goal_pose_stamped.header.stamp = rospy.Time.now()
    goal_pose_stamped.header.frame_id = "base_link"  # Change as appropriate

    # Set the pose
    goal_pose_stamped.pose.position = Point(x=0.1, y=0.0, z=0.0)  # Move 0.1 meters in x direction
    goal_pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # Publish the message
    pub.publish(goal_pose_stamped)
    rospy.loginfo("Published simple goal with PoseStamped")

if __name__ == '__main__':
    try:
        publish_simple_goal()
    except rospy.ROSInterruptException:
        pass

