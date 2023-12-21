#! /usr/bin/env python3

import rospy

from std_msgs.msg import String  # Replace with actual package and message type

def callback(data):
    # This function will be triggered when new data is received
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)  # Adjust how you process the data

def listener():
    # Initialize the ROS Node
    rospy.init_node('robot_state_listener', anonymous=True)

    # Create a subscriber to the topic
    rospy.Subscriber("/robot/arm/robot_state_publisher", String, callback)  # Replace [MessageType] with actual message type

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()

