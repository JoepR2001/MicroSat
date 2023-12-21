#! /usr/bin/env python3
import subprocess
import shlex
import rospy
from geometry_msgs.msg import Twist
import math
import time

def calculate_velocities(angle, velocity):
    # Assuming a simple relationship between angle, distance, and velocities
    linear_velocity = [math.cos(angle) * velocity, math.sin(angle) * velocity, 0.0]
    angular_velocity = [0.0, 0.0, 0.0]  # Adjust the angular velocity as needed
    return linear_velocity, angular_velocity

def publish_velocity_cmd(angle, velocity):
    # Calculate velocities based on angle and velocity
    linear_velocity, angular_velocity = calculate_velocities(angle, velocity)

    # Define the ROS topic and message type
    topic = "/robot/move/cmd_vel"
    message_type = "geometry_msgs/Twist"

    # Create a Twist message
    twist_msg = Twist()
    twist_msg.linear.x = linear_velocity[0]
    twist_msg.linear.y = linear_velocity[1]
    twist_msg.linear.z = linear_velocity[2]
    twist_msg.angular.x = angular_velocity[0]
    twist_msg.angular.y = angular_velocity[1]
    twist_msg.angular.z = angular_velocity[2]

    # Convert the message to a string
    twist_str = str(twist_msg)

    # Use subprocess to run the rostopic pub command
    command = f"rostopic pub {topic} {message_type} -r 3 -- '{twist_str}'"

    # Run the command in a subprocess
    subprocess.run(shlex.split(command))

if __name__ == "__main__":
    rospy.init_node("velocity_publisher")

    # Input parameters: angle in radians, velocity in m/s, distance in meters
    angle = math.radians(45)
    velocity = 0.01
    distance = 0.01
    duration = distance / velocity

    # Move forward
    publish_velocity_cmd(angle, velocity)

    # Sleep for the specified duration
    time.sleep(duration)

    # Stop
    publish_velocity_cmd(0, 0)
    
    time.sleep(duration)

    # Move in the opposite direction (backward)
    backward_angle = angle + math.pi  # Opposite direction
    publish_velocity_cmd(backward_angle, velocity, duration)
