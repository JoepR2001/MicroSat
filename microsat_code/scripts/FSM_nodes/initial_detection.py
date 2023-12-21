#!/usr/bin/env python

from base_func.base_pos_controller import publish_move_goal
from data_handler import DataHandler
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import cv2
import matplotlib.pyplot as plt

"""
This file is the initial detection node

It consists of the following functionalities:
- Target detection algorithm - TODO
- Rotate vehicle delta theta - PROGRESS
"""

handler = DataHandler()
handler.subscribe_to_topic('/robot/move/status', GoalStatusArray)
handler.subscribe_to_topic('/robot/front_rgbd_camera/color/image_raw', Image)

rospy.sleep(1)

bridge = CvBridge()

def rotate_base(theta):
	input_base = {
		"goal" : 	[0.0, 0.0, theta],
		"max_lin" : 	[0.0, 0.0, 0.0],
		"max_ang" :	[0.0, 0.0, 0.2]
	}
	publish_move_goal(input_base)
	rospy.sleep(1)
	
	while handler.status_text != "Goal has been reached":
		rospy.loginfo(f"ERROR, Current status: {handler.status_text}")
		rospy.sleep(1)


def initial_detection():
	i = 0
	bridge = CvBridge()
	
	try:
		cv_image_before = bridge.imgmsg_to_cv2(handler.camera_matrix, "bgr8")
	except Exception as e:
		rospy.logerr("CvBridge Error: {0}".format(e))
		return

	plt.figure(figsize=(10, 8))
	plt.imshow(cv2.cvtColor(cv_image_before, cv2.COLOR_BGR2RGB))
	plt.title("Image Before Moving")
	plt.axis('off')
	plt.show()
	rospy.loginfo(f"Image before moving processed {cv_image_before}")
	
	while i < 2:
		rospy.loginfo(f"{i}")
		rotate_base(-0.05)
		i += 1
		
	try:
        	cv_image_after = bridge.imgmsg_to_cv2(handler.camera_matrix, "bgr8")
	except Exception as e:
        	rospy.logerr("CvBridge Error: {0}".format(e))
        	return
	
	plt.figure(figsize=(10, 8))
	plt.imshow(cv2.cvtColor(cv_image_after, cv2.COLOR_BGR2RGB))
	plt.title("Image After Moving")
	plt.axis('off')
	plt.show()
	rospy.loginfo("Image after moving processed")
		
	return "FINISH"
	

if __name__ == "__main__":
	pass
