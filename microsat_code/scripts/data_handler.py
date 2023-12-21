#!/usr/bin/env python

import rospy
import threading
from actionlib_msgs.msg import GoalStatusArray


class DataHandler:
	def __init__(self):
		self.messages = {}
		self.lock = threading.Lock()

	def subscribe_to_topic(self, topic, msg_type):
		rospy.Subscriber(topic, msg_type, self.general_callback, callback_args=topic)

	def general_callback(self, msg, topic):
		with self.lock:
		    self.messages[topic] = msg

	def get_latest_message(self, topic):
		with self.lock:
		    return self.messages.get(topic, None)
		    
	@property
	def status_text(self):
		status_message = self.get_latest_message('/robot/move/status')
		# rospy.loginfo("Received message on: %s", status_message)
		if status_message and status_message.status_list:
			# Assuming the message of interest is the first in the list
			return status_message.status_list[-1].text
		return None
		
	@property
	def camera_matrix(self):
		camera_data = self.get_latest_message('/robot/front_rgbd_camera/color/image_raw')
		if camera_data:
			return camera_data
		return None


if __name__ == '__main__':
	handler = DataHandler()
	handler.subscribe_to_topic('/robot/move/status', GoalStatusArray)
	rospy.sleep(1)
	rospy.loginfo(handler.status_text)


