#!/usr/bin/env python

class StateMachine:
	def __init__(self):
		self.handlers = {}
		self.startState = None
		self.endStates = []

	def add_state(self, name, handler, end_state=0):
		name = name.upper()
		self.handlers[name] = handler
		if end_state:
			self.endStates.append(name)

	def set_start(self, name):
		self.startState = name.upper()

	def run(self):
		try:
			handler = self.handlers[self.startState]
		except:
			raise Exception("must call .set_start() before .run()")
		if not self.endStates:
			raise Exception("at least one state must be an end_state")

		while True:
			(newState) = handler()
			if newState.upper() in self.endStates:
				print("reached ", newState)
				break
			else:
				handler = self.handlers[newState.upper()]


if __name__ == "_main_":
	"""
	def initialization():
	    print("Current state: INITIALIZATION")
	    print("Robot initialized, entering INITIAL DETECTION")
	    return "INITIAL DETECTION"


	def initial_detection():
	    print("Current state: INITIAL DETECTION")
	    print("balloon detected, entering APPROACH TARGET")
	    return "APPROACH TARGET"

FINISH
	def approach_target():
	    print("Current state: APPROACH TARGET")

	    distance = "MEDIUM"

	    if distance == "MEDIUM":
		print("distance medium, entering POINTING ROBOT ARM")

	    if distance != "SMALL":
		print("distance still not small, continuing state APPROACH TARGET")

	    return "POINTING ROBOT ARM"


	def pointing_robot_arm():
	    print("Current state: POINTING ROBOT ARM")
	    print("arm ready, entering APPROACH ROBOT ARM")
	    return "APPROACH ROBOT ARM"


	def approach_robot_arm():
	    print("Current state: APPROACH ROBOT ARM")
	    print("target touched, entering FINISH")
	    return "FINISH"
	"""
	    
	m = StateMachine()
	m.add_state("INITIALIZATION", initialization)
	m.add_state("INITIAL DETECTION", initial_detection)
	m.add_state("APPROACH TARGET", approach_target)
	m.add_state("POINTING ROBOT ARM", pointing_robot_arm)
	m.add_state("APPROACH ROBOT ARM", approach_robot_arm)
	m.add_state("FINISH", None, end_state=1)

	m.set_start("Initialization")
	m.run()
