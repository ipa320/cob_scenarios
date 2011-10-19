#!/usr/bin/python

import roslib; roslib.load_manifest('cob_experimentation_days')
import rospy

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

class prepare_robot(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		sss.sleep(2)
		# say hello
		sss.say(["Hello, my name is Care-o-bot."])
	
		# bring robot into the starting state
		handle_tray = sss.move("tray","down",False)
		handle_torso = sss.move("torso","home",False)
		handle_arm = sss.move("arm","folded",False)
		handle_sdh = sss.move("sdh","cylclosed",False)
		
		# wait for all movements to be finished
		handle_tray.wait()
		handle_torso.wait()
		handle_arm.wait()
		handle_sdh.wait()
		
		# announce ready
		sss.say(["I am ready now."])
		return 'succeeded'

class say_goodbye(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		handle_base = sss.move("base","home",False)
		sss.say(["Goodbye, perhaps we will meet again."])
		sss.move("torso","shake",False)
		handle_base.wait()
		return 'succeeded'
