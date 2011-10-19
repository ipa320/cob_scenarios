#!/usr/bin/python

PKG = 'cob_generic_states'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import unittest

from generic_state_machines import *

class TestStates(unittest.TestCase):
	def __init__(self, *args):
		super(TestStates, self).__init__(*args)
		rospy.init_node('test_states')

	def test_sm_open_door(self):
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])

		# open the container
		with SM:
			smach.StateMachine.add('TEST', sm_open_door(),
				transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})

		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
			self.fail(error_message)

	def test_sm_pick_object(self):
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
		SM.userdata.object_name = "milk"

		# open the container
		with SM:
			smach.StateMachine.add('TEST', sm_pick_object(),
				transitions={'object_picked_side':'overall_succeeded',
							'object_picked_top':'overall_succeeded',
							'object_not_picked':'overall_failed',
							'failed':'overall_failed'})

		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
			self.fail(error_message)

# main
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'perception', TestStates)
