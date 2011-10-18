#!/usr/bin/python

import roslib; roslib.load_manifest('cob_experimentation_days')
import rospy

import smach
import smach_ros

# generic states
from generic_basic_states import *
from generic_navigation_states import *
from generic_manipulation_states import *
from generic_perception_states import *

# generic state machines
from generic_state_machines import *

# scenario specific states
from experimentation_days_states import *

# main
def main():
	rospy.init_node('experimentation_days')


	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['succeeded','failed'])
	SM.userdata.object_name = "milk"

	# open the container
	with SM:
		# add states to the container
		smach.StateMachine.add('PREPARE_ROBOT', prepare_robot(),
			transitions={'succeeded':'MOVE_TO_KITCHEN', 
						'failed':'failed'})
		
		smach.StateMachine.add('MOVE_TO_KITCHEN', approach_pose("kitchen"),
			transitions={'succeeded':'MOVE_TO_GRASP_POSITION', 
						'failed':'failed'})
						
		smach.StateMachine.add('MOVE_TO_GRASP_POSITION', approach_pose("grasp",mode="linear"),
			transitions={'succeeded':'SM_PICK_OBJECT', 
						'failed':'failed'})

		smach.StateMachine.add('SM_PICK_OBJECT', sm_pick_object(),
			transitions={'object_picked_side':'PUT_OBJECT_ON_TRAY_SIDE', 
						'object_picked_top':'PUT_OBJECT_ON_TRAY_TOP',  
						'object_not_picked':'failed', 
						'failed':'failed'})
						
		smach.StateMachine.add('MOVE_TO_POST_TABLE', approach_pose("post_table"),
			transitions={'succeeded':'PUT_OBJECT_ON_TRAY_SIDE', 
						'failed':'failed'},
			remapping={'base_pose':'base_pre_pose'})

		smach.StateMachine.add('PUT_OBJECT_ON_TRAY_SIDE', put_object_on_tray_side(),
				transitions={'succeeded':'MOVE_TO_DELIVER_POSITION', 
							'failed':'failed'})
				
		smach.StateMachine.add('PUT_OBJECT_ON_TRAY_TOP', put_object_on_tray_top(),
				transitions={'succeeded':'MOVE_TO_DELIVER_POSITION', 
							'failed':'failed'})

		smach.StateMachine.add('MOVE_TO_DELIVER_POSITION', approach_pose("order"),
			transitions={'succeeded':'DELIVER_OBJECT', 
						'failed':'failed'})

		smach.StateMachine.add('DELIVER_OBJECT', deliver_object(),
			transitions={'succeeded':'SAY_GOODBYE', 
						'retry':'DELIVER_OBJECT',
						'failed':'failed'})

		smach.StateMachine.add('SAY_GOODBYE', say_goodbye(),
			transitions={'succeeded':'succeeded', 
						'failed':'failed'})

	# Start SMACH viewer
	smach_viewer = smach_ros.IntrospectionServer('EXPERIMENTATION_DAYS', SM, 'EXPERIMENTATION_DAYS')
	smach_viewer.start()

	SM.execute()

	# stop SMACH viewer
	rospy.spin()
	# smach_thread.stop()
	smach_viewer.stop()

if __name__ == '__main__':
	main()
