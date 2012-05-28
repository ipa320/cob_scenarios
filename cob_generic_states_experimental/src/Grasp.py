import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class Grasp(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['grasped','not_grasped','failed'],
			input_keys=['object_name'])
	def execute(self, userdata):
		sss.say(["I am grasping " + userdata.object_name + " now."])
		sss.sleep(2)
		sss.say(["I grasped " + userdata.object_name + "."])
		return 'grasped'
