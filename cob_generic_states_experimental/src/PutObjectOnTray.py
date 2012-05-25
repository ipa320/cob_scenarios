import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

## Put object on tray side state
#
# This state puts a side grasped object on the tray
class PutObjectOnTray(smach.State):

	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		#TODO select position on tray depending on how many objects are on the tray already

		# move object to frontside
		handle_arm = sss.move("arm","grasp-to-tray",False)
		sss.move("head","front",False)
		sss.sleep(2)
		sss.move("tray","up")
		handle_arm.wait()

		# release object
		sss.move("sdh","cylopen")

		# move arm to backside again
		handle_arm = sss.move("arm","tray-to-folded",False)
		sss.sleep(3)
		sss.move("sdh","home")
		handle_arm.wait()
		return 'succeeded'
