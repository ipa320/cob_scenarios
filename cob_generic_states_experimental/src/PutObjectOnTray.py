import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
import tf
from simple_script_server import *  # import script
sss = simple_script_server()

from kinematics_msgs.srv import *
from sensor_msgs.msg import *

## Put object on tray side state
#
# This state puts a side grasped object on the tray
class PutObjectOnTray(smach.State):

	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'],
			input_keys=['object'])

		self.transformer = rospy.ServiceProxy('/cob_pose_transform/get_pose_stamped_transformed', GetPoseStampedTransformed)

	def execute(self, userdata):
		#TODO select position on tray depending on how many objects are on the tray already

		req = GetPoseStampedTransformedRequest()
		req.tip_name = rospy.get_param("/cob_arm_kinematics/arm/tip_name")
		req.root_name = rospy.get_param("/cob_arm_kinematics/arm/root_name")
		req.target.header.stamp=rospy.Time.now()
		req.target.header.frame_id='base_link'
		pos = req.target.pose.position
		pos.x, pos.y, pos.z = [0.640, -0.110, 0.902] # tray_right_link in base_link
		pos.x -= 0.13
		pos.y += 0.065
		pos.z += userdata.object.bounding_box_lwh.z/2.0 + 0.08 # saftety margin 8cm
		req.orientation_override.w = 1 # align hand to baselink
		[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(-1.5708,0,0) # rpy
		req.orientation_override.x = new_x
		req.orientation_override.y = new_y
		req.orientation_override.z = new_z
		req.orientation_override.w = new_w
		req.origin.header.frame_id='sdh_grasp_link'
		req.origin.header.stamp=req.target.header.stamp

		
		res = self.transformer(req)
		if not res.success:
			print "Service call failed"
			self.retries = 0
			sss.set_light('red')
			return 'failed'
			
		seed_js = JointState()
		seed_js.name = rospy.get_param("/arm_controller/joint_names")
		seed_js.position = rospy.get_param("/script_server/arm/intermediatefront")[0]

		sss.set_light('blue')
		# calculate ik solutions for grasp configuration
		pos_js, error_code = sss.calculate_ik(res.result, seed_js)
		if(error_code.val != error_code.SUCCESS):
			if error_code.val != error_code.NO_IK_SOLUTION:
				sss.set_light('red')
			rospy.logerr("Ik Failed")
			return 'failed'

		sss.set_light('yellow')
		sss.move("head","front",False)
		handle_arm = sss.move("arm", ['intermediateback', 'intermediatefront',list(pos_js.position)],False)

		sss.sleep(2)
		sss.move("tray","up")
		handle_arm.wait()

		# release object
		sss.move("sdh","cylopen")

		## move arm to backside again
		handle_arm = sss.move("arm","tray-to-folded",False)
		sss.sleep(3)
		sss.move("sdh","home")
		handle_arm.wait()
		return 'succeeded'
