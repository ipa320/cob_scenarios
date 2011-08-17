#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_apps
# \note
#   ROS package name: cob_generic_states
#
# \author
#   Author: Daniel Maeki
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: May 2011
#
# \brief
#   Implements generic states which can be used in multiple scenarios.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib
roslib.load_manifest('cob_generic_states')
import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

## Grasp side state
#
# This state will grasp an object with a side grasp
class grasp_side(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'])
		
	def execute(self, userdata):
		return 'succeeded'

## Grasp top state
#
# This state will grasp an object with a top grasp
class grasp_top(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'])
		
	def execute(self, userdata):
		return 'succeeded'

class grasp_drink(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['object_pose','object_detection_counter','drink_category','drink_name'], output_keys=['object_detection_counter'])
		self.iks = rospy.ServiceProxy('/arm_controller/get_ik', GetPositionIK)
		self.listener = tf.TransformListener()
		self.stiffness = rospy.ServiceProxy('/arm_controller/set_joint_stiffness', SetJointStiffness)

	def callIKSolver(self, current_pose, goal_pose):
		print "Calling IK Server 2"
		req = GetPositionIKRequest()
		req.ik_request.ik_link_name = "sdh_grasp_link"
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped = goal_pose
		#print req.ik_request
		resp = self.iks(req)
		result = []
		for o in resp.solution.joint_state.position:
			result.append(o)
		return (result, resp.error_code)	
		
	def execute(self, userdata):
		#sss.say(["I am grasping the " + userdata.drink_name + " now."],False)
		
		try:
			self.stiffness([300,300,300,300,300,300,300])
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		#print userdata.object_pose
		### call IK solver
		object_pose_in = userdata.object_pose
		#print object_pose_in
		object_pose_in.header.stamp = self.listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
		object_pose_bl  = self.listener.transformPose("/base_link", object_pose_in)
		
		# reject poses which are to high
		if (userdata.drink_category == "table"):
			limit = 0.9
		else:
			limit = 0.8
		if (object_pose_bl.pose.position.z <= limit):
			#sss.say(["unrealistic object detection result. I will try again."],False)
			return 'failed'
		
		look_at_table = rospy.get_param("/script_server/arm/look_at_table")

		if(userdata.drink_category == "table"):
			[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(-1.552, -0.042, 2.481) # rpy 
			object_pose_bl.pose.orientation.x = new_x
			object_pose_bl.pose.orientation.y = new_y
			object_pose_bl.pose.orientation.z = new_z
			object_pose_bl.pose.orientation.w = new_w

			object_pose_bl.pose.position.x = object_pose_bl.pose.position.x - 0.08
			#object_pose_bl.pose.position.y = object_pose_bl.pose.position.y - 0.05
			object_pose_bl.pose.position.z = object_pose_bl.pose.position.z + 0.03

			pre_grasp_bl = PoseStamped()
			post_grasp_bl = PoseStamped()
			pre_grasp_bl = copy.deepcopy(object_pose_bl)
			post_grasp_bl = copy.deepcopy(object_pose_bl)
		
			object_pose_bl.pose.position.z = object_pose_bl.pose.position.z - 0.02
		
			pre_grasp_bl.pose.position.x = pre_grasp_bl.pose.position.x + 0.05
			pre_grasp_bl.pose.position.y = pre_grasp_bl.pose.position.y + 0.05
		
			post_grasp_bl.pose.position.x = post_grasp_bl.pose.position.x + 0.1
			post_grasp_bl.pose.position.z = post_grasp_bl.pose.position.z + 0.1
		
			#print "Cartesian Pose Object: ", object_pose_bl
			#print "Cartesian Pose PreGrasp: ", pre_grasp_bl
			#print "Cartesian Pose PoseGrasp: ", post_grasp_bl
		
			arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp")


		else: #if snack
			[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(3.121, 0.077, -2.662) # rpy 
			object_pose_bl.pose.orientation.x = new_x
			object_pose_bl.pose.orientation.y = new_y
			object_pose_bl.pose.orientation.z = new_z
			object_pose_bl.pose.orientation.w = new_w

			object_pose_bl.pose.position.x = object_pose_bl.pose.position.x - 0.08
			object_pose_bl.pose.position.y = object_pose_bl.pose.position.y# + 0.02
			object_pose_bl.pose.position.z = object_pose_bl.pose.position.z + 0.07

			pre_grasp_bl = PoseStamped()
			post_grasp_bl = PoseStamped()
			pre_grasp_bl = copy.deepcopy(object_pose_bl)
			post_grasp_bl = copy.deepcopy(object_pose_bl)
		
			pre_grasp_bl.pose.position.z = pre_grasp_bl.pose.position.z + 0.12
		
		
			post_grasp_bl.pose.position.x = post_grasp_bl.pose.position.x + 0.05
			post_grasp_bl.pose.position.z = post_grasp_bl.pose.position.z + 0.15
		
			#print "Cartesian Pose Object: ", object_pose_bl
			#print "Cartesian Pose PreGrasp: ", pre_grasp_bl
			#print "Cartesian Pose PoseGrasp: ", post_grasp_bl
		
			arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp_top")
			#print arm_pre_grasp
				
			
		#print "look_at_table: ", look_at_table[0]
		(pre_grasp_conf, error_code) = self.callIKSolver(arm_pre_grasp[0], pre_grasp_bl)		
		if(error_code.val == error_code.SUCCESS):
			print pre_grasp_conf
		else:
			print "Ik pre_grasp Failed"
			#sss.say(["No IK solution found"])
			return 'failed'
			
		(object_pose_conf, error_code) = self.callIKSolver(pre_grasp_conf, object_pose_bl)
		if(error_code.val == error_code.SUCCESS):
			print object_pose_conf
		else:
			print "Ik object_pose Failed"
			#sss.say(["No IK solution found"])
			return 'failed'
			
		(post_grasp_conf, error_code) = self.callIKSolver(object_pose_conf, post_grasp_bl)
		if(error_code.val == error_code.SUCCESS):
			print post_grasp_conf
		else:
			print "Ik object_pose Failed"
			#sss.say(["No IK solution found"])
			return 'failed'	
			
		#sss.say( ["IK solution found, hahaha, I am now moving"],False)
		sss.say(["I am grasping the " + userdata.drink_name + " now."],False)
		print pre_grasp_conf
		print object_pose_conf
		
		#handle_arm = sss.move("arm", [arm_pre_grasp[0], pre_grasp_conf , object_pose_conf],False)
		sss.move("torso","home")
		handle_arm = sss.move("arm", [pre_grasp_conf , object_pose_conf],False)

		sss.sleep(1)
		sss.move("torso","shake",False)
		if(userdata.drink_category == "table"):
			sss.move("sdh", "cylopen")
			handle_arm.wait()
			sss.move("sdh", "cylclosed")
		else:
			sss.move("sdh", "spheropen")
			handle_arm.wait()
			sss.move("sdh", "spherclosed")
		
		# move arm to frontside and put object on tray
		sss.move("head","front",False)
		if userdata.drink_category == "table":
			handle_arm = sss.move("arm", [post_grasp_conf, "intermediateback","intermediatefront", "overtray"],False)
			sss.sleep(1)
			handle_base = sss.move("base","pre_table",False,mode="linear")
			handle_tray = sss.move("tray","up",False)
			handle_arm.wait()
			handle_tray.wait()
			sss.move("sdh","cylopen")
			handle_arm = sss.move("arm","tray-to-folded",False)
			
		else: #snack
			handle_arm = sss.move("arm", [post_grasp_conf, "intermediateback","intermediatefront","intermediatefront_top", "overtray_top"],False)
			sss.sleep(1)
			handle_base = sss.move("base","pre_shelf",False,mode="linear")
			handle_tray = sss.move("tray","up",False)
			handle_arm.wait()
			handle_tray.wait()
			sss.move("sdh","spheropen")
			handle_arm = sss.move("arm","tray_top-to-folded",False)

		sss.sleep(2)
		sss.move("sdh","home",False)
		handle_base.wait()
		sss.move("torso","shake",False)
		return 'succeeded'

