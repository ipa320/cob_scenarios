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
#   ROS stack name: cob_scenarios
# \note
#   ROS package name: cob_generic_states
#
# \author
#   Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2011
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

import copy

from simple_script_server import *
sss = simple_script_server()

import tf
from kinematics_msgs.srv import *

#this should be in manipulation_msgs
from cob_mmcontroller.msg import *


## Select grasp state
#
# This state select a grasping strategy. A high object will be grasped from the side, a low one from top.
class select_grasp(smach.State):

	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['top', 'side', 'failed'],
			input_keys=['object'])
		
		self.height_switch = 0.85 # Switch to select top or side grasp using the height of the object over the ground in [m].
		
		self.listener = tf.TransformListener()

	def execute(self, userdata):
		try:
			# transform object_pose into base_link
			object_pose_in = userdata.object.pose
			object_pose_in.header.stamp = self.listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
			object_pose_bl = self.listener.transformPose("/base_link", object_pose_in)
		except rospy.ROSException, e:
			print "Transformation not possible: %s"%e
			return 'failed'
		
		if object_pose_bl.pose.position.z >= self.height_switch: #TODO how to select grasps for objects within a cabinet or shelf?
			return 'side'
		else: 
			return 'top'


## Grasp side state
#
# This state will grasp an object with a side grasp
class grasp_side(smach.State):

	def __init__(self, max_retries = 1):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'retry', 'no_more_retries', 'failed'],
			input_keys=['object'])
		
		self.max_retries = max_retries
		self.retries = 0
		self.iks = rospy.ServiceProxy('/arm_controller/get_ik', GetPositionIK)
		self.listener = tf.TransformListener()
		self.stiffness = rospy.ServiceProxy('/arm_controller/set_joint_stiffness', SetJointStiffness)

	def callIKSolver(self, current_pose, goal_pose):
		req = GetPositionIKRequest()
		req.ik_request.ik_link_name = "sdh_grasp_link"
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped = goal_pose
		resp = self.iks(req)
		result = []
		for o in resp.solution.joint_state.position:
			result.append(o)
		return (result, resp.error_code)

	def execute(self, userdata):
		# check if maximum retries reached
		if self.retries > self.max_retries:
			self.retries = 0
			return 'no_more_retries'
	
		# make arm soft TODO: handle stiffness for schunk arm
		try:
			self.stiffness([300,300,300,100,100,100,100])
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.retries = 0
			return 'failed'
	
		# transform object_pose into base_link
		object_pose_in = userdata.object.pose
		object_pose_in.header.stamp = self.listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
		object_pose_bl = self.listener.transformPose("/base_link", object_pose_in)
	
		[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(-1.552, -0.042, 2.481) # rpy 
		object_pose_bl.pose.orientation.x = new_x
		object_pose_bl.pose.orientation.y = new_y
		object_pose_bl.pose.orientation.z = new_z
		object_pose_bl.pose.orientation.w = new_w

		# FIXME: this is calibration between camera and hand and should be removed from scripting level
		object_pose_bl.pose.position.x = object_pose_bl.pose.position.x - 0.06 #- 0.08
		#object_pose_bl.pose.position.y = object_pose_bl.pose.position.y - 0.05
		object_pose_bl.pose.position.z = object_pose_bl.pose.position.z - 0.1
		
		# calculate pre and post grasp positions
		pre_grasp_bl = PoseStamped()
		post_grasp_bl = PoseStamped()
		pre_grasp_bl = copy.deepcopy(object_pose_bl)
		post_grasp_bl = copy.deepcopy(object_pose_bl)

		pre_grasp_bl.pose.position.x = pre_grasp_bl.pose.position.x + 0.10 # x offset for pre grasp position
		pre_grasp_bl.pose.position.y = pre_grasp_bl.pose.position.y + 0.10 # y offset for pre grasp position
		post_grasp_bl.pose.position.x = post_grasp_bl.pose.position.x + 0.05 # x offset for post grasp position
		post_grasp_bl.pose.position.z = post_grasp_bl.pose.position.z + 0.15 # z offset for post grasp position
		
		# calculate ik solutions for pre grasp configuration
		arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp")
		(pre_grasp_conf, error_code) = self.callIKSolver(arm_pre_grasp[0], pre_grasp_bl)		
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik pre_grasp Failed")
			self.retries += 1
			return 'retry'
		
		# calculate ik solutions for grasp configuration
		(grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, object_pose_bl)
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik grasp Failed")
			self.retries += 1
			return 'retry'
		
		# calculate ik solutions for pre grasp configuration
		(post_grasp_conf, error_code) = self.callIKSolver(grasp_conf, post_grasp_bl)
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik post_grasp Failed")
			self.retries += 1
			return 'retry'	

		# execute grasp
		sss.say(["I am grasping the " + userdata.object.label + " now."],False)
		sss.move("torso","home")
		handle_arm = sss.move("arm", [pre_grasp_conf , grasp_conf],False)
		sss.move("sdh", "cylopen")
		handle_arm.wait()
		sss.move("sdh", "cylclosed")
	
		# move object to hold position
		sss.move("head","front",False)
		sss.move("arm", [post_grasp_conf, "hold"])
		
		self.retries = 0
		return 'succeeded'


## Grasp top state
#
# This state will grasp an object with a top grasp
class grasp_top(smach.State):

	def __init__(self, max_retries = 1):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'retry', 'no_more_retries', 'failed'],
			input_keys=['object'])
		
		self.max_retries = max_retries
		self.retries = 0
		self.iks = rospy.ServiceProxy('/arm_controller/get_ik', GetPositionIK)
		self.listener = tf.TransformListener()
		self.stiffness = rospy.ServiceProxy('/arm_controller/set_joint_stiffness', SetJointStiffness)

	def callIKSolver(self, current_pose, goal_pose):
		req = GetPositionIKRequest()
		req.ik_request.ik_link_name = "sdh_grasp_link"
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped = goal_pose
		resp = self.iks(req)
		result = []
		for o in resp.solution.joint_state.position:
			result.append(o)
		return (result, resp.error_code)

	def execute(self, userdata):
		# check if maximum retries reached
		if self.retries > self.max_retries:
			self.retries = 0
			return 'no_more_retries'
	
		# make arm soft TODO: handle stiffness for schunk arm
		try:
			self.stiffness([100,100,100,100,100,100,100])
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.retries = 0
			return 'failed'
	
		# transform object_pose into base_link
		object_pose_in = userdata.object.pose
		object_pose_in.header.stamp = self.listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
		object_pose_bl = self.listener.transformPose("/base_link", object_pose_in)
	
		# use a predefined (fixed) orientation for object_pose_bl
		[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(3.121, 0.077, -2.662) # rpy 
		object_pose_bl.pose.orientation.x = new_x
		object_pose_bl.pose.orientation.y = new_y
		object_pose_bl.pose.orientation.z = new_z
		object_pose_bl.pose.orientation.w = new_w

		# FIXME: this is calibration between camera and hand and should be removed from scripting level
		object_pose_bl.pose.position.x = object_pose_bl.pose.position.x -0.04 #- 0.08
		object_pose_bl.pose.position.y = object_pose_bl.pose.position.y# + 0.02
		object_pose_bl.pose.position.z = object_pose_bl.pose.position.z + 0.07

		# calculate pre and post grasp positions
		pre_grasp_bl = PoseStamped()
		post_grasp_bl = PoseStamped()
		pre_grasp_bl = copy.deepcopy(object_pose_bl)
		post_grasp_bl = copy.deepcopy(object_pose_bl)
	
		pre_grasp_bl.pose.position.z = pre_grasp_bl.pose.position.z + 0.12 # z offset for pre grasp position
		post_grasp_bl.pose.position.x = post_grasp_bl.pose.position.x + 0.05 # x offset for post grasp position
		post_grasp_bl.pose.position.z = post_grasp_bl.pose.position.z + 0.15 # z offset for post grasp position
		
		# calculate ik solutions for pre grasp configuration
		arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp_top")
		(pre_grasp_conf, error_code) = self.callIKSolver(arm_pre_grasp[0], pre_grasp_bl)		
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik pre_grasp Failed")
			self.retries += 1
			return 'retry'
		
		# calculate ik solutions for grasp configuration
		(grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, object_pose_bl)
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik grasp Failed")
			self.retries += 1
			return 'retry'
		
		# calculate ik solutions for pre grasp configuration
		(post_grasp_conf, error_code) = self.callIKSolver(grasp_conf, post_grasp_bl)
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik post_grasp Failed")
			self.retries += 1
			return 'retry'	

		# execute grasp
		sss.say(["I am grasping the " + userdata.object.label + " now."],False)
		sss.move("torso","home")
		handle_arm = sss.move("arm", [pre_grasp_conf , grasp_conf],False)
		sss.move("sdh", "spheropen")
		handle_arm.wait()
		sss.move("sdh", "spherclosed")
	
		# move object to frontside and put object on tray
		sss.move("head","front",False)
		sss.move("arm", [post_grasp_conf, "hold"])
		
		self.retries = 0
		return 'succeeded'


## Open door state
#
# This state will open a door
class open_door(smach.State):

	def __init__(self, max_retries = 1):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'retry', 'no_more_retries', 'failed'],
			input_keys=['object'])

		self.max_retries = max_retries
		self.retries = 0
		self.iks = rospy.ServiceProxy('/arm_controller/get_ik', GetPositionIK)
		self.listener = tf.TransformListener()
		self.mmstart = rospy.ServiceProxy('/mm/start', Trigger)
		self.mmstop = rospy.ServiceProxy('/mm/stop', Trigger)
		self.cartClient = actionlib.SimpleActionClient('/moveCirc', OpenFridgeAction)
		self.stiffness = rospy.ServiceProxy('/arm_controller/set_joint_stiffness', SetJointStiffness)

	def callIKSolver(self, current_pose, goal_pose):
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
		#TODO teach hinge and handle position relative to the door_pose (this means: detected ipa_logo)
		
		# check if maximum retries reached
		if self.retries > self.max_retries:
			self.retries = 0
			return 'no_more_retries'

		# make arm soft TODO: handle stiffness for schunk arm
		try:
			self.stiffness([100,100,100,100,100,100,100])
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.retries = 0
			return 'failed'

		try:
			# transform object_pose into base_link
			object_pose_in = userdata.object.pose
			object_pose_in.header.stamp = self.listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
			object_pose_bl = self.listener.transformPose("/base_link", object_pose_in)
		except rospy.ROSException, e:
			print "Transformation not possible: %s"%e
			return 'failed'
		
		arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp")

		# door handle pose
		door_handle_pose_bl = PoseStamped()
		door_handle_pose_bl.header.stamp = rospy.Time.now()
		door_handle_pose_bl.header.frame_id = "/base_link"
		door_handle_pose_bl.pose.position.x = object_pose_bl.pose.position.x+0.06 #+0.05
		door_handle_pose_bl.pose.position.y = object_pose_bl.pose.position.y+0.15
		door_handle_pose_bl.pose.position.z = object_pose_bl.pose.position.z-0.04 #-0.08
		door_handle_pose_bl.pose.orientation.x = -0.495
		door_handle_pose_bl.pose.orientation.y = -0.532
		door_handle_pose_bl.pose.orientation.z = 0.452
		door_handle_pose_bl.pose.orientation.w = 0.517

		# door hinge pose
		#door_hinge_pose_bl = PoseStamped()
		#door_hinge_pose_bl.header.stamp = rospy.Time.now()
		#door_hinge_pose_bl.header.frame_id = "/base_link"
		#door_hinge_pose_bl.pose.position.x = -0.7
		#door_hinge_pose_bl.pose.position.y = -0.4
		#door_hinge_pose_bl.pose.position.z = 0.9

		#quat = quaternion_from_euler(-3.14, 0, 0)
		#door_hinge_pose_bl.pose.orientation.x = quat[0]
		#door_hinge_pose_bl.pose.orientation.y = quat[1]
		#door_hinge_pose_bl.pose.orientation.z = quat[2]
		#door_hinge_pose_bl.pose.orientation.w = quat[3]

		# pre door handle position
		pre_door_handle_pose_bl = copy.deepcopy(door_handle_pose_bl)
		pre_door_handle_pose_bl.pose.position.x = pre_door_handle_pose_bl.pose.position.x + 0.05 # x offset for pre door position

		# calculate ik solutions for pre door configuration
		(pre_door_conf, error_code) = self.callIKSolver(arm_pre_grasp[0], pre_door_handle_pose_bl)		
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik pre_door_conf Failed")
			self.retries += 1
			return 'retry'

		# calculate ik solutions for door configuration
		(door_conf, error_code) = self.callIKSolver(arm_pre_grasp[0], door_handle_pose_bl)
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik door_conf Failed")
			self.retries += 1
			return 'retry'
		
		# move arm to handle
		sss.move("tray","up",False)
		handle_sdh = sss.move("sdh","cylopen",False)
		sss.move("torso","front")
		sss.move("arm", [pre_door_conf,door_conf])
		handle_sdh.wait()
		sss.move("sdh","cylclosed")

		# activate mm controller
		try:
			print "Starting MM controller"
			self.mmstart()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.retries = 0
			return 'failed'

		# wait for action server to become ready
		if not self.cartClient.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("moveCirc action server not ready within timeout, aborting...")
			self.retries = 0
			return 'failed'

		#syncmm movement to open fridge
		goal = OpenFridgeGoal()
		#goal.hinge = door_hinge_pose_bl
		self.cartClient.send_goal(goal)
		self.cartClient.wait_for_result(rospy.Duration.from_sec(20.0))

		# deactivate mm controller
		try:
			print "Stop MM controller"
			self.mmstop()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.retries = 0
			return 'failed'

		self.retries = 0
		
		sss.move("sdh","cyltotalopen")
		sss.move("arm","door_release")
		handle_arm = sss.move("arm","hold",False)
		sss.sleep(2)
		sss.move("sdh","cylclosed")
		handle_arm.wait()
		return 'succeeded'


## Put object on tray side state
#
# This state puts a side grasped object on the tray
class put_object_on_tray_side(smach.State):

	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		#TODO select position on tray depending on how many objects are on the tray already
		
		# move object to frontside
		handle_arm = sss.move("arm","grasp-to-tray",False)
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


## Put object on tray top state
#
# This state puts a top grasped object on the tray
class put_object_on_tray_top(smach.State):

	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		#TODO select position on tray depending on how many objects are on the tray already
		
		# move object to frontside
		handle_arm = sss.move("arm","grasp-to-tray_top",False)
		sss.sleep(2)
		sss.move("tray","up")
		handle_arm.wait()
		
		# release object
		sss.move("sdh","spheropen")
		
		# move arm to backside again
		handle_arm = sss.move("arm","tray_top-to-folded",False)
		sss.sleep(3)
		sss.move("sdh","home",False)
		handle_arm.wait()
		return 'succeeded'
