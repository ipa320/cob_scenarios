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

from simple_script_server import *
sss = simple_script_server()

from cob_generic_states.srv import *

## Initialize state
#
# This state will initialize all hardware drivers.
class initialize(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'])
		
	def execute(self, userdata):
		sss.set_light("yellow")

		# initialize components
		handle_head = sss.init("head")
		if handle_head.get_error_code() != 0:
			return 'failed'

		handle_torso = sss.init("torso")
		if handle_torso.get_error_code() != 0:
			return 'failed'
			
		handle_tray = sss.init("tray")
		if handle_tray.get_error_code() != 0:
			return 'failed'

		#handle_arm = sss.init("arm")
		#if handle_arm.get_error_code() != 0:
		#	return 'failed'

		handle_sdh = sss.init("sdh")
		#if handle_sdh.get_error_code() != 0:
		#	return 'failed'

		handle_base = sss.init("base")
		if handle_base.get_error_code() != 0:
			return 'failed'		
		
		# recover components
		handle_head = sss.recover("head")
		if handle_head.get_error_code() != 0:
			return 'failed'		
		
		handle_torso = sss.recover("torso")
		if handle_torso.get_error_code() != 0:
			return 'failed'
		
		handle_tray = sss.recover("tray")
		if handle_tray.get_error_code() != 0:
			return 'failed'

		handle_arm = sss.recover("arm")
		#if handle_arm.get_error_code() != 0:
		#	return 'failed'

		#handle_sdh = sss.recover("sdh")
		#if handle_sdh.get_error_code() != 0:
		#	return 'failed'

		handle_base = sss.recover("base")
		if handle_base.get_error_code() != 0:
			return 'failed'

		# set light
		sss.set_light("green")

		return 'succeeded'

## Wait for task state
#
# This state waits for a new task
class wait_for_task(smach.State):
	def __init__(self, tasks=[]):
		smach.State.__init__(self,
			outcomes=tasks)
		
		self.tasks = tasks
		rospy.Service('/trigger_new_task', TriggerTask, self.task_callback)

	def task_callback(self, req):
		res = TriggerTaskResponse()
		if self.tasks.count(req.task.data) == 0: # check if commended task is in list
			print "Task rejected. Given task was: ", req.task.data
			res.success.data = False
			res.error_message.data = "Task rejected"
		else:
			self.task = req.task.data
			print "New task triggered: ", self.task
			res.success.data = True
			res.error_message.data = "Task accepted"
			self.task_active = True
		return res

	def execute(self, userdata):
		self.task_active = False

		while True:
			print self.task_active
			if not self.task_active:
				sss.sleep(1) #TODO use loop_rate.sleep()
			else:
				self.task_active = True
				return self.task


## Get order state
#
# This state will get an order from the tablet_gui.
class get_order(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'no_order', 'failed'], 
			output_keys=['object_name'])

		self.srv_name_tablet_gui = "/tablet_gui"

	def execute(self, userdata):
		sss.say(["What do you want to order? Please select on my screen."],False)
		handle_tray = sss.move("tray","up",False)
		sss.move("torso","front",False)

		# check for tablet_gui service
		try:
			rospy.wait_for_service(self.srv_name_tablet_gui,10)
		except rospy.ROSException, e:
			print "Service not available: %s"%e
			return 'failed'
		
		# call tablet_gui service
		try:
			gui_service = rospy.ServiceProxy(self.srv_name_tablet_gui, OrderDrink)
			req = OrderDrinkRequest()
			res = gui_service(req) # TODO: use action to be able to cancel the order e.g. after timeout
			if len(res.drink.data) <= 0 or res.drink.data == "failed":
				rospy.logerr("Order failed")
				return 'no_order'
			userdata.object_name = res.drink.data
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
		
		sss.say(["You ordered " + userdata.object_name + "."],False)
		sss.move("torso","nod",False)
		sss.move("tray","down")
		return 'succeeded'


## Deliver object state
#
# This state will deliver an object which should be on the tray.
class deliver_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		#TODO implement deliver process
		return 'succeeded'
