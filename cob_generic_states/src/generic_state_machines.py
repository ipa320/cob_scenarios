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

import roslib; roslib.load_manifest('cob_generic_states')
import rospy

import smach
import smach_ros

# generic states
from generic_manipulation_states import *
from generic_perception_states import *

class sm_open_door(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'door_not_opened', 'failed'])
		
		with self:
			smach.StateMachine.add('DETECT_DOOR', detect_object("door", max_retries = 10),
				transitions={'succeeded':'OPEN_DOOR', 
							'no_object':'DETECT_DOOR', 
							'no_more_retries':'door_not_opened', 
							'failed':'failed'})

			smach.StateMachine.add('OPEN_DOOR', open_door(),
				transitions={'succeeded':'succeeded', 
							'no_ik_solution':'DETECT_DOOR',
							'no_more_retries':'door_not_opened',
							'failed':'failed'})

class sm_pick_object(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, 
			outcomes=['object_picked_top', 'object_picked_side', 'object_not_picked', 'failed'],
			input_keys=['object_name'])
		
		with self:
			smach.StateMachine.add('DETECT_OBJECT', detect_object(max_retries = 10),
				transitions={'succeeded':'SELECT_GRASP', 
							'no_object':'DETECT_OBJECT', 			#no_object -> retry DETECT_OBJECT
							'no_more_retries':'object_not_picked', 
							'failed':'failed'})


			smach.StateMachine.add('SELECT_GRASP', select_grasp(),
				transitions={'top':'GRASP_TOP', 
							'side':'GRASP_SIDE', 
							'failed':'failed'})
			
			smach.StateMachine.add('GRASP_SIDE', grasp_side(max_retries = 10),
				transitions={'succeeded':'object_picked_side', 
							'no_ik_solution':'DETECT_OBJECT', 		#no_ik_solution -> retry DETECT_OBJECT
							'no_more_retries':'object_not_picked', 
							'failed':'failed'})
				
			smach.StateMachine.add('GRASP_TOP', grasp_top(max_retries = 10),
				transitions={'succeeded':'object_picked_top', 
							'no_ik_solution':'DETECT_OBJECT', 
							'no_more_retries':'object_not_picked', 
							'failed':'failed'})
