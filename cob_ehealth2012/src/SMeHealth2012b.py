#!/usr/bin/python
import roslib
roslib.load_manifest('cob_ehealth2012')   # todo: additional command line parameter
import rospy
import smach
import smach_ros

from generic_basic_states import *
from generic_navigation_states import *
from generic_perception_states import *
from generic_manipulation_states import *

from simple_script_server import *  # import script

from PrepareRobot import *
from SelectObjectFromKeyboard import *
from Grasp import *
from PutObjectOnTray import *

class SMeHealth2012b(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['ended','failed'])
        with self:
            smach.StateMachine.add('PREPARE_ROBOT', PrepareRobot(),
                                   transitions={
                                    'succeeded':'SELECT_OBJECT_FROM_KEYBOARD'
                                   })
            smach.StateMachine.add('SELECT_OBJECT_FROM_KEYBOARD', SelectObjectFromKeyboard(),
                                   transitions={
                                    'objectSelected':'MOVE_TO_PREGRASP_POSITION',
                                    'quit':'ended'
                                  })
            smach.StateMachine.add('MOVE_TO_PREGRASP_POSITION', approach_pose([1,-0.5,0],mode="omni"),   
                        transitions={'succeeded':'MOVE_TO_GRASP_POSITION', 
                                                'failed':'failed'})
                                   
            smach.StateMachine.add('MOVE_TO_GRASP_POSITION', approach_pose([0,-0.5,0],mode="omni"),
                        transitions={'succeeded':'DETECT_OBJECT', 
                                                'failed':'failed'})
                                                
            smach.StateMachine.add('DETECT_OBJECT', detect_object(max_retries = 5),
                        transitions={'succeeded':'GRASP', 
                                                'no_object':'DETECT_OBJECT',    #no_object -> retry DETECT_OBJECT
                                                'no_more_retries':'SELECT_OBJECT_FROM_KEYBOARD', # object_not_picked
                                                'failed':'failed'})  # failed

            smach.StateMachine.add('GRASP', grasp_side_planned(),
                                   transitions={
                                    'grasped':'PUT_OBJECT_ON_TRAY',
                                    'not_grasped':'ended', 
                                    'failed':'failed'})

            smach.StateMachine.add('PUT_OBJECT_ON_TRAY', PutObjectOnTray(),
                                   transitions={
                                    'succeeded':'MOVE_TO_DELIVER_POSITION', 
                                    'failed':'failed'})

            smach.StateMachine.add('MOVE_TO_DELIVER_POSITION', approach_pose([2,-1,-1.57],mode="omni"),   
                        transitions={'succeeded':'DELIVER', 
                                                'failed':'failed'})

            smach.StateMachine.add('DELIVER', deliver_object(),
                                   transitions={
                                    'succeeded':'SELECT_OBJECT_FROM_KEYBOARD',
                                    'retry':'DELIVER',
                                    'failed':'failed'})
