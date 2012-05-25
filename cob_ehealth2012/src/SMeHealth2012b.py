#!/usr/bin/python
import roslib
roslib.load_manifest('cob_ehealth2012')   # todo: additional command line parameter
import rospy
import smach
import smach_ros

from generic_navigation_states import *
from generic_perception_states import *

from simple_script_server import *  # import script

from PrepareRobot import *
from SelectObjectFromKeyboard import *
from Grasp import *

class SMeHealth2012b(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['ended'])
        with self:
            smach.StateMachine.add('PREPAREROBOT', PrepareRobot(),
                                   transitions={
                                    'succeeded':'SELECTOBJECTFROMKEYBOARD'
                                   })
            smach.StateMachine.add('SELECTOBJECTFROMKEYBOARD', SelectObjectFromKeyboard(),
                                   transitions={
                                    'objectSelected':'MOVE_TO_PREGRASP_POSITION',
                                    'quit':'ended'
                                  })
            smach.StateMachine.add('MOVE_TO_PREGRASP_POSITION', approach_pose([1,-0.5,3.14],mode="omni"),   
                        transitions={'succeeded':'MOVE_TO_GRASP_POSITION', 
                                                'failed':'ended'})
                                   
            smach.StateMachine.add('MOVE_TO_GRASP_POSITION', approach_pose([1.5,-0.5,3.14],mode="linear"),
                        transitions={'succeeded':'DETECT_OBJECT', 
                                                'failed':'ended'})
                                                
            smach.StateMachine.add('DETECT_OBJECT', detect_object(max_retries = 5),
                        transitions={'succeeded':'GRASP', 
                                                'no_object':'DETECT_OBJECT',    #no_object -> retry DETECT_OBJECT
                                                'no_more_retries':'ended', # object_not_picked
                                                'failed':'ended'})  # failed

            smach.StateMachine.add('GRASP', Grasp(),
                                   transitions={
                                    'grasped':'ended',
                                    'not_grasped':'ended', 
                                    'failed':'ended'
                                   })
                                          
                                   
