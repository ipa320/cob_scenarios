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

from KeepMovingUntilObjectSelected import *
#from KeepMoving import *
from PrepareRobot import *
from GetObjectInfo import *
#from SelectObjectFromKeyboard import *
from Grasp import *
from PutObjectOnTray import *

class SMeHealth2012b(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended','failed'])
        with self:
            
            smach.StateMachine.add('PREPARE_ROBOT',PrepareRobot(),transitions={'succeeded':'KEEPMOVINGUNTILSELECTED'})
            
            smach.StateMachine.add('KEEPMOVINGUNTILSELECTED',KeepMovingUntilObjectSelected(),
                transitions={'objectSelected':'GET_OBJECT_INFO',
                    'quit':'ended'})
#            smach.StateMachine.add('SELECTOBJECT', SelectObjectFromKeyboard(), transitions={'objectSelected':'DETECT_OBJECT','quit':'ended'})

            smach.StateMachine.add('GET_OBJECT_INFO',GetObjectInfo(),
                transitions={'objectInfoReceived':'MOVE_TO_PREGRASP_POSITION',
                    'objectInfoNotReceived':'KEEPMOVINGUNTILSELECTED'})    

            smach.StateMachine.add('MOVE_TO_PREGRASP_POSITION', approach_pose(mode="omni"),   
                transitions={'succeeded':'MOVE_TO_GRASP_POSITION', 
                    'failed':'failed'},
                remapping={'base_pose':'base_pose_pre'})
                                   
            smach.StateMachine.add('MOVE_TO_GRASP_POSITION', approach_pose(mode="omni"),
                        transitions={'succeeded':'DETECT_OBJECT', 
                                                'failed':'failed'})
                                               
            smach.StateMachine.add('DETECT_OBJECT', detect_object(max_retries = 5),
                        transitions={'succeeded':'GRASP',
                                                'no_object':'DETECT_OBJECT',    #no_object -> retry DETECT_OBJECT
                                                'no_more_retries':'KEEPMOVINGUNTILSELECTED', # object_not_picked
                                                'failed':'failed'})  # failed

            smach.StateMachine.add('GRASP', grasp_side_planned(),
                                   transitions={
                                    'grasped':'MOVE_TO_POSTGRASP_POSITION',
                                    'not_grasped':'DETECT_OBJECT', 
                                    'failed':'failed'})

            smach.StateMachine.add('MOVE_TO_POSTGRASP_POSITION', approach_pose(mode="omni"),   
                        transitions={'succeeded':'PUT_OBJECT_ON_TRAY', 
                                                'failed':'failed'},
                        remapping={'base_pose':'base_pose_pre'})

            smach.StateMachine.add('PUT_OBJECT_ON_TRAY', PutObjectOnTray(),
                                   transitions={
                                    'succeeded':'MOVE_TO_DELIVER_POSITION', 
                                    'failed':'failed'})

            smach.StateMachine.add('MOVE_TO_DELIVER_POSITION', approach_pose("deliver",mode="omni"),   
                        transitions={'succeeded':'DELIVER', 
                                                'failed':'failed'})

            smach.StateMachine.add('DELIVER', deliver_object(),
                                   transitions={
                                    'succeeded':'MOVE_TO_HOME_POSITION',
                                    'retry':'DELIVER',
                                    'failed':'failed'})

            smach.StateMachine.add('MOVE_TO_HOME_POSITION', approach_pose("home",mode="omni"),   
                        transitions={'succeeded':'KEEPMOVINGUNTILSELECTED', 
                                                'failed':'failed'})
                                    
            ## smach.StateMachine.userdata.object_name = "milk"                        
