#!/usr/bin/python
import roslib
roslib.load_manifest('cob_ehealth2012')   # todo: additional command line parameter
import rospy
import smach
import smach_ros

#from generic_basic_states import *
#from generic_navigation_states import *
#from generic_perception_states import *

#from simple_script_server import *  # import script
from State1 import *
#from KeepMoving import *
#from PrepareRobot import *
#from SelectObjectFromKeyboard import *
#from Grasp import *
#from PutObjectOnTray import *

class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE1', State1(),
                                   transitions={
                                    'succeeded':'ended'   # SELECT_OBJECT_FROM_KEYBOARD
                                   })
 
rospy.init_node('eHealth2012')
sm = SM()
outcome = sm.execute()
    

