#!/usr/bin/python

import roslib
roslib.load_manifest('ehealth2012')
import rospy
import smach
import smach_ros

# auto-add these imports
from PrepareRobot import *
from SelectObjectFromKeyboard import *

from simple_script_server import *  # import script

sss = simple_script_server()

class SMeHealth2012a(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['ended'])
        
        with self:
            smach.StateMachine.add('PREPAREROBOT', PrepareRobot(),
                                   transitions={'succeeded':'SELECTOBJECTFROMKEYBOARD'}) 
            smach.StateMachine.add('SELECTOBJECTFROMKEYBOARD', SelectObjectFromKeyboard(),
                                   transitions={'objectSelected':'ended',
                                                'quit':'ended'})
 
 
