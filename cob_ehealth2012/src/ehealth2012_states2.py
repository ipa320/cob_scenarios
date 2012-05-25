#!/usr/bin/python
import roslib
roslib.load_manifest('ehealth2012')
import rospy
import smach
import smach_ros
from generic_navigation_states import *
from simple_script_server import *  # import script



class MySubstate1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        print 'Entering MySubstate1'
        return 'succeeded'
    
class MySubstate2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        return 'succeeded'

class HierSM1(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded1'])
        
        with self:
            smach.StateMachine.add('SUB1', MySubstate1(),
                                   transitions={'succeeded':'succeeded1'}) 
 
 
