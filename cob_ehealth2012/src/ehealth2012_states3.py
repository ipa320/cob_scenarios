#!/usr/bin/python

import roslib
roslib.load_manifest('ehealth2012')
import rospy
import smach
import smach_ros
from generic_navigation_states import *
from simple_script_server import *  # import script
sss = simple_script_server()

from ehealth2012_states2 import *

def main():
    rospy.init_node('eHealth2012_3sm')
    #sm = smach.StateMachine(outcomes=['ended'])
    #with sm:
    #    smach.StateMachine.add('FOO', HierSM1(),
    #                          transitions={'succeeded1':'ended'})
    
    sm = HierSM1()
    outcome = sm.execute()
    
if __name__=='__main__':
    main()
   
