#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')   # todo: additional command line parameter
import rospy
import smach
import smach_ros
#from concurrence2 import Concurrence2
#from SelectObjectFromKeyboard import *
#from generic_basic_states import *
#from generic_navigation_states import *
#from generic_perception_states import *
from State1 import State1
from State2 import State2
#from simple_script_server import *  # import script
#from State1 import *
#from KeepMoving import *
#from PrepareRobot import *
#from SelectObjectFromKeyboard import *
#from Grasp import *
#from PutObjectOnTray import *

# class SM(smach.Concurrence):
class SM(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(self, outcomes=['objectSelected','quit'],default_outcome='quit',outcome_map={'objectSelected':{'State2':'succeeded'}},child_termination_cb=lambda x: True,outcome_cb=lambda x: 'objectSelected')
        print "Concurrence2"
        with self:
            smach.Concurrence.add('State1',State1())
            smach.Concurrence.add('State2',State2())
 
rospy.init_node('eHealth2012')
sm = SM()
outcome = sm.execute()
#rospy.spin()


