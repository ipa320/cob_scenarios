#!/usr/bin/python
import roslib
roslib.load_manifest('cob_ehealth2012')   # todo: additional command line parameter
import rospy
import smach
import smach_ros
from SelectObjectFromKeyboard import *
#from generic_basic_states import *
#from generic_navigation_states import *
#from generic_perception_states import *

#from simple_script_server import *  # import script
#from State1 import *
from KeepMoving import *
#from PrepareRobot import *
#from SelectObjectFromKeyboard import *
#from Grasp import *
#from PutObjectOnTray import *

class SM(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(self, outcomes=['ended','aborted'],default_outcome='aborted',outcome_map={'ended':{'STATE2':'objectSelected'}})
        with self:
            smach.Concurrence.add('STATE1', KeepMoving())
            smach.Concurrence.add('STATE2',SelectObjectFromKeyboard())
 
rospy.init_node('eHealth2012')
sm = SM()
outcome = sm.execute()
#rospy.spin()


