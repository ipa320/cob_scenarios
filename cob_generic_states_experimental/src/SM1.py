#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')   # todo: additional command line parameter
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
        smach.Concurrence.__init__(self, outcomes=['objectSelected','quit'],default_outcome='quit',outcome_map={'objectSelected':{'SelectObjectFromKeyboard':'objectSelected'}})
        with self:
            smach.Concurrence.add('KeepMoving', KeepMoving())
            smach.Concurrence.add('SelectObjectFromKeyboard',SelectObjectFromKeyboard())
 
rospy.init_node('eHealth2012')
sm = SM()
outcome = sm.execute()
#rospy.spin()


