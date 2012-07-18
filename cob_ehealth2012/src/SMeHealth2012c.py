#!/usr/bin/python
import roslib
roslib.load_manifest('cob_ehealth2012')   # todo: additional command line parameter
import rospy
import smach
import smach_ros
# from SelectObjectFromKeyboard import *
from SMeHealth2012b import *
from InitSDH import *
#from SelectObjectFromTablet import *
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

class SMeHealth2012c(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(
            self,
            outcomes=['quit'],
            default_outcome='quit')
        with self:
            smach.Concurrence.add('SMeHealth2012b', SMeHealth2012b())
            smach.Concurrence.add('InitSDH',InitSDH())
            # smach.Concurrence.add('SelectObjectFromKeyboard',SelectObjectFromTablet(), remapping={'object_name':'object_name'})
 
#rospy.init_node('eHealth2012')
#sm = SM()
#outcome = sm.execute()
#rospy.spin()


