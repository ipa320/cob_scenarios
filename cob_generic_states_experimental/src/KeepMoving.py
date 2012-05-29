import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class KeepMoving(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded'])
    def execute(self, userdata):
        sss.move_base_rel("base", [0.1,0,0.8]) #sleep(2) # 2 
        return 'succeeded'
    
