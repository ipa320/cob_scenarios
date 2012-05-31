import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class State2(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            output_keys=['object_name'])
        print "enetring State2"
    def execute(self, userdata):
        rospy.sleep(4)
        userdata.object_name = "bla"
        print "state2 succeeded"
        return 'succeeded'
