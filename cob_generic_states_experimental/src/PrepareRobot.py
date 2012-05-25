import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class PrepareRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded'])
    def execute(self, userdata):
        sss.sleep(2) # 2 
        sss.say(["Preparing."],False)
        
        # bring robot into the starting state
        handle_tray = sss.move("tray","down",False)
        handle_torso = sss.move("torso","home",False)
        handle_arm = sss.move("arm","folded",False)
        handle_sdh = sss.move("sdh","cylclosed",False)
        handle_head = sss.move("head","front",False)
        
        # wait for all movements to be finished
        handle_tray.wait()
        handle_torso.wait()
        handle_arm.wait()
        handle_sdh.wait()
        handle_head.wait()
        
        # announce ready
        sss.say(["Ready."])
        return 'succeeded'
    
