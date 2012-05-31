#!/usr/bin/python

import roslib
roslib.load_manifest('cob_ehealth2012')
import rospy
import smach
import smach_ros
# from SMeHealth2 import *
from simple_script_server import *  # import script
sss = simple_script_server()

from SMeHealth2012b import *

def main():
    rospy.init_node('AUTOMATICA2012')
    #sm = smach.StateMachine(outcomes=['ended'])
    #with sm:
    #    smach.StateMachine.add('FOO', HierSM1(),
    #                          transitions={'succeeded1':'ended'})
    
    SM = SMeHealth2012b()
    
    
   	# Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('AUTOMATICA2012', SM, 'AUTOMATICA2012')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()
    
if __name__=='__main__':
    main()
   
