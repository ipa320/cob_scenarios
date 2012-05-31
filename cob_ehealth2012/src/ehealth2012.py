#!/usr/bin/python

import roslib
roslib.load_manifest('cob_ehealth2012')
import rospy
import smach
import smach_ros
from SMeHealth2012b import *
from simple_script_server import *  # import script
sss = simple_script_server()

#from SMeHealth2012b import *

def main():
    rospy.init_node('ehealth2012')
    #sm = smach.StateMachine(outcomes=['ended'])
    #with sm:
    #    smach.StateMachine.add('FOO', HierSM1(),
    #                          transitions={'succeeded1':'ended'})
    
    sm = SMeHealth2012b()

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()
    
if __name__=='__main__':
    main()
   
