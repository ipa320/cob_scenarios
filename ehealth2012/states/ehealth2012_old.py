#!/usr/bin/python

import roslib
roslib.load_manifest('ehealth2012')
import rospy
import smach
import smach_ros
from generic_navigation_states import *
from simple_script_server import *  # import script
from ehealth2012_states1 import *

sss = simple_script_server()

# "exit" = pseudo object that triggers end of the script loop
proto_objects = ['exit', 'milk', 'salt', 'tomato_sauce', 'tomato_soup', 'zwieback']

def main():
    rospy.init_node('eHealth2012sm')
    sm = smach.StateMachine(outcomes=['ended'])

    with sm:
        smach.StateMachine.add('PREPARE_ROBOT', prepare_robot(),
                               transitions={'succeeded':'WAITFORTASK',
                                            'failed':'WAITFORTASK'})

        smach.StateMachine.add('WAITFORTASK', WaitForTask(),
                               transitions={'taskSelected':'HIERSM1', 
                                            'exit':'ended'})
        
        #smach.StateMachine.add('EXECUTETASK', approach_pose('deliver', mode="omni"),  
        #                       transitions={'succeeded':'WAITFORTASK', 'failed':'WAITFORTASK'})

        #smach.StateMachine.add('EXECUTETASK', ExecuteTask(),
        #                       transitions={'succeeded':'WAITFORTASK', 'failed':'WAITFORTASK'})

        smach.StateMachine.add('HIERSM1', HierSM1(),
                               transitions={'succeded':'WAITFORTASK',
                                            'exit':'ended'})

        # Start SMACH viewer
        #smach_viewer = smach_ros.IntrospectionServer('EXPERIMENTATION_DAYS', sm, 'EXPERIMENTATION_DAYS')
        #smach_viewer.start()

        outcome = sm.execute()

        #rospy.spin()  # ?
        #smach_viewer.stop()

if __name__=='__main__':
    main()

