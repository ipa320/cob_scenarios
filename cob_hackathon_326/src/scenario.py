#!/usr/bin/python

import roslib
roslib.load_manifest('cob_hackathon_326')
import rospy
import smach
import smach_ros
from SM import *

def main():
    rospy.init_node('HACKATHON326')
    
    sm = SM()

    sis = smach_ros.IntrospectionServer('HACKATHON326', sm, 'HACKATHON326')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()
    
if __name__=='__main__':
    main()
