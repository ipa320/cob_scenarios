#!/usr/bin/python
import roslib
roslib.load_manifest('cob_hackathon_326')
import rospy
import smach
import smach_ros

from ChangeFloor import *
from WonderAround import *
from SearchPeople import *
from FetchObject import *

class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended','failed'])
        with self:
            
            self.userdata.base_pose = "home2"
            
#            smach.StateMachine.add('CHANGE_FLOOR',ChangeFloor(),
#                                   transitions={'changed':'WONDER_AROUND',
#                                                'not_changed':'ended',
#                                                'failed':'failed'})
            self.sm_con = smach.Concurrence(outcomes=['finished','failed'],
                                            default_outcome='finished',
                                            outcome_map={'finished':
                                                {'FETCH_OBJECT':'fetched',
                                                 'WONDER_AROUND':'finished'}},
                                            input_keys=['base_pose'])
                                         
            with self.sm_con:
                # Add states to the container
                smach.Concurrence.add('FETCH_OBJECT', FetchObject())
                smach.Concurrence.add('WONDER_AROUND', WonderAround())
                                         
            smach.StateMachine.add('CONCURRENT',self.sm_con,
                                   transitions={'finished':'ended',
                                                'failed':'failed'})
            
#            smach.StateMachine.add('WONDER_AROUND',WonderAround(),
#                                   transitions={'finished':'ended',
#                                                'failed':'failed'})

#            smach.StateMachine.add('SEARCH_PEOPLE',SearchPeople(),
#                                   transitions={'found':'ended',
#                                                'finished':'ended',
#                                                'failed':'failed'})
