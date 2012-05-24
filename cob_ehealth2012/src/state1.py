#!/usr/bin/python
import roslib
roslib.load_manifest('ehealth2012')   # todo: additional command line parameter
import rospy
import smach
import smach_ros
from generic_navigation_states import *
from simple_script_server import *  # import script

/home/tys/git/care-o-bot/cob_scenarios/ehealth2012/states
state1

class state1(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=[succeeded,failure])
        with self:
            smach.StateMachine.add('BLA', BLUB(),
                                   transitions={
                                    'OUTBLA1':'BLA1',
                                    'OUTBLA2':'BLA2'
                                   })
            smach.StateMachine.add('BLA1', BLUB(),
                                   transitions={

                                   })

