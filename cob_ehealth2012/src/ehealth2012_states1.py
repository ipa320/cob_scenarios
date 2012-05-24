#!/usr/bin/python

import roslib
roslib.load_manifest('ehealth2012')
import rospy
import smach
import smach_ros
from generic_navigation_states import *
from simple_script_server import *  # import script
sss = simple_script_server()

class prepare_robot(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'failed'])
    def execute(self, userdata):
        #sss.sleep(2)
        # say hello
        #sss.say(["Hello, my name is Care-o-bot."])
        
        # bring robot into the starting state
        #handle_tray = sss.move("tray","down",False)
        #handle_torso = sss.move("torso","home",False)
        #handle_arm = sss.move("arm","folded",False)
        #handle_sdh = sss.move("sdh","cylclosed",False)
        
        # wait for all movements to be finished
        #handle_tray.wait()
        #handle_torso.wait()
        #handle_arm.wait()
        #handle_sdh.wait()
        
        # announce ready
        #sss.say(["I am ready now."])
        return 'succeeded'

class WaitForTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['taskSelected', 'exit'])

    def execute(self, userdata):
        print 'Care-o-bot services:'
        for i in range(len(proto_objects)):
            print 'Pick ' + str(i) + '  ' + proto_objects[i]
        task = -100
        while task not in [str(i) for i in range(len(proto_objects))]:
            task = raw_input('Please select a task: ')
        print 'You selected task #' + task
        task = int(task)
        if task==0:
            return 'exit'
        else:
            return 'taskSelected'

class ExecuteTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    
    def execute(self, userdata):
        print 'Executing task....'
        ## sss.move_base_rel("base", [0, 0.1, 0.4])
        # approach_pose("pre_kitchen", mode="omni")
        p = approach_pose('deliver', mode="omni")
        p.execute()
        print 'Task executed.'
        return 'succeeded'
