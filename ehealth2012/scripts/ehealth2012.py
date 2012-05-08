#!/usr/bin/python

import roslib
roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# "exit" = pseudo object that triggers end of the script loop
proto_objects = ['exit', 'milk', 'salt', 'tomato_sauce', 'tomato_soup', 'zwieback']

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
        smach.State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
        print 'Executing task....'
        print 'Task executed.'
        return 'success'

def main():
    rospy.init_node('eHealth2012sm')
    sm = smach.StateMachine(outcomes=['ended'])

    with sm:
        smach.StateMachine.add('WAITFORTASK', WaitForTask(),
                               transitions={'taskSelected':'EXECUTETASK',
                                            'exit':'ended'})
        smach.StateMachine.add('EXECUTETASK', ExecuteTask(),
                               transitions={'success':'WAITFORTASK'})

    outcome = sm.execute()

if __name__=='__main__':
    main()

