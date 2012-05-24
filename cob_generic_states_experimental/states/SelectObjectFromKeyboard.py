import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

class SelectObjectFromKeyboard(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['objectSelected','quit'],
                             output_keys=['objectname'])
    def execute(self, userdata):
        proto_objects = ['quit', 'milk', 'salt', 'tomato_sauce', 'tomato_soup', 'zwieback']
        print 'Please select an object:'
        for i in range(len(proto_objects)):
            print str(i) + '  ' + proto_objects[i]
        objectID = ""
        while objectID not in [str(i) for i in range(len(proto_objects))]:
            objectID = raw_input('Please select an object: ')
        print 'You selected #' + objectID
        objectID = int(objectID)
        if objectID==0:
            print 'Quit.'
            return 'quit'
        else:
            userdata.objectname = proto_objects[ objectID ] 
            return 'objectSelected'