import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
##from simple_script_server import *  # import script
##sss = simple_script_server()

class SelectObjectFromKeyboard(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['objectSelected','quit'],
                             output_keys=['object_name'])
        rospy.Service('/trigger_new_task', TriggerTask, self.task_callback)
        self.objectID = ""
        
    def task_callback(self, req):
        self.objectID = req.task.data
        res = TriggerTaskResponse()
        return res        
        
    def execute(self, userdata):
        # proto_objects = ['quit', 'milk', 'HohesC', 'Fanta','Pringles','salt', 'tomato_sauce', 'tomato_soup', 'zwieback']
        proto_objects = ['pringles', 'tomatosauce', 'chocolate', 'juice', 'fanta']
        print 'Please select an object:'
        for i in range(len(proto_objects)):
            print str(i) + '  ' + proto_objects[i]
        self.objectID = ""
        while self.objectID not in [str(i) for i in range(len(proto_objects))]:
            #objectID = "1" #FIXME hardcoded to milk for testing
            rospy.sleep(1)
        print 'You selected #' + str(objectID)
        objectID = int(objectID)
        if objectID==10:
            print 'Quit.'
            userdata.object_name = "quit"
            return 'quit'
        else:
            print 'Selected: ' + str(proto_objects[ objectID ])
            userdata.object_name = proto_objects[ objectID ] 
            return 'objectSelected'
