import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
from raw_exhibition.srv import *

sss = simple_script_server()

class SelectObjectFromTablet(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['objectSelected','quit'],
                             output_keys=['object_name'])
        rospy.Service('/trigger_new_order', Order, self.order_cb)
        rospy.Service('/trigger_new_task', TriggerTask, self.task_cb)
        self.ordered_object_ids = []

    def task_cb(self,req):
        print "trigger new task not implemented"
        return TriggerTaskResponse()

    def order_cb(self,req):
        print "ordered_object_ids"
        print req.ordered_object_ids
        
        self.ordered_object_ids = req.ordered_object_ids
        
        res = OrderResponse
        res.success = True
        return res
                             
    def execute(self, userdata):
        proto_objects = ['quit', 'milk', 'salt', 'tomato_sauce', 'tomato_soup', 'zwieback']
        print 'Please select an object on tablet gui'
        for i in range(len(proto_objects)):
            print str(i) + '  ' + proto_objects[i]

        #objectID = ""
        #while objectID not in [str(i) for i in range(len(proto_objects))]:
        #    #objectID = "1" #FIXME hardcoded to milk for testing
        #    objectID = raw_input('Please select an object: ')
        #print 'You selected #' + objectID
        
        while len(self.ordered_object_ids) == 0 and not rospy.is_shutdown():
            rospy.sleep(1)
        
        # set objectID and reset list of ordered objects
        objectID = self.ordered_object_ids[0]
        self.ordered_object_ids = []
        
        print 'You selected #' + str(objectID)
        
        if objectID==0:
            print 'Quit.'
            return 'quit'
        else:
            userdata.object_name = proto_objects[ objectID ]
            return 'objectSelected'
