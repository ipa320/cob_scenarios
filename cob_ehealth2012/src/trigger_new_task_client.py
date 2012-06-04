#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_ehealth2012')

import sys

import rospy
from raw_exhibition.srv import *

def order_service_client(order):
	rospy.wait_for_service('/trigger_new_order',5)
	try:
		order_service = rospy.ServiceProxy('/trigger_new_order', Order)
		req = OrderRequest()
		req.ordered_object_ids = [int(order)]
		print req
		res = order_service(req)
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
    while True:
        proto_objects = ['pringles', 'tomatosauce', 'chocolate', 'juice', 'fanta']
        print 'Please select an object:'
        for i in range(len(proto_objects)):
            print str(i) + '  ' + proto_objects[i]
        objectID = ""
        while str(objectID) not in [str(i) for i in range(len(proto_objects))]:
            objectID = raw_input('Please select an object: ')
        order_service_client(objectID)
