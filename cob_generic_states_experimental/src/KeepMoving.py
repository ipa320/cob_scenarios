import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros
import time
import random
from simple_script_server import *  # import script
sss = simple_script_server()

class KeepMoving(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
      outcomes=['succeeded'], input_keys=['concurrent_stop'], output_keys=['concurrent_stop'])

  def execute(self, userdata):
    userdata.concurrent_stop = False
    statements = ["Hello", "What can I bring for you?", "Fanta or Orange juice?", "My name is Care-O-bot", "Ich bin ein Saarbrucker"]
    while not userdata.concurrent_stop and not rospy.is_shutdown():
      choice = random.randint(1,4)
      if choice==1:
        choice2 = random.randint(0, len(statements)-1)
        sss.say([statements[choice2]])
      elif choice==2:       
        sss.set_light("red")      
      elif choice==3:
        sss.set_light("blue")
      else:
		sss.set_light("green")  
      #else:
      #  sss.move("torso","front")
        #sss.move("torso", "back")
        #rospy.sleep(0.5)	 	
      rospy.sleep(1.5)
    
    return 'succeeded'
