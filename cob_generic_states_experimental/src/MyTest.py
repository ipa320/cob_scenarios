#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'
        


# gets called when ANY child state terminates
def child_term_cb(outcome_map):

  # terminate all running states if FOO finished with outcome 'outcome3'
  if outcome_map['FOO'] == 'outcome3':
    return True

  # terminate all running states if BAR finished
  if outcome_map['BAR']:
    return True

  # in all other case, just keep running, don't terminate anything
  return False


# gets called when ALL child states are terminated
def out_cb(outcome_map):
   if outcome_map['FOO'] == 'succeeded':
      return 'outcome1'
   else:
      return 'outcome2'


# creating the concurrence state machine
sm = Concurrence(outcomes=['outcome1', 'outcome2'],
                 default_outcome='outcome1',
                 input_keys=['sm_input'],
                 output_keys=['sm_output'],
                 child_termination_cb = child_term_cb,
                 outcome_cb = out_cb)
                 
with sm:
   Concurrence.add('FOO', Foo(),
                   remapping={'foo_in':'input'})

   Concurrence.add('BAR', Bar(),
                   remapping={'bar_out':'bar_out'})


