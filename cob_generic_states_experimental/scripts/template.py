#!/usr/bin/python
import roslib
roslib.load_manifest('ehealth2012')   # todo: additional command line parameter
import rospy
import smach
import smach_ros
#from generic_navigation_states import *
from simple_script_server import *  # import script
#[[[cog
import cog
import os
from xml.dom.minidom import parse, parseString
#]]]
#[[[end]]]
#[[[cog
doc = parse(os.path.join(path, state + '.xml'))
state_name, state_description = [doc.getElementsByTagName('state')[0].getAttributeNode(x).nodeValue
                                 for x in ['name', 'description']]
outcomes = [x.childNodes[0].data
            for x in doc.getElementsByTagName('outcome')]
input_keys = [x.childNodes[0].data
              for x in doc.getElementsByTagName('input_key')]
complexState = len(doc.getElementsByTagName('graph'))==1 

if complexState:  # complex state
    nodes = [(x.getAttributeNode('name').nodeValue, x.getAttributeNode('state').nodeValue)
             for x in doc.getElementsByTagName('node')]
    edges = [(x.getAttributeNode('from').nodeValue, x.getAttributeNode('via').nodeValue, x.getAttributeNode('to').nodeValue)
             for x in doc.getElementsByTagName('edge')] 

for node in nodes:
    cog.outl('from ' + node[1] + ' import *')

cog.outl('')
cog.outl('class ' + state + '(smach.StateMachine):')
cog.outl('    def __init__(self):')
cog.outl('        smach.StateMachine.__init__(self, outcomes=[' + \
         ','.join(["'" + x + "'" for x in outcomes]) + '])')
cog.outl('        with self:')
for node in nodes:
    cog.outl("            smach.StateMachine.add('" + node[0] + "', " + node[1] + "(),")
    cog.outl("                                   transitions={")
    transitions = ["                                    '" + edge[1] + "':'" + edge[2] + "'"
                   for edge in edges if edge[0]==node[0]]
    transText = ',\n'.join(transitions)
    cog.outl(transText)
    cog.outl("                                   })")
#]]]
#[[[end]]]

