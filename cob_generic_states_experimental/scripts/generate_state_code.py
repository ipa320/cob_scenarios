#!/usr/bin/python
import sys
import os
import subprocess

if sys.argv[1]=='-h':   # todo: properly implement command line options
    print 'Usage: generate_state_code <packagename> <statename>'
else:
    packagePath = subprocess.Popen('rospack find ' + sys.argv[1], shell=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0].strip()
    if len(packagePath)==0:
        print 'Could not find package. Aborting...'
        sys.exit(1)
    
    statePath = os.path.join(packagePath, 'states')
    template = os.path.join(subprocess.Popen('rospack find cob_generic_states_experimental', shell=True,
                                             stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0].strip(),
                            'scripts/template.py')
    commandCall = 'cog.py -d -D path=' + statePath + ' -D state=' + sys.argv[2] + ' ' + \
                    template + ' > ' + os.path.join(statePath, sys.argv[2] + '.py') 
    print commandCall
    os.system(commandCall)
