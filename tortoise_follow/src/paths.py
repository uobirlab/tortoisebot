#!/usr/bin/env python
import roslib; roslib.load_manifest('tortoise_follow')
import rospy
import sys


path=""
for p in sys.path:
    path+='%s:'%p.strip()

print path[:-1]
    
