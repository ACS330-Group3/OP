#!/usr/bin/env python

import rospy
from opl_msgs import *

def handleLocation(req):
    

    

def main():
    ropsy.init_node('opl_server')
    rospy.spin()

if __name__ == '__main__':
    main()
   
   rospy.loginfo('Omniplatform server node created')

    service = rospy.Service('location_op_service')
