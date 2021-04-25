#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from VrepCustomBot import *

if __name__ == '__main__':

    opbot = VrepBot()

    if opbot.path_goto == True
        print('OP at DS')
    else:
        print('Waiting'
    rospy.init_node('opl_pub')

    pub = rospy.Publisher('/cube_location', String, queue_size=10) 

    rospy.spin()

    
    

