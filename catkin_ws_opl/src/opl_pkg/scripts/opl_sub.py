#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from VrepCustomBot import *

opbot = VrepBot()
def callback_receive_location(msg):
    if opbot.path_goto > [4,6,1]:
        print('At DS')
    
    else:
        print('Wating' )

def main():
    rospy.init_node('location_op')

    pub = rospy.Publisher('cube_location', String  , queue_size=10)

    rospy.Subscriber('/opl_pub', String ,callback_receive_location)

    rospy.spin()
if __name__ == '__main__':
    main()
    # ropsy.init_node('location_op')

   # sub = rospy.Subscriber('/opl_pub', String, callback_receive_location)

   # rospy.spin()
