#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from opl_msgs.msg import LocationStatusOp
from VrepCustomBot.py import *


def callback_receive_location(msg):
   if  [x,y,gamma] == [6,8,2]
    rospy.loginfo('x:  {}, y: {}, gamma: {}'.format(x,y,z))
    print('At DS')

def main():
    rospy.init_node('location_op')

    pub = rospy.Publisher('cube_location', String  , queue_size=10)

    rospy.Subscriber('/opl_pub', String , monitor.callback_receive_location)
if __name__ == '__main__':
    main()
    # ropsy.init_node('location_op')

   # sub = rospy.Subscriber('/opl_pub', String, callback_receive_location)

   # rospy.spin()
