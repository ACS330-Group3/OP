#!/usr/bin/env python

import rospy
from op_msgs import Location

def handle_location(req):
   
    rospy.loginfo('Location of robot is x: ', req.x_coordinate,'y: ',req.y_coordinate, 'z: ',req.z_coordinate )
   

if __name__ == '__main__':
    rospy.init_node("communication_service_op")
    rospy.loginfo("Node created")

    service = rospy.Service("/robot_location",Location,handle_location )

    rospy.loginfo('Service has been started')

    rospy.spin()
