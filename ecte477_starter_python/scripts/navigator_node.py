#!/usr/bin/env python
"""
    navigator_node.py

    A ROS node that sends a PoseStamped goal to return home once the robot enters the RETURING state

    Subscribed:
    Publishes: 

    Created: 2020/02/07
    Author: Brendan Halloran
"""

import rospy
from commands import RobotState

class navigator_node:
    def __init__(self):
        self.navigating = False

    def callback_state(self, data):
        state = RobotState(data.data)
    
if __name__ == '__main__':
    print "Starting ROS Navigator module"
    rospy.init_node('navigator_node', anonymous=True)
    nv = navigator_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Navigator module"