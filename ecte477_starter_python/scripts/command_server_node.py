#!/usr/bin/env python
"""
    command_server_node.py

    A ROS node that sends commands based on the current state of the 
    USAR problem. 

    Listens to the start and stop commands on the cmd/ topic. These can 
    be sent with:
    rostopic pub -1 /cmd std_msgs/String -- 'start'
    rostopic pub -1 /cmd std_msgs/String -- 'stop'

    Subscribed: cmd/
    Publishes:

    Created: 2020/02/04
    Author: Brendan Halloran
"""

import rospy

from commands import Commands, RobotState
from std_msgs.msg import String

class command_server_node:
    def __init__(self):
        self.state = RobotState.WAITING_TO_START

        self.subscriber_command = rospy.Subscriber('cmd/', String, self.callback_command)
        self.publisher_state = rospy.Publisher('state/', String, queue_size=1)

        # Publish the current state at 10Hz to make sure other nodes get the correct info
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    # Run at 10Hz
    def loop(self):
        state_msg = String()
        state_msg.data = self.state.value
        self.publisher_state.publish(state_msg)



    def callback_command(self, data):
        command = Commands(data.data)

        if command is Commands.START:
            self.state = RobotState.EXPLORING
        elif command is Commands.STOP:
            self.state = RobotState.PAUSED
    
if __name__ == '__main__':
    print "Starting ROS Command Server module"
    rospy.init_node('command_server_node', anonymous=True)
    cs = command_server_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Command Server module"