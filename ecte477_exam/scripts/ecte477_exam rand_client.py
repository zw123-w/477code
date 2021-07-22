#!/usr/bin/env python

import rospy

from actionlib import SimpleActionClient
from ecte477_exam.msg import RandAction, RandFeedGoal

class rand_client:
    def __init__(self,name):
        self.name = rand_client
        
        self.action_client = SimpleActionClient('Rand_server', RandAction)
        self.action_client.wait_for_server()
        
     def execute_callback(self.goal):
         goal = RandGoal(order = 20)
         self.action_client.send_goal(goal)
         self.action_client.wait_for_result()
        
         result = self.action_client.wait_for_result()
         rospy.loginfo('[Rand Client] Result: {}'. format(', ', join([str(n) for n in result.sequence])))
    
if __name__ == '__main__':
  
    print "Starting ROS RandDist Client module"
    rospy.init_node('rand_client', anonymous=True, log_level=rospy.DEBUG)
    rand = rand_client('rand_client')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS RandDist Client moudle"
