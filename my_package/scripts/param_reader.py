#!/usr/bin/env python
 import rospy

 class param_reader:
    def __init__(self):
       
        self.beacons = rospy.get_param("~beacons")
        for x in self.beacons:
            id = x["id"]
            top_colour = x["top"]
            bottom_colour = x["bottom"]
            print("Beacon number {} is {} on the top and {} on the bottom".format(id, top_colour, bottom_colour))
            if top_colour == "blue" and bottom_colour == "red":
                print("The blue-red beacon is found!")

 if __name__ == '__main__':
    try:
        rospy.init_node('param_reader')
        rospy.loginfo("Starting Param Reader Node!")
        reader = param_reader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass