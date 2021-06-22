#!/usr/bin/env python
"""
    beacon_node.py

    A ROS node that searching in colour images for any beacons and tries to 
    find its location relative to the robot.

    Subscribed: camera/colour/image_raw/compressed, camera/aligned_depth_to_color/image_raw/
    Publishes: 

    Created: 2020/02/04
    Author: Brendan Halloran
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from colours import Colours
from cv_bridge import CvBridge, CvBridgeError

# Constants
DEPTH_SCALE = 0.001     # Depth is given in integer values on 1mm

class beacon_node:
    def __init__(self):
        # Object used to convert between ROS images and OpenCV images
        self.bridge = CvBridge()

        # Subsccribers
        self.subscriber_colour_image = rospy.Subscriber('camera/color/image_raw/compressed', CompressedImage, self.callback_colour_image)
        self.subscriber_depth = rospy.Subscriber('camera/aligned_depth_to_color/image_raw/', Image, self.callback_depth)
        
        # Beacon data loaded from YAML file into ROS parameter
        self.beacons = rospy.get_param("~beacons")
        self.beacons_found = [False]*len(self.beacons)

        print self.beacons              # beacons is a list of dictionaries
        print self.beacons_found        # beacons_found is a list of booleans
        print self.beacons[1]['top']    # You can access the dict values for each beacon by name


    def callback_colour_image(self, data):
        print "callback_colour_image()" # For testing, can be removed
        # Decompress image and load into numpy array and OpenCV frame
        np_arr = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the result
        cv2.imshow('Colour Image', frame)
        cv2.waitKey(2)
        
    def callback_depth(self, data):
        print "callback_depth()" # For testing, can be removed
        try:
            # Convert image into OpenCV frame and numpy array
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)

            # Find the depth of the centre pixel in metres
            center_idx = np.array(depth_array.shape) / 2
            print "center depth: {0}".format( DEPTH_SCALE * depth_array[center_idx[0], center_idx[1]])

            # Display the result
            cv2.imshow('Depth Image', depth_image)
            cv2.waitKey(2)
        except CvBridgeError, e:
            print e

if __name__ == '__main__':
    print "Starting ROS Beacon Detector module"
    rospy.init_node('beacon_node', anonymous=True)
    bd = beacon_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Beacon Detector module"
