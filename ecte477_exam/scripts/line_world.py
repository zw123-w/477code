#!/usr/bin/env python
"""
    navigator_node.py
    Created: 2021/05/18
    Author: Brendan Halloran
"""

import rospy, message_filters, cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion, Point, Vector3
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import transformations as trans

from std_msgs.msg import String, Int16, ColorRGBA


# Helper stuff
from enum import Enum
class Colours(Enum):
    RED = 'red'
    GREEN = 'green'
    BLUE = 'blue'
    YELLOW = 'yellow'
class ColourRange:
    def __init__(self, lower, upper, rgba):
        self.lower = lower
        self.upper = upper
        self.rgba = rgba
class ColourFoundStruct:
    def __init__(self, found, x):
        self.found = found
        self.x = x
        
        
class line_follower:
    def __init__(self, name):
        self.name = name
        self.bridge = CvBridge()
        cv2.namedWindow("window", 1)

        colour_image_topic = "/camera/rgb/image_raw"
        depth_image_topic = "/camera/depth/image_raw"

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        colour_sub = message_filters.Subscriber(colour_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)
        self.camera_sync = message_filters.ApproximateTimeSynchronizer([colour_sub, depth_sub], queue_size=10, slop=0.2)
        self.camera_sync.registerCallback(self.callback_camera)

        self.twist = Twist()
        self.image = None
        while not rospy.is_shutdown():
            if self.image != None:
                cv2.imshow("window", self.image)
                cv2.waitKey(30)

 def __init__(self, name):

        self.name = name                # Just used for loginfo labels
        self.bridge = CvBridge()        # Used for image message conversion
        cv2.namedWindow("window", 1)    # This is the graphical window that our test images will be sent to.

        # Dictionary for storing colour data as HSV low, HSV high, and marker RGBA colour
        self.colour_ranges = {}
        self.colour_ranges[Colours.RED] = ColourRange((0, 200, 100), (5, 255, 255), ColorRGBA(1, 0, 0, 1))
        self.colour_ranges[Colours.BLUE] = ColourRange((115, 200, 100), (125, 255, 255), ColorRGBA(0, 0, 1, 1))
        self.colour_ranges[Colours.YELLOW] = ColourRange((25, 200, 100), (35, 255, 255),ColorRGBA(1, 1, 0, 1))

        # Topic names set here to make Gazebo/Physical robot changes easier
        colour_image_topic = "/camera/rgb/image_raw"
        depth_image_topic = "/camera/depth/image_raw"

        # Logic for tracking if we have stopped for the stop sign
        self.stop_seen = False              # Set to true when the beacon is first seen
        self.stop_done = False              # Set to true when the stop action is done
        self.stop_time = rospy.Time()       # Updated to the time that the stop action starts

        # Movement actions sent here
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # This is the combined colour and depth image subscriber.
        colour_sub = message_filters.Subscriber(colour_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)
        self.camera_sync = message_filters.ApproximateTimeSynchronizer([colour_sub, depth_sub], queue_size=10, slop=0.2)
        self.camera_sync.registerCallback(self.callback_camera)

        self.twist = Twist()    # Movement command published each callback
        self.image = None       # Test image sent to window
        while not rospy.is_shutdown():
            if self.image != None:
                cv2.imshow("window", self.image)
                cv2.waitKey(30)

    def callback_camera(self, colour_msg, depth_msg):
        rospy.loginfo("[%s] callback_camera()", self.name)

        colour_image = self.bridge.imgmsg_to_cv2(colour_msg,desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        
        # Convert the colour and depth image messages into OpenCV types
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg,desired_encoding='bgr8')
        output_image = colour_image.copy()
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        # self.image = colour_image

        hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        lower_yellow = (25, 200, 100)
        upper_yellow = (35, 255, 255)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # self.image = mask

        h, w, d = colour_image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # self.image = mask

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(colour_image, (cx, cy), 20, (0,0,255), -1)
            self.image = colour_image
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(self.twist)
                                               
                
if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    rospy.loginfo("[line_follower] Starting ROS Line Follower Module")
    lf = line_follower("line_follower")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[line_follower] Shutting Down ROS Line Follower Module")
        cv2.destroyAllWindows()

