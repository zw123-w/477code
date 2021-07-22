#!/usr/bin/env python
"""
    demo3.py
    Created: 2021/05/25
    Author: Brendan Halloran
"""

import rospy, message_filters, cv2
import numpy as np
import transformations as trans
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int16, ColorRGBA
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion, Point, Vector3
from cv_bridge import CvBridge, CvBridgeError

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

class demo3:
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

        # Convert the colour and depth image messages into OpenCV types
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg,desired_encoding='bgr8')
        output_image = colour_image.copy()
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        # self.image = colour_image

        # OBSTACLE
        # We do this first so we can se the movement command to zero and exit early
        # if the obstacle is in the way. Avoid extra processing that isnt needed.
        rospy.loginfo("[%s] Depth Image: %s", self.name, str(depth_image.shape))
        dh, dw = depth_image.shape
        centre_depth = depth_image[dh/2, dw/2]      # Just looking at the centre pixel in the depth image
        rospy.loginfo("[%s] Centre Depth: %s", self.name, str(centre_depth))
        if centre_depth != 1.0 and centre_depth != 0.0 and not np.isnan(centre_depth) and centre_depth < 0.8:   # 0.8 is our threshold. Other values are different representations of inf
                                                                                                                # 1.0 and NaN can happen in Gazebo and 0.0 happens on TB3 for inf far away.    
            rospy.loginfo("[%s] Obstacle, stopping", self.name)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return

        # LANE MARKINGS
        # Check for yellow and red lines, then take average as the target orientation
        colours_found = {}
        for colour in [Colours.YELLOW, Colours.RED]:

            colours_found[colour] = ColourFoundStruct(False,0)  # Assume not found

            # Mask off colour range
            hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.colour_ranges[colour].lower, self.colour_ranges[colour].upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Limit search region to bottom.
            h, w, d = colour_image.shape
            search_top = 2*h/4
            search_bot = 2*h/4 + 100
            mask[0:search_top, 0:w] = 0     # Set other parts of image to black
            mask[search_bot:h, 0:w] = 0

            # self.image = mask

            # Use moments to find weighted centroid of that colour blob
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                colours_found[colour] = ColourFoundStruct(True,cx)      # If found, we only need the x value for steering
                cv2.circle(output_image, (cx, cy), 10, (255,0,0), -1)


        # Logic for what to do depending on what lane markings were seen
        if not colours_found[Colours.YELLOW].found and not colours_found[Colours.RED].found:    # Stop if neither seen anymore
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0  
        elif not colours_found[Colours.YELLOW].found:       # Turn towards yellow line because we cant see it
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.4
        elif not colours_found[Colours.RED].found:          # Turn towards red line because we cant see it
            self.twist.linear.x = 0.2
            self.twist.angular.z = -0.4
        else:                                               # Keep centroid in middle of image if both lines are seen
            centroid = (colours_found[Colours.YELLOW].x + colours_found[Colours.RED].x)/2
            err = centroid - w/2
            self.twist.linear.x = 0.26
            self.twist.angular.z = -float(err) / 500
            
        # STOP SIGN
        # This is written as a for-loop despite being one one colour because I have
        # repurposed the code above. An ideal solution would combine both for-loops
        for colour in [Colours.BLUE]:

            # Mask off colour range
            hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.colour_ranges[colour].lower, self.colour_ranges[colour].upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            self.image = mask.copy()

            # Use moments to find weighted centroid of that colour blob
            # Could also be done using code from Demo 2 - that would be more
            # robust because it removes smaller detections and waits for a 
            # substantially sized detection.
            M = cv2.moments(mask)
            depth_cx = 0
            depth_cy = 0
            if M['m00'] > 0:
                rospy.loginfo("[%s] Stop sign seen", self.name)
                depth_cx = int(M['m10']/M['m00'])
                depth_cy = int(M['m01']/M['m00'])
                cv2.circle(output_image, (depth_cx, depth_cy), 10, (0,255,0), -1)
                if not self.stop_seen:
                    rospy.loginfo("[%s] First time seeing it", self.name)
                    self.stop_seen = True
                    self.stop_done = False

            # This is true once it is seen for the first time
            if self.stop_seen:
                depth = depth_image[depth_cy, depth_cx]
                rospy.loginfo("[%s] Depth: %s", self.name, str(depth))

                # This is our stopping distance
                if depth < 0.2:
                    rospy.loginfo("[%s] Stop sign close enough", self.name)
                    if not self.stop_done:      # Only stop once
                        rospy.loginfo("[%s] Stopping for first time", self.name)
                        self.stop_done = True
                        self.stop_time = rospy.Time.now()
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0

                # Once we have stopped for the first time, we want to only remain stopping if
                # less than 2 seconds has elapsed.  
                if self.stop_done:
                    time_diff = rospy.Time.now().secs - self.stop_time.secs
                    if time_diff < 2:
                        rospy.loginfo("[%s] Still stopping", self.name)
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                    else:
                        rospy.loginfo("[%s] Stopping done", self.name)


        self.cmd_vel_pub.publish(self.twist)
        # self.image = output_image
                
if __name__ == '__main__':
    rospy.init_node('demo3', anonymous=True)
    rospy.loginfo("[demo3] Starting ROS Line Follower Module")
    lf = demo3("demo3")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[demo3] Shutting Down ROS Line Follower Module")
        cv2.destroyAllWindows()