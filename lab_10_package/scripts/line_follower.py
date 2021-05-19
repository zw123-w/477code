#!/usr/bin/env python

import rospy, message_filters, cv2
from sensor_msgs.msg import Image, colour_image, depth_image_topic
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class line_follower:
    def __init__(self, name):
        self.name = name
        self.bridge = CvBridge()
        self.image = colour_image
        self.image = None
        self.image = mask
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd_vel_pub.publish(twist)

        while not rospy.is_shutdown():
            if self.image != None:
                cv2.imshow("window", self.image)
                cv2.waitKey(30)


    def callback_camera(self, colour_msg, depth_msg):
        rospy.loginfo("[%s] callback_camera()", self.name)
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg,desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        rospy.Subscriber(depth_image_topic, Image, self.callback_depth)
        message_filters.Subscriber(depth_image_topic, Image)

        colour_image_topic = "/camera/rgb/image_raw"
        depth_image_topic = "/camera/depth/image_raw"
        colour_sub = message_filters.Subscriber(colour_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)
        self.camera_sync = message_filters.TimeSynchronizer([colour_sub, depth_sub], 10)
        self.camera_sync.registerCallback(self.callback_camera)
        self.camera_sync = message_filters.ApproximateTimeSynchronizer([colour_sub, depth_sub], queue_size=10, slop=0.1)

hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
lower_yellow = (25, 200, 100)
upper_yellow = (35, 255, 255)
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
mask = cv2.erode(mask, None, iterations=2)
mask = cv2.dilate(mask, None, iterations=2)


M = cv2.moments(mask)
if M['m00'] > 0:
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])

cv2.circle(colour_image, (cx, cy), 20, (0,0,255), -1)

err = cx - w/2
twist = Twist()
twist.linear.x = 0.2 # Constant forward movement
twist.angular.z = -float(err) / 1000


h, w, d = colour_image.shape
search_top = 3*h/4
search_bot = 3*h/4 + 20
mask[0:search_top, 0:w] = 0
mask[search_bot:h, 0:w] = 0


if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    rospy.loginfo("[line_follower] Starting Line Follower Module")
    lf = line_follower("line_follower")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[line_follower] Shutting Down Line Follower Module")