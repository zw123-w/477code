#!/usr/bin/env python

import laser_assembler
import rospy
import time

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import SetBool
import numpy as np 
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters,cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class demo3:
    def __init__(self, name):
        self.name = name
        self.bridge= CvBridge()
        self.colour_frame= None
        self.num_colour_images=0

        colour_image_topic = "/camera/rgb/image_raw"
        depth_image_topic = "/camera/depth/image_raw"
        colour_sub = message_filters.Subscriber(colour_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)
        
        self.camera_sync = message_filters.ApproximateTimeSynchronizer([colour_sub, depth_sub], queue_size=10,slop=0.1)

        self.camera_sync.registerCallback(self.callback_camera)
        rospy.Subscriber(depth_image_topic, Image, self.callback_depth)
        message_filters.Subscriber(depth_image_topic, Image)

        # Subs and pubs
        self.publisher_move_base_simple_goal = rospy.Publisher('move_base_simple/goal/', PoseStamped, queue_size=1)
        self.publisher_map = rospy.Publisher('/ecte477/map', OccupancyGrid, queue_size=1)
        self.publisher_path = rospy.Publisher('/ecte477/path', Path, queue_size=1)
      
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.image = None
        while not rospy.is_shutdown():
            if self.image != None:
                cv2.imshow("window", self.image)
                cv2.waitKey(30)
                self.twist = Twist()
                self.loop()

    def callback_camera(self, colour_msg, depth_msg):

        rospy.loginfo("[%s] callback_camera()", self.name)
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg,desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        self.image = colour_image
        
        hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        h, w, d = colour_image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        self.image = mask
        self.seen_yellow = False
        self.cx_yellow = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            self.cx_yellow = int(M['m10']/M['m00'])
            self.cy_yellow = int(M['m01']/M['m00'])
            cv2.circle(colour_image, (self.cx_yellow, self.cy_yellow), 20, (0,0,255), -1)
            self.image = colour_image
            self.seen_yellow = True


        hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([15, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        h, w, d = colour_image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        self.image = mask
        self.seen_red = False
        self.cx_red = 0
        
        M = cv2.moments(mask)
        if M['m00'] > 0:
            self.cx_red = int(M['m10']/M['m00'])
            self.cy_red = int(M['m01']/M['m00'])
            cv2.circle(colour_image, (self.cx_red, self.cy_red), 20, (0,0,255), -1)
            self.image = colour_image
            self.seen_red = True

        if self.seen_yellow and self.seen_red:
            err = (self.cx_red + self.cx_yellow)/2 - w/2 
            twist = Twist()
            twist.linear.x = 0.2 # Constant forward movement
            twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(twist)
     
    def callback(self,msg):
       # The Turtlebot should halt until the moving obstacle leaves
       if msg.ranges[360] > 1:
           self.move.linear.x = 0.5 
           self.move.angular.z = 0.0 

       if msg.ranges[360] < 1:
           self.move.linear.x = 0.0
           self.move.angular.z = 0.0
       self.pub.publish(self.move)
    
    def callback_colour(self,colour_image):
        rospy.loginfo('[Image Processing] callback_colour')
        np_array=np.fromstring(colour_image.data,np.uint8)
        self.colour_frame=cv2.imdecode(np_array,cv2.IMREAD_COLOR)
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        lower_blue = (100, 100, 100)
        upper_blue = (110, 255, 255)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cv2.imshow('Masked Image', mask)
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        
        if len(contours) == 0:
            return

        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        colour_cv_frame = cv2.rectangle(colour_cv_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        
        cv2.waitKey(0)
        cv2.circle(colour_image, (x, y), 20, (0,0,255), -1)
        
        #stop robot and sleep for 2 seconds
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        rospy.sleep(2.)  #sleep for 2 seconds

    def callback_depth(self, depth_image):
        rospy.loginfo('[Image Processing] callback_depth')
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

        dh, dw = self.depth_frame.shape
        centre_depth = self.depth_frame[dh/2, dw/2]
        thre1 = 0.8
        
        if centre_depth > 0.0 and centre_depth < thre1:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            rospy.sleep(2.)

    def loop(self):
        if self.colour_frame !=None:
            cv2.imshow('Colour Image', self.colour_frame)
            cv2.imshow('Depth Image', self.depth_frame)
            resp=cv2.waitKey(80)
            if resp==ord('c'):
                rospy.loginfo('[Image Processing] Saving colour')
                cv2.imwrite('colour_{}.png'.format(self.num_colour_images),self.colour_frame)
                self.num_colour_images +=1
            if resp == ord('d'):
                rospy.loginfo('[Image Processing] Saving depth')
                cv2.imwrite('depth_{}.png'.format(self.num_depth_images), self.depth_frame)
                self.num_depth_images += 1
        

    def callback_camera_info(self, camera_info):
        self.K = np.array(camera_info.K).reshape([3, 3])

        if self.K == None or self.transform_cam_to_world == None or self.depth_array == None:
            return
        depth = self.depth_array[y, x]
        p_h = np.array([[x], [y], [1]]) # homogeneous vector
        p3d = depth * np.matmul(np.linalg.inv(self.K), p_h)
        p3d_h = np.array([[p3d[2][0]], [-p3d[0][0]], [-p3d[1][0]], [1]])
        p3d_w_h = np.matmul(self.transform_cam_to_world, p3d_h)
        p3d_w = np.array([[p3d_w_h[0][0]/p3d_w_h[3][0]], [p3d_w_h[1][0]/p3d_w_h[3][0]], [p3d_w_h[2][0]/p3d_w_h[3][0]]])


if __name__ == '__main__':
    rospy.init_node('demo3', anonymous = True)
    rospy.loginfo("[demo3] Starting Line Follower Module")
    lf = demo3("demo3")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[demo3] Shutting Down Line Follower Module")

