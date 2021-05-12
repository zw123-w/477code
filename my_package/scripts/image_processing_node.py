import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError


class image_processing_node:
    def __init__(self,name):
        self.bridge= CvBridge()
        self.colour_frame= None
        self.num_colour_images=0
        self.subscriber_colour=rospy.Subscriber('/camera/rgb/image_raw/comprssed',
        CompressedImage, self.callback_colour)
        self.subscriber_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback_depth)
        r=rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()


    def callback_colour(self,colour_image):
        rospy.loginfo('[Image Processing] callback_colour')
        np_array=np.fromstring(colour_image.data,np.uint8)
        self.colour_frame=cv2.imdecode(np_array,cv2.IMREAD_COLOR)
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        #image processing

        # define the list of boundaries
        boundaries = [
	        ([0, 100, 255], [50, 255, 255]),
	        ([25, 100, 255], [45, 255, 255]),
	        ([115, 100, 255], [130, 255, 255]),
	        ([45, 100, 255], [65, 255, 255])
            ]

        # loop over the boundaries
        for (lower, upper) in boundaries:
	    # create NumPy arrays from the boundaries
	        lower = np.array(lower, dtype = "uint8")
	        upper = np.array(upper, dtype = "uint8")
	        mask = cv2.inRange(hsv, lower, upper)
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
        
        


    def callback_depth(self, depth_image):
        rospy.loginfo('[Image Processing] callback_depth')
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        


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

if __name__=='_main_':
    print "Starting ROS Image Processing Module"
    rospy,init_node('image_processing_node',anonymous=True,log_level=rospy.DEBUG)
    ipn=image_processing_node('image_processing_node')
    try:
        rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down ROS Image Processing Module"
