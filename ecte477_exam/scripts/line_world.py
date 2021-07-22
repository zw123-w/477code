import rospy, cv2, imutils
import numpy as np
import transformations as trans
from cv_bridge import CvBridge, CvBridgeError

# Messages used
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String, Int16, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from demonstration2.msg import Beacon

# Helper classes to tidy up looping over colours.
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

class my_node:
    def __init__(self, name):
        ##
        ## Deliverable 1 Code with unnecessary parts removed
        ##
        self.name = name
        # Subs and pubs
        self.publisher_map = rospy.Publisher('/ecte477/map', OccupancyGrid, queue_size=1)
        self.subscriber_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.subscriber_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.publisher_path = rospy.Publisher('/ecte477/path', Path, queue_size=1)
        # Object for storing path
        self.path = Path()
        self.path.header.frame_id = "odom"

        ##
        ## New Code
        ##

        # The camera matrix K = [fx, 0, cu; 0, fy, cv; 0, 0, 1]. 
        # Needs default value while waiting for topic.
        self.camera_K_inv = np.identity(3)
        self.camera_info_valid = False

        # Transformation from camera from to world frame, in SE(3)
        # Needs default value while waiting for topic.
        self.transform_cam_to_world = np.identity(4) # Robot starts at origin
        self.transform_info_valid = False

        # Subscribers to camera topics
        self.subscriber_colour = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback_colour)
        self.subscriber_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback_depth)
        self.subscriber_camera_info = rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.callback_camera_info)

        # Load beacons from parameter (~ means private). YAML file gets turning into list of dicts
        self.beacons = rospy.get_param("~beacons")
        self.beacons_found = [False]*len(self.beacons) # Separately keep track of which beacons are found.
        self.num_beacons_remain = len(self.beacons)

        # Test lines - can ignore
        print self.beacons              # beacons is a list of dictionaries
        print self.beacons_found        # beacons_found is a list of booleans
        print self.beacons[0]['top']    # You can access the dict values for each beacon by name

        # Inital variables for images
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.num_colour_images = 0
        self.num_depth_images = 0

        # Dictionary for storing colour data as HSV low, HSV high, and marker RGBA colour
        self.colour_ranges = {}
        self.colour_ranges[Colours.RED] = ColourRange((0, 248, 100), (5, 255, 255), ColorRGBA(1, 0, 0, 1))
        self.colour_ranges[Colours.GREEN] = ColourRange((56, 245, 100), (61, 255, 225), ColorRGBA(0, 1, 0, 1))
        self.colour_ranges[Colours.BLUE] = ColourRange((118, 245, 100), (122, 255, 255), ColorRGBA(0, 0, 1, 1))
        self.colour_ranges[Colours.YELLOW] = ColourRange((29, 240, 100), (32, 255, 226),ColorRGBA(1, 1, 0, 1))

        # Publisher for marker array and initial message for publishing
        self.publisher_markers = rospy.Publisher('ecte477/markers/', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray() # This is added to every time a beacon is found.

        # Publisher for custom beacon object
        self.publisher_beacon = rospy.Publisher('ecte477/beacons/', Beacon, queue_size=1)

        # Loop forever
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            self.publisher_markers.publish(self.marker_array)   # Publish this every loop
            r.sleep()

    ##
    ##  New code - Loop function
    ##

    # if the image and depth frames have been received, then process them for beacons
    def loop(self):
    
        if self.colour_frame != None and self.depth_frame != None:
            # rospy.loginfo("[%s] Processing frame", self.name)
            # frame = self.colour_frame.copy()    # Take a copy for displaying the result with the bounding box

            # Blur and convert to HSV for easy masking
            blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # We will process for all four colours and assume finding two means there's a beacon
            num_colours_found = 0
            colours_centres = []    # Used to determine if rects look like a beacon, and for localising beacon
            colours_list = []       # We want to know which colours were found

            # Iterate over our Colours enum
            for colour in Colours:

                # Mask based on HSV values in our colour ranges dict. Erode and dilate to fill in any gaps
                mask = cv2.inRange(hsv, self.colour_ranges[colour].lower, self.colour_ranges[colour].upper)
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                # Finds contours describing any blobs in the image (largest should be a beacon half)
                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                centre = None

                if len(cnts) > 0:
                    # If there are any contours, then the largest should correspond to a beacon half
                    c = max(cnts, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)
                    if w > 20 and h > 20:   # Small contours are either far away beacons or mismatches
                        colours_centres.append((x + w/2, y + h/2))
                        colours_list.append(colour)
                        num_colours_found += 1
                        
            # rospy.loginfo("[%s] Rectangles processed", self.name)

            # Two colours should mean we are looking at one beacon
            if num_colours_found == 2:
                # rospy.loginfo("[%s] Two colours found, might be a beacon", self.name)
                # Figure out which is top and which is bottom

                # Assume the lower y-valued colour is the top colour 
                colour_botom = None
                colour_top = None
                if colours_centres[0][1] > colours_centres[1][1]:
                    colour_botom = colours_list[0]
                    colour_top = colours_list[1]
                else:
                    colour_botom = colours_list[1]
                    colour_top = colours_list[0]

                # If rectangles aren't essentially on top of each other then this probably wasnt a beacon
                centre_x_diff = abs(colours_centres[0][0] - colours_centres[1][0])
                if centre_x_diff > 20:
                    return

                # rospy.loginfo("[%s] Rectangles aligned horizontally, must be a beacon", self.name)

                # Take the average of the two centres to find the 'beacon centre' pixel
                centre = ((colours_centres[0][0] + colours_centres[1][0])/2,
                        (colours_centres[0][1] + colours_centres[1][1])/2)

                # If the centre point is not inside the depth image, then we can't proceed.
                if centre[0] < 0 or centre[0] >= 1920 or centre[1] < 0 or centre[1] >= 1080:
                    return
                # rospy.loginfo("[%s] Centre inside image, can perform localisation", self.name)
                # Get depth from the depth image and convert to metres
                depth = self.depth_frame[centre[1], centre[0]]  # This assumes the depth image is in metres, would otherwise need scaling

                # Convert from pixel/depth to 3D point
                if self.camera_info_valid == False or self.transform_info_valid == False:
                    # The camera_info and odom topics haven't loaded the correct data yet
                    # camera_info might not be published after a Gazebo Ctrl-R world reset
                    return  

                # rospy.loginfo("[%s] Calculating 3D point from 2D point and depth", self.name)
                
                # P_3d_h = [X'; Y'; Z'; W'] = [R|t]*d*inv(K)*[x; y; 1]
                # P_3d = [X'/W'; Y'/W'; Z'/W']
                point_h = np.array([[centre[0]], [centre[1]], [1]])
                point_3d = depth * np.matmul(self.camera_K_inv, point_h)
                point_3d_h = np.array([[point_3d[2][0]], [-point_3d[0][0]], [-point_3d[1][0]], [1]]) # axes need swapping from image to robot [x, y, z] => [z, -x, -y]
                point_3d_world_h = np.matmul(self.transform_cam_to_world, point_3d_h)
                point_3d_world = np.array([[point_3d_world_h[0][0]/point_3d_world_h[3][0]], [point_3d_world_h[1][0]/point_3d_world_h[3][0]], [point_3d_world_h[2][0]/point_3d_world_h[3][0]]])

                # msg = "Beacon [" + colour_top.value + ", " + colour_botom.value + "] found at (" + repr(point_3d[0][0]) + ", " + repr(point_3d[1][0]) + ")" 
                # rospy.loginfo("[%s] %s", self.name, msg) 

                # Plot on map if this corresponds to a valid beacon that hasn't been found yet. Use our Beacons param dict to find valid beacons.
                for i in range(len(self.beacons)):
                    if self.beacons[i]['top'] == colour_top.value and self.beacons[i]['bottom'] == colour_botom.value and self.beacons_found[i] == False:
                        # Set beacon as found and decrement the number of beacons yet to be found
                        self.beacons_found[i] = True
                        self.num_beacons_remain -= 1
                        # msg = "Beacons remaining: {0}".format(self.num_beacons_remain)
                        # rospy.loginfo("[%s] %s", self.name, msg) 

                        # Add a marker to the marker array corresponding to this beacon.
                        marker = Marker()
                        marker.header.frame_id = 'map'      # This is the frame we transformed into. If we skipped the cam to world transform then it would be base_link
                        marker.header.stamp = rospy.Time.now()
                        marker.id = i
                        marker.pose = Pose(Point(point_3d_world[0][0], point_3d_world[1][0], 0), Quaternion(0.0, 1.0, 0.0, 1.0))
                        marker.scale = Vector3(0.3, 0.3, 0.3)
                        marker.color = self.colour_ranges[colour_top].rgba
                        self.marker_array.markers.append(marker)

                        # Publish a new Beacon object with same parameters as the marker and the colour names
                        beacon = Beacon()
                        beacon.header.frame_id = 'map' 
                        beacon.header.stamp = marker.header.stamp
                        beacon.top = colour_top.value
                        beacon.bottom = colour_botom.value
                        beacon.position = marker.pose.position
                        self.publisher_beacon.publish(beacon)

                        break   # Beacon detected has been associated with a so far unseen beacon, so we can break from loop 


    ##
    ## Demonstration 1 callbacks - only odom has changed slightly
    ##
		
    # Simply repeact the map data to the correct topic
    def callback_map(self, data):
        # rospy.loginfo("[%s] callback_map", self.name)
        self.publisher_map.publish(data)
        
    # Turn the odometry info into a path and repeat it to the correct topic
    # Also stores the transformation as an SE3 matrix for use with beacon locating
    def callback_odom(self, odometry):
        # rospy.loginfo("[%s] callback_odom", self.name)

        # Create path object
        pose_stamped = PoseStamped()
        pose_stamped.pose = odometry.pose.pose
        self.path.poses.append(pose_stamped)
        self.publisher_path.publish(self.path)

        # Store SE3 transformation for beacon locating
        self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)
        self.transform_info_valid = True

    ##
    ##  New callbacks
    ##

    def callback_colour(self, colour_image):
        # rospy.loginfo("[%s] callback_colour", self.name)
        np_array = np.fromstring(colour_image.data, np.uint8)
        self.colour_frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

    def callback_depth(self, depth_image):
        # rospy.loginfo("[%s] callback_depth", self.name)
        self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

    def callback_camera_info(self, camera_info):
        # rospy.logwarn("[%s] callback_camera_info", self.name)
        self.camera_K_inv = np.linalg.inv(np.array(camera_info.K).reshape([3, 3]))
        self.camera_info_valid = True
