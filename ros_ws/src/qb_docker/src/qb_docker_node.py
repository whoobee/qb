#!/usr/bin/env python

# import generic stuff
from __future__ import print_function
import time
import traceback
import queue
# import ros stuff
import rospy
import std_msgs.msg
from std_msgs.msg import Float64, Int32, String
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from visualization_msgs.msg import Marker
import diagnostic_updater, diagnostic_msgs.msg
from geometry_msgs.msg import Pose
# import image msg types
from sensor_msgs.msg import Image # import rgb img msg ype
from sensor_msgs.msg import CameraInfo # import pointcloud msg type
# import cv2
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
# import realsense 2
import pyrealsense2 as rs2
# import numpy
import numpy as np


desired_aruco_dictionary = "DICT_ARUCO_ORIGINAL"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

#ROS logger class
class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #

#ROS parameter parse
def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val

#qB class
class QbDocker(object):
    #execution rate
    main_rate = 10
    #ros pub freq
    qb_docker_calc_hz = 2

    #ArUco coordonates
    center_x = -1
    center_y = -1
    top_right = -1
    top_left = -1
    bottom_right = -1
    bottom_left = -1

    intrinsics = None

    #init
    def __init__(self):
        #read params
        self.qb_docker_rgb = get_param('~qb_docker_rgb', "/qb_docker_rgb")
        self.qb_docker_rgb_info = get_param('~qb_docker_rgb_info', "/qb_docker_rgb_info")
        self.qb_docker_depth  = get_param('~qb_docker_depth' , "/qb_docker_depth")
        self.qb_docker_depth_info  = get_param('~qb_docker_depth_info' , "/qb_docker_depth_info")
        self.qb_docker_aruco_dict  = get_param('~qb_docker_aruco_dict' , "DICT_ARUCO_ORIGINAL")
        self.qb_docker_aruco_id  = get_param('~qb_docker_aruco_id' , 701)
        #ArUco dictionary selection
        self.aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[self.qb_docker_aruco_dict])
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        #setup shutdown hook
        rospy.on_shutdown(self.terminate)
        #setup pub and sub
        # Node is subscribing to the rgb stream topic
        rospy.Subscriber(self.qb_docker_rgb, Image, self.rgb_stream_cbk)
        # Node is subscribing to the rgb info frame topic
        rospy.Subscriber(self.qb_docker_rgb_info, CameraInfo, self.rgb_stream_info_cbk)
        # Node is subscribing to the depth frame topic
        rospy.Subscriber(self.qb_docker_depth, Image, self.depth_stream_cbk)
        # Node is subscribing to the depth info frame topic
        rospy.Subscriber(self.qb_docker_depth_info, CameraInfo, self.depth_stream_info_cbk)
        # Node publisher for marker
        self.pub = rospy.Publisher("/marker", Marker, queue_size = 100)
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # define Marker
        self.marker = Marker()
        self.marker.header.frame_id = "d435_link"
        self.marker.id = 1
        self.marker.type = 2
        self.marker.action = 2
        self.marker.pose = Pose()
        self.marker.color.r = 1.0
        self.marker.color.g = 0.5
        self.marker.color.b = 0.2
        self.marker.color.a = 0.3
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1
        self.marker.frame_locked = False
        self.marker.ns = 'marker_test_%d' % Marker.CUBE

    #main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.qb_docker_calc_hz)), self.fast_timer)
    
    #timer setup for main loop cyclic exec
    def fast_timer(self, timer_event):
        #calculate timestamp and publish
        time_now = rospy.Time.now()
        self.marker.header.stamp = time_now
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD

        #self.marker.pose.position.x = 0
        #self.marker.pose.position.y = 0
        #self.marker.pose.position.z = 0

        self.marker.lifetime = rospy.Duration.from_sec(10)
        rospy.loginfo("Publish marker")
        self.pub.publish(self.marker)

    
    #shutdown hook
    def terminate(self):
        # Close down the video stream when done
        cv2.destroyAllWindows()

    #rgb stream subscriber cbk
    def rgb_stream_cbk(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data)
            # Display image
            #cv2.imshow("camera", frame)

            # Detect ArUco markers in the video frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(
            frame, self.aruco_dictionary, parameters=self.aruco_parameters)
            # Check that at least one ArUco marker was detected
            if len(corners) > 0:
                # Flatten the ArUco IDs list
                ids = ids.flatten()
                # Loop over the detected ArUco corners
                for (marker_corner, marker_id) in zip(corners, ids):
                    if(marker_id == self.qb_docker_aruco_id):
                        # Extract the marker corners
                        corners = marker_corner.reshape((4, 2))
                        (top_left, top_right, bottom_right, bottom_left) = corners
                        # Convert the (x,y) coordinate pairs to integers
                        self.top_right = (int(top_right[0]), int(top_right[1]))
                        self.bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                        self.bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                        self.top_left = (int(top_left[0]), int(top_left[1]))
                        # Calculate and draw the center of the ArUco marker
                        self.center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                        self.center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                        # Dump some debug data
                        #rospy.loginfo("Found ArUco id (" + str(marker_id) + ") at coordonates [x=" + str(self.center_x) +", y=" + str(self.center_y) + "]")
            # synch frames
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
            return

    #rgb stream info subscriber cbk
    def rgb_stream_info_cbk(self, cameraInfo):
        try:
            # import pdb; pdb.set_trace()
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    #depth stream subscriber cbk
    def depth_stream_cbk(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix_center = (self.center_x, self.center_y)
            depth = (cv_image[int(pix_center[1]), int(pix_center[0])])
            #rospy.loginfo("Found ArUco id (" + str(self.qb_docker_aruco_id) + ") at coordonates [x=" + str(self.center_x) +", y=" + str(self.center_y) + "] at depth of (" + str(depth) + " mm)")
            if self.intrinsics:
                depth = cv_image[int(pix_center[1]), int(pix_center[0])]
                result_center = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix_center[0], pix_center[1]], depth)
                #rospy.loginfo("Result Center: " + str(result_center))

                depth_top_right = cv_image[self.top_right]
                result_top_right = rs2.rs2_deproject_pixel_to_point(self.intrinsics, self.top_right, depth_top_right)
                #rospy.loginfo("Result Top Right: " + str(result_top_right))

                depth_top_left = cv_image[self.top_left]
                result_top_left = rs2.rs2_deproject_pixel_to_point(self.intrinsics, self.top_left, depth_top_left)
                #rospy.loginfo("Result Top Left: " + str(result_top_left))

                depth_bot_right = cv_image[self.bottom_right]
                result_bot_right = rs2.rs2_deproject_pixel_to_point(self.intrinsics, self.bottom_right, depth_bot_right)
                #rospy.loginfo("Result Bot Right: " + str(result_bot_right))

                depth_bot_left = cv_image[self.bottom_left]
                result_bot_left = rs2.rs2_deproject_pixel_to_point(self.intrinsics, self.bottom_left, depth_bot_left)
                #rospy.loginfo("Result Bot Left: " + str(result_bot_left))

                ux, uy, uz = u = [result_top_right[0]-result_top_left[0], result_top_right[1]-result_top_left[1], result_top_right[2]-result_top_left[2]]
                vx, vy, vz = v = [result_bot_right[0]-result_top_left[0], result_bot_right[1]-result_top_left[1], result_bot_right[2]-result_top_left[2]]
                
                u_cross_v = [uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx]
                normal = np.array(u_cross_v)

                self.marker.pose.position.y = result_center[0]/1000
                self.marker.pose.position.z = result_center[1]/1000
                self.marker.pose.position.x = result_center[2]/1000

                rospy.loginfo("normal: " + str(result_center))

        except CvBridgeError as e:
            print(e)
            return

    #depth stream info subscriber cbk
    def depth_stream_info_cbk(self, cameraInfo):
        try:
            # import pdb; pdb.set_trace()
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

#main thread
def start_manager():
    #init ROS node
    rospy.init_node('qb_docker_mngr')

    #init and start the qB class
    qb_docker = QbDocker()
    qb_docker.main_loop()
    #shutdown listener
    while not rospy.is_shutdown():
        qb_docker.main_rate.sleep()
    
#main thread entry point
if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass

