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
import diagnostic_updater, diagnostic_msgs.msg
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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

    vel_msg = Twist()

    aruco_found = False
    centered = False
    theta=0
    odom_yaw=0
    kP=0.4

    #init
    def __init__(self):
        #read params
        self.qb_docker_rgb = get_param('~qb_docker_rgb', "/qb_docker_rgb")
        self.qb_docker_rgb_info = get_param('~qb_docker_rgb_info', "/qb_docker_rgb_info")
        self.qb_docker_depth  = get_param('~qb_docker_depth' , "/qb_docker_depth")
        self.qb_docker_depth_info  = get_param('~qb_docker_depth_info' , "/qb_docker_depth_info")
        self.qb_docker_aruco_dict  = get_param('~qb_docker_aruco_dict' , "DICT_ARUCO_ORIGINAL")
        self.qb_docker_aruco_id  = get_param('~qb_docker_aruco_id' , 701)
        self.qb_docker_odom  = get_param('~qb_docker_odom' , "/odom")
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
        # Node is subscribing to the odometry topic
        rospy.Subscriber(self.qb_docker_odom, Odometry, self.odom_info_cbk)
        # Node publisher for cmd_vel
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

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
        #set linear speed to 0
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        #set angular speed on z only
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        if(not self.aruco_found):
            self.vel_msg.angular.z = abs(0.4)
            self.pub.publish(self.vel_msg)

    
    #shutdown hook
    def terminate(self):
        # Close down the video stream when done
        self.vel_msg.angular.z = abs(0)
        cv2.destroyAllWindows()

    #odom stream subscriber cbk
    def odom_info_cbk(self, data):
        roll = pitch = yaw = 0.0
        orientation = [ data.pose.pose.orientation.x, 
                        data.pose.pose.orientation.y, 
                        data.pose.pose.orientation.z, 
                        data.pose.pose.orientation.w ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
        self.odom_yaw = yaw
        if(self.centered == True):
            self.vel_msg.angular.z = abs(self.kP * (self.theta - self.odom_yaw))
            self.pub.publish(self.vel_msg)
            rospy.loginfo("CMD-vel yaw: " + str(self.vel_msg.angular.z))
            rospy.loginfo("Odom yaw: " + str(np.degrees(yaw)))

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
                        #if((self.center_x > (data.width/2) - 10) and (self.center_x < (data.width/2) + 10) ):
                        #rospy.loginfo("ArUco is in the middle")
                        self.aruco_found = True
                        self.vel_msg.angular.z = abs(0)
                        self.pub.publish(self.vel_msg)

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
            pix_marker_center = (self.center_x, self.center_y)
            pix_camera_center = (data.width/2, self.center_y)
            if self.intrinsics:
                if self.aruco_found == True:
                    depth_marker = cv_image[int(pix_marker_center[1]), int(pix_marker_center[0])]
                    result_marker_center = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix_marker_center[0], pix_marker_center[1]], depth_marker)
                    depth_image = cv_image[int(pix_camera_center[1]), int(pix_camera_center[0])]
                    result_image_center = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix_camera_center[0], pix_camera_center[1]], depth_image)

                    ux, uy, uz = u = [result_marker_center[0], result_marker_center[1], result_marker_center[2]]
                    vx, vy, vz = v = [result_image_center[0], result_image_center[1], result_image_center[2]]
                    #rospy.loginfo("u vect: " + str(u))
                    #rospy.loginfo("v vect: " + str(v))

                    u1 = u / np.linalg.norm(u)
                    v1 = v / np.linalg.norm(v)
                    #rospy.loginfo("u1 vect: " + str(u1))
                    #rospy.loginfo("v1 vect: " + str(v1))

                    prod = np.dot(u1, v1)
                    #rospy.loginfo("Calculated dot prod: " + str(prod))
                    _theta = np.arccos(np.clip(np.dot(u1, v1), -1.0, 1.0))

                    if(self.centered == False):
                        self.centered = True
                        self.theta = self.odom_yaw + _theta
                        rospy.loginfo("Calculated theta: " + str(np.degrees(self.theta)))
 
                #rospy.loginfo("Result Center: " + str(result_center))
                #depth_top_right = cv_image[self.top_right]
                #result_top_right = rs2.rs2_deproject_pixel_to_point(self.intrinsics, self.top_right, depth_top_right)
                #rospy.loginfo("Result Top Right: " + str(result_top_right))
                #depth_top_left = cv_image[self.top_left]
                #result_top_left = rs2.rs2_deproject_pixel_to_point(self.intrinsics, self.top_left, depth_top_left)
                #rospy.loginfo("Result Top Left: " + str(result_top_left))
                #depth_bot_right = cv_image[self.bottom_right]
                #result_bot_right = rs2.rs2_deproject_pixel_to_point(self.intrinsics, self.bottom_right, depth_bot_right)
                #rospy.loginfo("Result Bot Right: " + str(result_bot_right))
                #depth_bot_left = cv_image[self.bottom_left]
                #result_bot_left = rs2.rs2_deproject_pixel_to_point(self.intrinsics, self.bottom_left, depth_bot_left)
                #rospy.loginfo("Result Bot Left: " + str(result_bot_left))
                #ux, uy, uz = u = [result_top_right[0]-result_top_left[0], result_top_right[1]-result_top_left[1], result_top_right[2]-result_top_left[2]]
                #vx, vy, vz = v = [result_bot_right[0]-result_top_left[0], result_bot_right[1]-result_top_left[1], result_bot_right[2]-result_top_left[2]]
                #u_cross_v = [uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx]
                #normal = np.array(u_cross_v)
                #rospy.loginfo("normal: " + str(result_center))

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
