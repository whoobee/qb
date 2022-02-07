#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
# import control packages
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

# initialization substate
SS_INIT     = 0
# rotate substate
SS_ROTATE   = 1
# angle adjust substate
SS_ADJUST   = 2
# verification substate
SS_VERIFY   = 3
# depth measurment substate
SS_MEASURE  = 4
# success substate
SS_DONE     = 5
# failure substate
SS_FAILURE  = 6

class QbDockSearchState(EventState):
    '''
    Driving state for a ground robot. This state allows the robot to dock into a charger
    at a specified velocity/ speed.

    -- rotation_speed   float   Speed at which to drive the robot
    -- rotation_angle   float   How far to drive the robot before leaving this state
    -- command_velocity string  Command velocity topic name
    -- marker_id        int     Id of the ArUco marker. Possible values: 0 -- 999
    -- dictionary       string  Name of the ArUco dictionary to be used. 
                                Possible values (DICT_4X4_50, DICT_4X4_100, DICT_4X4_250, DICT_4X4_1000, DICT_5X5_50, 
                                                DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000, DICT_6X6_50, DICT_6X6_100, 
                                                DICT_6X6_250, DICT_6X6_1000, DICT_7X7_50, DICT_7X7_100, DICT_7X7_250, 
                                                DICT_7X7_1000, DICT_ARUCO_ORIGINAL)
    -- verbose          bool    Option for enabling debug information

    <= failed         If behavior is unable to ready on time
    <= done           Example for a failure outcome.

    '''

    def __init__(self, rotation_speed, rotation_angle, command_velocity, marker_id, dictionary, verbose):
        super(QbDockSearchState, self).__init__(outcomes=['failed', 'done'])
        # Substate initialization
        self.local_substate = SS_INIT
        # Init local params
        self._start_time = None
        self._speed = rotation_speed
        self._angle = rotation_angle
        self._command_velocity = command_velocity
        self._marker_id = marker_id
        self.qb_docker_rgb = "/d435/color/image_raw"
        self.qb_docker_depth = "/d435/depth/image_rect_raw"
        self.qb_docker_depth_info = "/d435/depth/camera_info"
        self.qb_docker_odom = "/odom"
        self._verbose = verbose
        self.counter = 0
        # ArUco dictionary selection
        self.aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[dictionary])
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        # ArUco coordonates
        self.marker_center_x = -1
        self.marker_center_y = -1
        self.marker_top_right = -1
        self.marker_top_left = -1
        self.marker_bottom_right = -1
        self.marker_bottom_left = -1
        # Depth camera intrinsics
        self.depth_cam_intrinsics = None
        # Init velocity command
        self.vel_msg = Twist()
        # Accepted error for alignment angle
        self.angular_z_accepted_err = 0.05
        # Proportional reg parameter
        self.kP = 2
        # Angle to adjust the robot position after marker is found
        self.adjust_angle = 0.0

    def execute(self, userdata):
        if(self.counter == 0):
            if(self._verbose):
                Logger.loginfo("DOCK SEARCH RUNNING!")
        # rotate the robot x deg
        if(self.local_substate == SS_ROTATE):
            if(self._verbose):Logger.loginfo("Substate = SS_ROTATE")
            # rotate self._angle degrees
            if(self.rotate_deg(self._angle) == True):
                # Rotation complete
                self.local_substate = SS_VERIFY
        # check if marker is visible
        elif(self.local_substate == SS_VERIFY):
            if(self._verbose):Logger.loginfo("Substate = SS_VERIFY")
            # We wait for the camera callback to search for ArUco marker,
            # handling is done in rgb_stream_cbk(self, data) function
            pass
        # calculate distance to the center of the marker
        elif(self.local_substate == SS_MEASURE):
            if(self._verbose):Logger.loginfo("Substate = SS_MEASURE")
            # We wait for the camera callback to measure the distance and angle to ArUco marker,
            # handling is done in depth_stream_cbk(self, data) function
            pass
        # fine adjust the robot to face the marker
        elif(self.local_substate == SS_ADJUST):
            if(self._verbose):Logger.loginfo("Substate = SS_ADJUST")
            # rotate self.adjust_angle degrees
            if(self.rotate_deg(self.adjust_angle) == True):
                # Rotation complete
                self.local_substate = SS_DONE
        # mark state as done
        elif(self.local_substate == SS_DONE):
            if(self._verbose):Logger.loginfo("Substate = SS_DONE")
            return 'done'
        # mark state as failed
        else: #(self.local_substate == SS_FAILURE)
            if(self._verbose):Logger.loginfo("Substate = SS_FAILURE")
            return 'failed'
        # increment call count
        self.counter = self.counter + 1

    def on_enter(self, userdata):
        if(self._verbose):Logger.loginfo("DOCK SEARCH STARTED!")
        self._start_time = rospy.Time.now()
        #setup pub and sub
        # Node is subscribing to the rgb stream topic
        self.sub_rgb = rospy.Subscriber(self.qb_docker_rgb, Image, self.rgb_stream_cbk)
        # Node is subscribing to the depth frame topic
        self.sub_depth = rospy.Subscriber(self.qb_docker_depth, Image, self.depth_stream_cbk)
        # Node is subscribing to the depth info frame topic
        self.sub_depth_info = rospy.Subscriber(self.qb_docker_depth_info, CameraInfo, self.depth_stream_info_cbk)
        # Node is subscribing to the odometry topic
        self.sub_odom = rospy.Subscriber(self.qb_docker_odom, Odometry, self.odom_info_cbk)
        # Node publisher for cmd_vel
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # Check if we don't already see the marker
        self.local_substate = SS_VERIFY
        
    def on_exit(self, userdata):
        if(self._verbose):Logger.loginfo("DOCK SEARCH ENDED!")
        # unregister subscribers
        self.sub_rgb.unregister()
        self.sub_depth.unregister()
        self.sub_depth_info.unregister()
        self.sub_odom.unregister()
        # Make sure that the robot rotation is really 0
        self.vel_msg.angular.z = 0.0
        self.pub.publish(self.vel_msg)
        # Clean OpenCV
        cv2.destroyAllWindows()
        
    def on_start(self):
        if(self._verbose):Logger.loginfo("DOCK SEARCH READY!")
        
    def on_stop(self):
        if(self._verbose):Logger.loginfo("DOCK SEARCH STOPPED!")


    # Rotate robot x deg
    def rotate_deg(self, _deg):
        result = False
        # Convert angle to radians
        _rad_angle = np.radians(_deg)
        # Use a simple proportional regulator
        self.vel_msg.angular.z = (self.kP * (_rad_angle - self.odom_yaw))
        # clamp rotation speed to the provided value
        if(self.vel_msg.angular.z > self._speed+self._speed): self.vel_msg.angular.z = self._speed+self._speed
        # publish velocity msg
        self.pub.publish(self.vel_msg)
        # debug info
        if(self._verbose):Logger.loginfo("Cmd-vel yaw: " + str(self.vel_msg.angular.z))
        if(self._verbose):Logger.loginfo("Target angle: " + str(_deg))
        # check if we reached the desired angle
        if((self.vel_msg.angular.z <= self.angular_z_accepted_err) and (self.vel_msg.angular.z >= -self.angular_z_accepted_err)):
            # goal reached, stop rotating and return success
            if(self._verbose):Logger.loginfo("Alignment done: " + str(self.vel_msg.angular.z))
            self.vel_msg.angular.z = 0.0
            self.pub.publish(self.vel_msg)
            result = True
        return result

    #odom stream subscriber cbk
    def odom_info_cbk(self, data):
        roll = pitch = yaw = 0.0
        # get robot orientation
        orientation = [ data.pose.pose.orientation.x, 
                        data.pose.pose.orientation.y, 
                        data.pose.pose.orientation.z, 
                        data.pose.pose.orientation.w ]
        # calculate euler angle and get yaw rotation
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
        self.odom_yaw = yaw


    #rgb stream subscriber cbk
    def rgb_stream_cbk(self, data):
        marker_found = False
        # in case we are in verify substate check for aruco markers
        if(self.local_substate == SS_VERIFY):
            try:
                # Convert ROS Image message to OpenCV image
                frame = self.bridge.imgmsg_to_cv2(data)
                # Detect ArUco markers in the video frame
                (corners, ids, rejected) = cv2.aruco.detectMarkers(
                frame, self.aruco_dictionary, parameters=self.aruco_parameters)
                # Check that at least one ArUco marker was detected
                if len(corners) > 0:
                    # Flatten the ArUco IDs list
                    ids = ids.flatten()
                    # Loop over the detected ArUco corners
                    for (marker_corner, marker_id) in zip(corners, ids):
                        # check if the marker is the required one
                        if(marker_id == self._marker_id):
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
                            # Consider marker as found
                            marker_found = True
                # synch frames
                cv2.waitKey(1)
            # Oh no, somthing went wrong with OpenCV
            except CvBridgeError as e:
                Logger.logerr("CvBridge exception: %s"%e)
                # Switch to failure substate
                self.local_substate = SS_FAILURE
            # In case we found the marker
            if(marker_found == True):
                # Switch to adjust substate
                self.local_substate = SS_ADJUST
                if(self._verbose):Logger.loginfo("Marker " + str(self._marker_id) + " was found.")
            else:
                # Search again
                self.local_substate = SS_ROTATE
                self._angle = 30 + np.degrees(self.odom_yaw)
                if(self._verbose):Logger.loginfo("Marker " + str(self._marker_id) + " was not found.")
        # Done
        return

    #depth stream subscriber cbk
    def depth_stream_cbk(self, data):
        angle_calculated = False
        # in case we are in measure substate calculate the distance to the marker
        if(self.local_substate == SS_MEASURE):
            try:
                # Get OpenCV image handler
                cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
                # Get center of marker pixel
                pix_marker_center = (self.center_x, self.center_y)
                # Get center of image pixel
                pix_camera_center = (data.width/2, self.center_y)
                # In case we determined the camera configuration
                if self.depth_cam_intrinsics:
                    # Calculate the distance to the center of the marker
                    depth_marker = cv_image[int(pix_marker_center[1]), int(pix_marker_center[0])]
                    result_marker_center = rs2.rs2_deproject_pixel_to_point(self.depth_cam_intrinsics, [pix_marker_center[0], pix_marker_center[1]], depth_marker)
                    # Calculate the distance to the center of the image
                    depth_image = cv_image[int(pix_camera_center[1]), int(pix_camera_center[0])]
                    result_image_center = rs2.rs2_deproject_pixel_to_point(self.depth_cam_intrinsics, [pix_camera_center[0], pix_camera_center[1]], depth_image)
                    # Define vector from the camera lens to marker center
                    ux, uy, uz = u = [result_marker_center[0], result_marker_center[1], result_marker_center[2]]
                    # Define vector from the camera lens to image center
                    vx, vy, vz = v = [result_image_center[0], result_image_center[1], result_image_center[2]]
                    # Normalize the vectors
                    u1 = u / np.linalg.norm(u)
                    v1 = v / np.linalg.norm(v)
                    # Calculate the angle between the vectors (dot product)
                    prod = np.dot(u1, v1)
                    self.adjust_angle = np.arccos(np.clip(np.dot(u1, v1), -1.0, 1.0))
                    angle_calculated = True
            # Oh no, somthing went wrong with OpenCV
            except CvBridgeError as e:
                self.adjust_angle = 0.0
                Logger.logerr("CvBridge exception: %s"%e)
                # Switch to failure substate
                self.local_substate = SS_FAILURE
            # In case we found the angle, switch to SS_ADJUST substate
            if(angle_calculated == True):
                self.local_substate = SS_ADJUST
            else:
                # in case angle not determined, try again
                self.adjust_angle = 0.0
        # Done
        return

    #depth stream info subscriber cbk
    def depth_stream_info_cbk(self, cameraInfo):
        try:
            # get depth camera info and calculate image error
            if self.depth_cam_intrinsics:
                return
            self.depth_cam_intrinsics = rs2.intrinsics()
            self.depth_cam_intrinsics.width = cameraInfo.width
            self.depth_cam_intrinsics.height = cameraInfo.height
            self.depth_cam_intrinsics.ppx = cameraInfo.K[2]
            self.depth_cam_intrinsics.ppy = cameraInfo.K[5]
            self.depth_cam_intrinsics.fx = cameraInfo.K[0]
            self.depth_cam_intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.depth_cam_intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.depth_cam_intrinsics.model = rs2.distortion.kannala_brandt4
            self.depth_cam_intrinsics.coeffs = [i for i in cameraInfo.D]
        # Oh no, somthing went wrong with OpenCV
        except CvBridgeError as e:
            Logger.logerr("CvBridge exception: %s"%e)
            # Switch to failure substate
            self.local_substate = SS_FAILURE
            return