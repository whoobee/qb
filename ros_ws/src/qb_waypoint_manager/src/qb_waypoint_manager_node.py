#!/usr/bin/env python

# import generic stuff
from __future__ import print_function
import time
import sys
import os
import json
import traceback
import queue
import actionlib
# import ros stuff
import rospy
import std_msgs.msg
from std_msgs.msg import Float64, Int32, String
import diagnostic_updater, diagnostic_msgs.msg
#import ros Pose and Odom msgs
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
# import custome msgs and srvs
from qb_waypoint_manager.msg import qb_waypoint
from qb_waypoint_manager.srv import QbWaypointSave, QbWaypointSaveResponse
from qb_waypoint_manager.srv import QbWaypointSet, QbWaypointSetResponse
from qb_waypoint_manager.srv import QbWaypointGet, QbWaypointGetResponse
from qb_waypoint_manager.srv import QbWaypointRemove, QbWaypointRemoveResponse

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
class QbWaypointManager(object):
    #execution rate
    main_rate = 10
    #ros pub freq
    qb_waypoint_manager_calc_hz = 2
    #current pose
    current_pose = Pose()
    #waypoints_data
    waypoint_array = []
    #config state
    new_config = False
    #config file handler
    config_file_hndl = None

    #init
    def __init__(self):
        #read params
        self.qb_waypoint_config = get_param('~qb_waypoint_config', "")
        self.qb_odom_topic = get_param('~qb_odom_topic', "/odom")
        self.qb_waypoint_verbose = get_param('~verbose', False)
        self.qb_wait_for_odom = get_param('~wait_for_odom', 10) # value in hundreds of miliseconds
        #setup shutdown hook
        rospy.on_shutdown(self.terminate)
        #setup pub and sub
        self.qb_waypoint_save_srv = rospy.Service('qb_waypoint_save', QbWaypointSave, self.qb_waypoint_save_hndl)
        self.qb_waypoint_set_srv = rospy.Service('qb_waypoint_set', QbWaypointSet, self.qb_waypoint_set_hndl)
        self.qb_waypoint_get_srv = rospy.Service('qb_waypoint_get', QbWaypointGet, self.qb_waypoint_get_hndl)
        self.qb_waypoint_remove_srv = rospy.Service('qb_waypoint_remove', QbWaypointRemove, self.qb_waypoint_remove_hndl)
        # reset odom fram flag
        self.qb_odom_frame_received = False
        # check if configuration file is configured in launchfile
        if(self.qb_waypoint_config == ""):
            rospy.logerr("Waypoints configuration file path not set!")
            sys.exit(os.EX_CONFIG) 
        
        #create config file
        self.config_file = os.path.join(self.qb_waypoint_config, "waypoints.json")
        # check if configuration file exists
        if(os.path.isfile(self.config_file)):
            # file exists, open file
            self.config_file_hndl = open(self.config_file, 'r')
        else:
            # file doesn't exist, create a new file
            rospy.logwarn("No configuration found. Creating a new configuration file: " + self.config_file)
            self.config_file_hndl = open(self.config_file, 'w+')
            self.new_config = True
        # check if file is sane
        if(self.new_config != True):
            try:
                # load json data
                self.deserialize_waypoint_array(json.load(self.config_file_hndl))
                if(self.qb_waypoint_verbose):rospy.loginfo("Waypoints found: \n" + str(self.waypoint_array) + "\n")
            except Exception as e:
                # file is empty or corrupted, therefore overwrite it
                rospy.logwarn("The configuration file is empty or corrupted, therfore it will be overwritten.")
                if(self.qb_waypoint_verbose):rospy.logerr("Exception during json.load: " + str(e))
                self.new_config = True
        self.config_file_hndl.close()


    #waypoint save handler
    def qb_odom_cbk(self, data):
        self.current_pose = data.pose.pose
        self.qb_odom_frame_received = True

    #waypoint save handler
    def qb_waypoint_save_hndl(self, req):
        return_value = False
        # setup watchdog for odometry frame wait
        wdg_cnt = self.qb_wait_for_odom
        # subscribe to odometry topic
        self.qb_odom_sub = rospy.Subscriber(self.qb_odom_topic, Odometry, self.qb_odom_cbk)
        # wait for the odometry frame to be received
        while ((self.qb_odom_frame_received == False) and (wdg_cnt > 0)):
            time.sleep(0.1)
            wdg_cnt = wdg_cnt - 1
        if(wdg_cnt == 0):
            # Odometry frame wasn't received
            rospy.logerr("Odometry frame is not available.")
            rospy.loginfo("Please check if qb_odom_topic parameter is correctly configured, or if odometry data is available.")
            # return response
            return QbWaypointSaveResponse(return_value)
        else:
            # reset watchdog and frame received
            wdg_cnt = self.qb_wait_for_odom
            self.qb_odom_frame_received = True
        # create the waipoint message
        local_wp = qb_waypoint()
        local_wp.name = str(req.name)
        local_wp.pose = self.current_pose
        # check if waypoint already exists
        if(self.new_config != True):
            for wp in self.waypoint_array:
                if(wp.name == req.name):
                    # waypoint already extists, return error
                    rospy.logerr("Waypoint with the same name \"" + str(req.name) + "\" already exists in configuration.")
                    rospy.loginfo("Please remove the waypoint \"" + str(req.name) + "\" if you wish to save a new instance.")
                    # return response
                    return QbWaypointSaveResponse(return_value)
        # save data to file
        if(self.save_waypoit_to_config(local_wp) == True):
            if(self.qb_waypoint_verbose):rospy.loginfo("Waypoint \"" + str(req.name) + "\" saved.")
            return_value = True
        else:
            # error while saving
            if(self.qb_waypoint_verbose):rospy.logerr("Waypoint \"" + str(req.name) + "\" could not be saved to file.")
        # return response
        return QbWaypointSaveResponse(return_value)

    #waypoint set handler
    def qb_waypoint_set_hndl(self, req):
        return_value = False
        # check if waypoint already exists
        if(self.new_config != True):
            for wp in self.waypoint_array:
                if(wp.name == req.waypoint.name):
                    # waypoint already extists, return error
                    rospy.logerr("Waypoint with the same name \"" + str(req.waypoint.name) + "\" already exists in configuration.")
                    rospy.loginfo("Please remove the waypoint \"" + str(req.waypoint.name) + "\" if you wish to save a new instance.")
                    # return response
                    return QbWaypointSetResponse(return_value)
        # save data to file
        print(req.waypoint)
        if(self.save_waypoit_to_config(req.waypoint) == True):
            if(self.qb_waypoint_verbose):rospy.loginfo("Waypoint \"" + str(req.waypoint.name) + "\" saved.")
            return_value = True
        else:
            # error while saving
            if(self.qb_waypoint_verbose):rospy.logerr("Waypoint \"" + str(req.waypoint.name) + "\" could not be saved to file.")
        # return response
        return QbWaypointSetResponse(return_value)

    #waypoint get handler
    def qb_waypoint_get_hndl(self, req):
        waypoint_data = None
        # sarch for the waypoint with name <req> in the <waypoint_array>
        if(self.new_config != True):
            for wp in self.waypoint_array:
                if(wp.name == req.name):
                    waypoint_data = wp
        # check if waypoint was found
        if(waypoint_data != None):
            if(self.qb_waypoint_verbose):rospy.loginfo("Waypoint \"" + str(waypoint_data.name) + "\" found and returned.")
        else:
            waypoint_data = qb_waypoint()
            waypoint_data.name = "!ERROR!"
            if(self.qb_waypoint_verbose):rospy.logwarn("Waypoint \"" + str(req) + "\" was not found.")
        return QbWaypointGetResponse(waypoint_data)

    #waypoint remove handler
    def qb_waypoint_remove_hndl(self, req):
        waypoint_data = None
        return_value = False
        # sarch for the waypoint with name <req> in the <waypoint_array>
        if(self.new_config != True):
            for wp in self.waypoint_array:
                if(wp.name == req.name):
                    waypoint_data = wp
                    self.waypoint_array.remove(wp)
                    self.save_config()
        # check if waypoint was found
        if(waypoint_data != None):
            return_value = True
            if(self.qb_waypoint_verbose):rospy.loginfo("Waypoint \"" + str(waypoint_data.name) + "\" removed from config file")
        else:
            if(self.qb_waypoint_verbose):rospy.logwarn("Waypoint \"" + str(req) + "\" was not found.")
        return QbWaypointRemoveResponse(return_value)

    # save waypoint to json
    def save_waypoit_to_config(self, _wp):
        return_value = False
        # file exists, open file
        self.config_file_hndl = open(self.config_file, 'w')
        # in case config was empty or corrupted, replace data
        if(self.new_config != True):
            self.waypoint_array.append(_wp)
            if(self.qb_waypoint_verbose):rospy.loginfo("Waypoint appended: " + str(_wp))
        else:
            self.waypoint_array = [_wp]
            if(self.qb_waypoint_verbose):rospy.logwarn("Waypoint created: " + str(_wp))
            self.new_config = False
        # save data to file
        try:
            # save data to json
            json.dump(self.serialize_waypoint_array(), self.config_file_hndl, indent=4, sort_keys=True)
            return_value = True
        except Exception as e:
            # error while saving
            if(self.qb_waypoint_verbose):rospy.logerr("Exception during json.dump: " + str(e))
            pass
        # close the config file
        self.config_file_hndl.close()
        # return response
        return return_value

    # save waypoint to json
    def save_config(self):
        return_value = False
        # file exists, open file
        self.config_file_hndl = open(self.config_file, 'w')
        # save data to file
        try:
            # save data to json
            json.dump(self.serialize_waypoint_array(), self.config_file_hndl, indent=4, sort_keys=True)
            return_value = True
        except:
            # error while saving
            pass
        # close the config file
        self.config_file_hndl.close()
        # return response
        return return_value

    # serialize one waypoint data into jsonable data
    def serialize_wp(self, _wp):
        orientation = {"x":_wp.pose.orientation.x, "y":_wp.pose.orientation.y, "z":_wp.pose.orientation.z, "w":_wp.pose.orientation.w}
        position = {"x":_wp.pose.position.x, "y":_wp.pose.position.y, "z":_wp.pose.position.z}
        pose = {"position":position, "orientation":orientation}
        serialized_waypoint = {"name":_wp.name, "pose":pose}
        return serialized_waypoint

    # deserialize jsonable data into waypoint data
    def deserialize_wp(self, _wp):
        local_waypoint = qb_waypoint()
        local_waypoint.name = _wp["name"]
        local_waypoint.pose.position.x = _wp["pose"]["position"]["x"]
        local_waypoint.pose.position.y = _wp["pose"]["position"]["y"]
        local_waypoint.pose.position.z = _wp["pose"]["position"]["z"]
        local_waypoint.pose.orientation.x = _wp["pose"]["orientation"]["x"]
        local_waypoint.pose.orientation.y = _wp["pose"]["orientation"]["y"]
        local_waypoint.pose.orientation.z = _wp["pose"]["orientation"]["z"]
        local_waypoint.pose.orientation.w = _wp["pose"]["orientation"]["w"]
        return local_waypoint

    # serialize the entire waypoint arrays
    def serialize_waypoint_array(self):
        serialized_waypoint_array = []
        # loop through the entire array
        for wp in self.waypoint_array:
            serialized_waypoint_array.append(self.serialize_wp(wp))
        # return the new serialized array
        return serialized_waypoint_array

    # deserialize the entire waypoint arrays
    def deserialize_waypoint_array(self, _wps):
        # loop through the entire jsonable array, 
        # deserialize each element and append it to the waypoint_array
        for wp in _wps:
            self.waypoint_array.append(self.deserialize_wp(wp))

    #main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.qb_waypoint_manager_calc_hz)), self.fast_timer)
    
    #timer setup for main loop cyclic exec
    def fast_timer(self, timer_event):
        #calculate timestamp and publish
        time_now = rospy.Time.now()

    
    #shutdown hook
    def terminate(self):
        # Close the config file
        self.config_file_hndl.close()

#main thread
def start_manager():
    #init ROS node
    rospy.init_node('qb_waypoint_mngr')

    #init and start the qB class
    qb_waypoint_manager = QbWaypointManager()
    qb_waypoint_manager.main_loop()
    #shutdown listener
    while not rospy.is_shutdown():
        qb_waypoint_manager.main_rate.sleep()
    
#main thread entry point
if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass
