#!/usr/bin/env python

from __future__ import print_function
import rospy
import std_msgs.msg
from std_msgs.msg import Float64, Int32, String
#from move_base.msg import *
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import diagnostic_updater, diagnostic_msgs.msg

import time
import traceback
import Queue

import json

#system state
STATE_MON_OFF_STATE      = 0
STATE_MON_OS_ON_STATE    = 1
STATE_MON_STARTING_STATE = 2
STATE_MON_READY_STATE    = 3
STATE_MON_RUNNING_STATE  = 4
STATE_MON_RECOVERY_STATE = 5
STATE_MON_ERROR_STATE    = 6

ALIVE_TIMEOUT = 5
ACTUATOR_TIMEOUT = 5
NAV_TIMEOUT = 5

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
class QbState(object):  
    #execution rate
    main_rate = 10
    #ros pub freq
    qb_state_calc_hz = 2
    #state var
    qb_state_msg = STATE_MON_OS_ON_STATE
    #wdt def
    alive_timeout_timer = ALIVE_TIMEOUT
    actuator_timeout_timer = ACTUATOR_TIMEOUT
    nav_timeout_timer = NAV_TIMEOUT
    #sub msg flags
    alive_engaged = False
    actuator_engaged = False
    nav_engaged = False

    #init
    def __init__(self):
        #read params
        self.qb_state_out_topic = get_param('~qb_state_out_topic', "/qb_state")
        self.qb_alive_in_topic  = get_param('~qb_alive_in_topic' , "/qb_alive")
        self.actuator_state_in_topic = get_param('~actuator_state_in_topic', "/odrive/status")
        self.nav_state_in_topic = get_param('~nav_state_in_topic', "/move_base/status")
        #setup shutdown hook
        rospy.on_shutdown(self.terminate)
        #setup pub and sub
        self.qb_state_publisher  = rospy.Publisher(self.qb_state_out_topic, Int32, queue_size=2)
        self.qb_alive_state_subscriber = rospy.Subscriber(self.qb_alive_in_topic, String, self.alive_state_in_callback)
        self.qb_nav_state_subscriber = rospy.Subscriber(self.nav_state_in_topic, GoalStatusArray, self.nav_state_in_callback)
        self.actuator_state_subscriber = rospy.Subscriber(self.actuator_state_in_topic, String, self.actuator_state_in_callback)

    #main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.qb_state_calc_hz)), self.fast_timer)
    
    #timer setup for main loop cyclic exec
    def fast_timer(self, timer_event):
        #reset wdt
        if(self.alive_engaged):
            self.alive_timeout_timer = self.alive_timeout_timer - 1
        if(self.actuator_engaged):
            self.actuator_timeout_timer = self.actuator_timeout_timer - 1
        if(self.nav_engaged):
            self.nav_timeout_timer = self.nav_timeout_timer - 1
        #check if wdt expired
        if(self.alive_timeout_timer <= 0):
            #alive wdt expired
            self.qb_state_msg = STATE_MON_OS_ON_STATE
        elif(self.actuator_timeout_timer <= 0):
            #actuator wdt expired
            self.qb_state_msg = STATE_MON_OS_ON_STATE
        elif(self.nav_timeout_timer <= 0):
            #nav wdt expired
            self.qb_state_msg = STATE_MON_READY_STATE
        else:
            pass
        #calculate timestamp and publish
        time_now = rospy.Time.now()
        self.pub_state(time_now)
    
    #shutdown hook
    def terminate(self):
        self.qb_state_msg = STATE_MON_ERROR_STATE

    #qb alive monitor callback function
    def alive_state_in_callback(self, alive_state):
        #at least one msg received
        self.alive_engaged = True
        #update wdt
        self.alive_timeout_timer = ALIVE_TIMEOUT
        #check if the qB stack is up and running
        #check is qB is in Starting State
        if(self.qb_state_msg <= STATE_MON_STARTING_STATE):
            if(alive_state.data == "alive"):
                self.qb_state_msg = STATE_MON_STARTING_STATE
            elif (alive_state.data == "dead"):
                self.qb_state_msg = STATE_MON_OS_ON_STATE

    #actuator state monitor callback function
    def actuator_state_in_callback(self, actuator_state):
        #at least one msg received
        self.actuator_engaged = True
        #update wdt
        self.actuator_timeout_timer = ACTUATOR_TIMEOUT
        #check if the ODrive stack is up and running
        #check is qB is in Starting State
        if(self.qb_state_msg >= STATE_MON_STARTING_STATE):
            #in ready check if nav stack has initialized
            if(actuator_state.data == "connected"):
                self.qb_state_msg = STATE_MON_READY_STATE
            elif (actuator_state.data == "disconnected"):
                self.qb_state_msg = STATE_MON_OS_ON_STATE

    #ROS navigation state callback function (from ROS Nav Stack)
    def nav_state_in_callback(self, nav_state):
        #at least one msg received
        self.nav_engaged = True
        #update wdt
        self.nav_timeout_timer = NAV_TIMEOUT
        #check is qB is in Ready State
        if(self.qb_state_msg >= STATE_MON_READY_STATE):
            #in ready check if nav stack has initialized
            if(nav_state.status_list):
                if(nav_state.status_list[0].status == 1):
                    #nav stack initialized
                    self.qb_state_msg = STATE_MON_RUNNING_STATE
                else:
                    #nav stack not initialized
                    self.qb_state_msg = STATE_MON_READY_STATE

    #publisher func
    def pub_state(self, time_now):
        self.qb_state_publisher.publish(self.qb_state_msg)

#main thread
def start_manager():
    #init ROS node
    rospy.init_node('qb_state_mngr')
    #init and start the qB class
    qb_state = QbState()
    qb_state.main_loop()
    #shutdown listener
    while not rospy.is_shutdown():
        qb_state.main_rate.sleep()
    
#main thread entry point
if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass

