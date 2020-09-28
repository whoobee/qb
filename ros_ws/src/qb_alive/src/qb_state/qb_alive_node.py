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
    qb_alive_calc_hz = 1
    #state var
    qb_alive_msg = "alive"

    #init
    def __init__(self):
        #setup shutdown hook
        rospy.on_shutdown(self.terminate)
        #setup pub and sub
        self.qb_alive_publisher  = rospy.Publisher("/qb_alive", String, queue_size=2)

    #main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.qb_alive_calc_hz)), self.fast_timer)
    
    #timer setup for main loop cyclic exec
    def fast_timer(self, timer_event):
        time_now = rospy.Time.now()
        self.pub_state(time_now)
    
    #shutdown hook
    def terminate(self):
        time_now = rospy.Time.now()
        self.qb_alive_msg = "dead"
        self.pub_state(time_now)

    #publisher func
    def pub_state(self, time_now):
        self.qb_alive_publisher.publish(self.qb_alive_msg)

#main thread
def start_manager():
    #init ROS node
    rospy.init_node('qb_alive_mngr')
    #init and start the qB class
    qb_alive = QbState()
    qb_alive.main_loop()
    #shutdown listener
    while not rospy.is_shutdown():
        qb_alive.main_rate.sleep()
    
#main thread entry point
if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass

