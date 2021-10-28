#!/usr/bin/env python

from __future__ import print_function
import rospy
import std_msgs.msg
from std_msgs.msg import Float64, Int32, String
from qb_ani.msg import qb_ani_req
from qb_ani.msg import qb_ani_stat
#from move_base.msg import *
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import diagnostic_updater, diagnostic_msgs.msg

import time
import traceback
import queue

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
class QbAni(object):  
    #execution rate
    main_rate = 10
    #ros pub freq
    qb_ani_calc_hz = 2
    #animation var
    qb_ani_msg = qb_ani_req()

    #init
    def __init__(self):
        #read params
        self.qb_ani_status = get_param('~qb_ani_status', "/qb_ani_status")
        self.qb_ani_req  = get_param('~qb_ani_req' , "/qb_ani_req")
        #setup shutdown hook
        rospy.on_shutdown(self.terminate)
        #setup pub and sub
        self.qb_ani_publisher  = rospy.Publisher(self.qb_ani_req, qb_ani_req, queue_size=2)
        self.qb_ani_status_subscriber = rospy.Subscriber(self.qb_ani_status, qb_ani_stat, self.qb_ani_status_callback)

    #main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.qb_ani_calc_hz)), self.fast_timer)
    
    #timer setup for main loop cyclic exec
    def fast_timer(self, timer_event):
        #calculate timestamp and publish
        time_now = rospy.Time.now()
        self.pub_animation_request(time_now)
    
    #shutdown hook
    def terminate(self):
        pass

    #qb alive monitor callback function
    def qb_ani_status_callback(self, ani_state):
        if(ani_state.state == qb_ani_stat.ANI_STATUS_RUNNIG):
            self.qb_ani_msg.rotation_angle[0] = self.qb_ani_msg.rotation_angle[0]+1

    #publisher func
    def pub_animation_request(self, time_now):
        self.qb_ani_msg.rotation_angle[0] = 1
        self.qb_ani_msg.rotation_angle[1] = 2
        self.qb_ani_msg.rotation_angle[2] = 3

        self.qb_ani_msg.rotation_speed[0] = 4
        self.qb_ani_msg.rotation_speed[1] = 5
        self.qb_ani_msg.rotation_speed[2] = 6

        self.qb_ani_msg.recess_travel = 7
        self.qb_ani_msg.recess_speed = 8

        self.qb_ani_publisher.publish(self.qb_ani_msg)

#main thread
def start_manager():
    #init ROS node
    rospy.init_node('qb_ani_mngr')
    #init and start the qB class
    qb_ani = QbAni()
    qb_ani.main_loop()
    #shutdown listener
    while not rospy.is_shutdown():
        qb_ani.main_rate.sleep()
    
#main thread entry point
if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass

