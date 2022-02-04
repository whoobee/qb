#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

class QbDockApproachState(EventState):
    '''
    Driving state for a ground robot. This state allows the robot to dock into a charger
    at a specified velocity/ speed.

    -- rotation_speed float  Speed at which to drive the robot
    -- rotation_angle float  How far to drive the robot before leaving this state

    <= failed         If behavior is unable to ready on time
    <= done           Example for a failure outcome.

    '''

    def __init__(self, rotation_speed, rotation_angle):
        super(QbDockApproachState, self).__init__(outcomes=['failed', 'done'])
        self._start_time = None
        self._speed = rotation_speed
        self._angle = rotation_angle
        self.data = None
        self.counter = 0

    def execute(self, userdata):
        Logger.loginfo("DOCK APPROACH RUNNING!")
        if(self.counter < 10):
            self.counter = self.counter + 1
        else:
            return 'done'

    def on_enter(self, userdata):
        Logger.loginfo("DOCK APPROACH STARTED!")
        self._start_time = rospy.Time.now()
        
    def on_exit(self, userdata):
        Logger.loginfo("DOCK APPROACH ENDED!")
        
    def on_start(self):
        Logger.loginfo("DOCK APPROACH READY!")
        
    def on_stop(self):
        Logger.loginfo("DOCK APPROACH STOPPED!")
