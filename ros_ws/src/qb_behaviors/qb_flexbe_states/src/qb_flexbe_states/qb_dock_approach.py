#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

class QbDockApproachState(EventState):
    '''
    State for qB aproaching the docking station. This state receives the waypoint info from a waypoint manager node
    based on the provided name.

    -- approach_speed float  Speed at which to drive the robot to the waypoint
    -- approach_waypoint_name string  Name of the waypoint to get via the waypoint manager

    <= failed         qB is unable to reach the waypoint
    <= done           Waypoint reached

    '''

    def __init__(self, approach_speed, approach_waypoint_name):
        super(QbDockApproachState, self).__init__(outcomes=['failed', 'done'])
        self._start_time = None
        self._speed = approach_speed
        self._waypoint_name = approach_waypoint_name
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
