#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class QbDockState(EventState):
    '''
	Driving state for a ground robot. This state allows the robot to dock into a charger
    at a specified velocity/ speed.

	-- dock_speed 	 float 	Speed at which to drive the robot
    -- dock_max_dist float  How far to drive the robot before leaving this state
    -- dock_dist     float  Distance at which to determine blockage of robot path

	<= failed 			    If behavior is unable to ready on time
	<= done 				Example for a failure outcome.

	'''

    def __init__(self, dock_speed, dock_max_dist, dock_dist):
        super(QbDockState, self).__init__(outcomes=['failed', 'done'])
        self._start_time = None
        self.data = None
        self._speed = dock_speed
        self._dist = dock_max_dist
        self._dock_dist = dock_dist

        self._infinite_distance = 9999
        self._back_region = 0
        self._back_region_range = 50

        self.vel_topic = '/cmd_vel'
        self.scan_topic = '/scan'

        #create publisher passing it the vel_topic_name and msg_type
        self.pub = ProxyPublisher({self.vel_topic: Twist})
        #create subscriber
        self.scan_sub = ProxySubscriberCached({self.scan_topic: LaserScan})
        self.scan_sub.set_callback(self.scan_topic, self.scan_callback)
    
    def get_min_distance(self, _sensor_data, _angle, _arc):
        min_dist = self._infinite_distance

        if(_sensor_data is not None):
            for vertex in range(_angle-_arc, _angle+_arc):
                if _sensor_data[vertex] != 0 and _sensor_data[vertex] < min_dist:
                    min_dist = _sensor_data[vertex]
        if min_dist == self._infinite_distance:
            min_dist = 0
        return min_dist

    def execute(self, userdata):
        if not self.cmd_pub:
            return 'failed'
        #run obstacle checks [index 0: left, 360: middle, 719: right]
        back_distance = self.get_min_distance(self.data.ranges, self._back_region, self._back_region_range)
        Logger.loginfo('Dock distance is: %s' % back_distance)
        if back_distance <= self._dock_dist:
            return 'failed'

        #measure distance travelled
        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
        distance_travelled = elapsed_time * self._speed

        if distance_travelled >= self._dist:
            return 'done'
        
        #drive
        self.pub.publish(self.vel_topic, self.cmd_pub)

    def on_enter(self, userdata):
        Logger.loginfo("DOCK STARTED!")
        #set robot speed here
        self.cmd_pub = Twist()
        self.cmd_pub.linear.x = (-self._speed)
        self.cmd_pub.angular.z = 0.0
        
    def on_exit(self, userdata):
        self.cmd_pub.linear.x = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)
        Logger.loginfo("DOCK ENDED!")
        
    def on_start(self):
        Logger.loginfo("DOCK READY!")
        self._start_time = rospy.Time.now() #bug detected! (move to on_enter)
        
    def on_stop(self):
		Logger.loginfo("DOCK STOPPED!")

    def scan_callback(self, data):
        self.data = data