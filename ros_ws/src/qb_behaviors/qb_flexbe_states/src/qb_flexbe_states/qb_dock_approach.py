#!/usr/bin/env python
import rospy
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import custom msgs and srvs
from qb_waypoint_manager.msg import qb_waypoint
from qb_waypoint_manager.srv import QbWaypointSave, QbWaypointSaveResponse
from qb_waypoint_manager.srv import QbWaypointSet, QbWaypointSetResponse
from qb_waypoint_manager.srv import QbWaypointGet, QbWaypointGetResponse
from qb_waypoint_manager.srv import QbWaypointRemove, QbWaypointRemoveResponse

class QbDockApproachState(EventState):
    '''
    State for qB aproaching the docking station. This state receives the waypoint info from a waypoint manager node
    based on the provided name.

    -- approach_speed float  Speed at which to drive the robot to the waypoint
    -- approach_waypoint_name string  Name of the waypoint to get via the waypoint manager

    <= failed         qB is unable to reach the waypoint
    <= done           Waypoint reached

    '''

    def __init__(self, approach_speed, waypoint_manager_name, approach_waypoint_name):
        super(QbDockApproachState, self).__init__(outcomes=['failed', 'done'])
        self._start_time = None
        self._speed = approach_speed
        self._waypoint_name = approach_waypoint_name
        self._waypoint_manager = waypoint_manager_name
        self.data = None
        self.counter = 0
        self.waypoint = qb_waypoint()
        self.goal = MoveBaseGoal()

    def execute(self, userdata):
        if(self.counter == 0):Logger.loginfo("DOCK APPROACH RUNNING!")
        # increment execution count
        self.counter = self.counter + 1
        if(self.waypoint.name != "!ERROR!"):
            # send goal to move base
            self.qb_mb_client_hndl.send_goal(self.goal)
            # check if goal is received
            wait = self.qb_mb_client_hndl.wait_for_result()
            if not wait:
                # Something went wrong with move base action server
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                return 'failed'
            else:
                # Goal reached!
                Logger.loginfo("Move Base goal reached: " + str(wait))
                return 'done'
        else:
            # Received waipoint is invalid
            rospy.logerr("Invalid waypoint received")
            return 'failed'

    def on_enter(self, userdata):
        Logger.loginfo("DOCK APPROACH STARTED!")
        self._start_time = rospy.Time.now()
        # wait for waypoint manager service
        rospy.wait_for_service("/qb_waypoint_mngr/qb_waypoint_get")
        # get goal from waypoint manager
        try:
            self.qb_wp_client_hndl = rospy.ServiceProxy("/qb_waypoint_mngr/qb_waypoint_get", QbWaypointGet)
            resp = self.qb_wp_client_hndl("QB_WP_DOCK")
            Logger.loginfo("Response: " + str(resp.waypoint))
            # waypoint received from manager, set the goal info
            self.waypoint = resp.waypoint
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose = resp.waypoint.pose
            self.goal.target_pose.pose.orientation.x = 0.0
            self.goal.target_pose.pose.orientation.y = 0.0
        # in case of error log it
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            Logger.logerr("Service call failed: %s"%e)

        if(self.waypoint.name != "!ERROR!"):
            # get goal from waypoint manager
            try:
                self.qb_mb_client_hndl = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                self.qb_mb_client_hndl.wait_for_server()
            # in case of error log it
            except Exception as e:
                print("Service call failed: %s"%e)
                Logger.logerr("Service call failed: %s"%e)
        
    def on_exit(self, userdata):
        Logger.loginfo("DOCK APPROACH ENDED!")
        
    def on_start(self):
        Logger.loginfo("DOCK APPROACH READY!")
        
    def on_stop(self):
        Logger.loginfo("DOCK APPROACH STOPPED!")
