#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg

def handle_map_frame(data):
    pub = rospy.Publisher("map", nav_msgs.msg.OccupancyGrid, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    data.header.frame_id = "/map"
    pub.publish(data)
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('map_frame_broadcaster')
    rospy.Subscriber("/occupancy", nav_msgs.msg.OccupancyGrid, handle_map_frame)
    ##rospy.Subscriber('/sensor_us', sensor_msgs.msg.Range, handle_camera_pose)
    rospy.spin()
