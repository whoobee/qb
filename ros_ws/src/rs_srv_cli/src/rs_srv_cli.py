import rospy
import tf2_ros
from realsense2_camera.srv import NodePosition, NodePositionResponse
from geometry_msgs.msg import PoseStamped, TransformStamped

pose_data = PoseStamped()
pose_data_received = False

def rs_pose_client(_anchor_name, _anchor_frame):
    global pose_data
    global pose_data_received
    rospy.init_node("rs_srv_cli")
    rospy.wait_for_service("/t265/relocation/read_static_node")
    try:
        pose_data = rospy.ServiceProxy("/t265/relocation/read_static_node", NodePosition)
        pose_data = pose_data(_anchor_name, _anchor_frame)
        pose_data_received = True
        print("[INFO] Pose data received from static node.")
    except rospy.ServiceException as e:
        print("[ERROR] Service call failed: %s", e)

def rs_pose_pub():
    global pose_data
    global pose_data_received

    tf_msg = TransformStamped()

    if(pose_data_received == True):
        pub = rospy.Publisher("/anchor_pose", PoseStamped, queue_size=10)
        tf_pub = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pose_data.position.header.stamp = rospy.Time.now()
            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = "/anchor_pose"
            tf_msg.child_frame_id  = "/map"
            tf_msg.transform.translation.x = pose_data.position.pose.position.x
            tf_msg.transform.translation.y = pose_data.position.pose.position.y
            tf_msg.transform.translation.z = pose_data.position.pose.position.z
            tf_msg.transform.rotation.x = pose_data.position.pose.orientation.x
            tf_msg.transform.rotation.y = pose_data.position.pose.orientation.y
            tf_msg.transform.rotation.z = pose_data.position.pose.orientation.z
            tf_msg.transform.rotation.w = pose_data.position.pose.orientation.w

            tf_pub.sendTransform(tf_msg)
            pub.publish(pose_data.position)
            rate.sleep()

if __name__ == '__main__':
    rs_pose_client("base", "/anchor")
    try:
        rs_pose_pub()
    except rospy.ROSInterruptException:
        pass
    