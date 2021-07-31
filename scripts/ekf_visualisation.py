#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    
def listener():
    rospy.init_node('ekf_visualisation', anonymous=True)

    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()