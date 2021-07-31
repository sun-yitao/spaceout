#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from matplotlib import pyplot as plt
import seaborn as sns
import pandas as pd


odom, imu, combined = [], [], []

def callback_combined(data):
    global odom, imu, combined
    if len(combined) < 50:
        combined.append(data.pose.pose.position)
    else:
        x,y,types = [],[],[]
        for p in odom:
            x.append(p.x)
            y.append(p.y)
            types.append('odom')
        for p in imu:
            x.append(p.x)
            y.append(p.y)
            types.append('imu')
        for p in combined:
            x.append(p.x)
            y.append(p.y)
            types.append('combined')
        df = pd.DataFrame(data={'x': x, 'y': y, 'type': types})
        rospy.loginfo(df)
        plot = sns.scatterplot(x="x", y="y", hue='type', data=df)
        plot.figure.savefig("/home/parallels/Downloads/output.png")
        rospy.loginfo("Figure saved")
        odom, imu, combined = [],[],[]



def callback_odom(data):
    odom.append(data.pose.pose.position)

def callback_imu(data):
    imu.append(data.pose.pose.position)
    
def listener():
    rospy.init_node('ekf_visualisation', anonymous=True)

    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback_combined)
    rospy.Subscriber("odom", PoseWithCovarianceStamped, callback_odom)
    rospy.Subscriber("/imu", PoseWithCovarianceStamped, callback_imu)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()