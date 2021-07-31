#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


from matplotlib import pyplot as plt
import seaborn as sns
import pandas as pd


odom, imu, combined = [], [], []
current_speed_x = 0
current_speed_y = 0

def callback_combined(data):
    global odom, imu, combined
    if len(combined) < 50:
        combined.append(data.pose.pose.position)
    else:
        x,y,types = [],[],[]
        rospy.loginfo(len(odom))
        rospy.loginfo(len(imu))
        for p in odom:
            x.append(p.x)
            y.append(p.y)
            types.append('odom')
        for p in imu:
            x.append(p['x'])
            y.append(p['y'])
            types.append('imu')
        for p in combined:
            x.append(p.x)
            y.append(p.y)
            types.append('combined')
        df = pd.DataFrame(data={'x': x, 'y': y, 'type': types})
        rospy.loginfo(df)
        plot = sns.scatterplot(x="x", y="y", hue='type', data=df)
        handles, labels = plot.get_legend_handles_labels()
        plot.legend(handles[:min(4, len(handles))], labels[:min(4, len(handles))])  
        plot.figure.savefig("/home/parallels/Downloads/output.png")
        df.to_csv('/home/parallels/Downloads/output.csv')
        rospy.loginfo("Figure saved")
        odom, imu, combined = [],[],[]



def callback_odom(data):
    odom.append(data.pose.pose.position)

def callback_imu(data):
    global current_speed_x, current_speed_y
    current_speed_x += (data.linear_acceleration.x / 30)
    current_speed_y += (data.linear_acceleration.y / 30)
    if not imu:
        imu.append({'x': current_speed_x, 'y': current_speed_y})
    else:
        imu.append({'x': imu[-1]['x'] + current_speed_x, 'y': imu[-1]['y'] + current_speed_y})
    
def listener():
    rospy.init_node('ekf_visualisation', anonymous=True)

    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback_combined)
    rospy.Subscriber("odom", Odometry, callback_odom)
    rospy.Subscriber("/imu", Imu, callback_imu)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()