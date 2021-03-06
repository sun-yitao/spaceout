#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


from matplotlib import pyplot as plt
import seaborn as sns
import pandas as pd


gt, odom, imu, combined = [], [], [], [] # odom is gps
current_speed_x = 0
current_speed_y = 0
num_outputs_saved = 0

def callback_combined(data):
    global gt, odom, imu, combined, num_outputs_saved
    if len(combined) < 50:
        combined.append(data.pose.pose.position)
    else:
        x,y,types = [],[],[]
        rospy.loginfo(len(odom))
        rospy.loginfo(len(imu))
        for p in gt:
            x.append((-p.y + odom[0].x - 1.765)*10)
            y.append((p.x)*10)
            types.append('ground_truth')
        for p in odom:
            x.append(p.x*10)
            y.append(p.y*10)
            types.append('gps')
        for p in imu:
            x.append((p['x'] + odom[0].x)*10)
            y.append((p['y'] + odom[0].y)*10)
            types.append('imu')
        for p in combined:
            x.append(p.x*10)
            y.append(p.y*10)
            types.append('combined')
        df = pd.DataFrame(data={'x': x, 'y': y, 'type': types})
        rospy.loginfo(df)
        plot = sns.scatterplot(x="x", y="y", hue='type', data=df, marker='x')
        plt.autoscale()
        handles, labels = plot.get_legend_handles_labels()
        plot.legend(handles[:min(5, len(handles))], labels[:min(5, len(handles))], loc='lower right')
        plot.figure.savefig("/home/parallels/Downloads/output" + str(num_outputs_saved) + '.png')
        rospy.loginfo("Figure saved")
        num_outputs_saved += 1
        del gt[:]
        del odom[:]
        del imu[:]
        del combined[:]



def callback_odom(data):
    odom.append(data.pose.pose.position)

def callback_gt(data):
    gt.append(data.pose.pose.position)

def callback_imu(data):
    global current_speed_x, current_speed_y
    current_speed_x += (data.linear_acceleration.x / 10)
    current_speed_y += (data.linear_acceleration.y / 10)
    if not imu:
        imu.append({'x': 0, 'y': 0})
    else:
        imu.append({'x': imu[-1]['x'] + current_speed_x / 10, 'y': imu[-1]['y'] + current_speed_y / 10})
    
def listener():
    rospy.init_node('ekf_visualisation', anonymous=True)

    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback_combined)
    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/wheel_odom", Odometry, callback_gt)
    rospy.Subscriber("/imu", Imu, callback_imu)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()