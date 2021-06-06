#!/usr/bin/env python
import os.path
import rospy
import rosbag
from nav_msgs.msg import Odometry
import sys

import pandas as pd
df = pd.read_csv(
    '/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_odometry_data.csv')

# rospy.init_node('odometry_bag_writer')
#startTime = rospy.Time.now()

file_name = 'paper_solution_odom.bag'
if not os.path.isfile(file_name):
    outbag = file_name
else:
    print("Outbag already exists : ", file_name)
    sys.exit(0)
ms = 0
with rosbag.Bag(outbag, 'w') as bag:
    for row in range(df.shape[0]):

        # Get Timestamp
        timestamp = rospy.Time.from_sec(ms*1e-03)

        # Write Odometry Messag
        odom_msg = Odometry()
        # Header
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "velodyne"
        odom_msg.header.stamp = timestamp
        # Postion
        odom_msg.pose.pose.position.x = df['x'][row]
        odom_msg.pose.pose.position.y = df['y'][row]
        odom_msg.pose.pose.position.z = df['z'][row]
        # Orientation
        odom_msg.pose.pose.orientation.w = df['qx'][row]
        odom_msg.pose.pose.orientation.x = df['qy'][row]
        odom_msg.pose.pose.orientation.y = df['qz'][row]
        odom_msg.pose.pose.orientation.z = df['qw'][row]

        # Write msg to bag
        bag.write("/paper_solution/odometry", odom_msg, timestamp)
        ms += 100
