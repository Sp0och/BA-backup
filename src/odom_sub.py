#!/usr/bin/env python

# ROS
import rospy
import tf
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, euler_from_matrix
from geometry_msgs.msg import *

# Python
import numpy as np
import csv


def storage(trans, rot, timestamp, TINV):
    # create rotation matrix
    R = quaternion_matrix(rot)

    # their GT pose
    euler_complete = euler_from_matrix(R, 'sxyz')
    writer_complete.writerow(
        [trans[0], trans[1], trans[2], euler_complete[0], euler_complete[1], euler_complete[2], timestamp])

    # Complete initial state
    Transform = np.copy(R)
    Transform[0:3, 3] = trans
    print("Transform: ", Transform)
    # Iteration STEPS:
    T_step = np.matmul(TINV, Transform)
    trans_steps = np.copy(T_step[0:3, 3])
    R_step = np.copy(T_step)
    R_step[0:3, 3] = 0
    euler_step = euler_from_matrix(R_step, 'sxyz')

    writer_steps.writerow(
        [trans_steps[0], trans_steps[1], trans_steps[2], euler_step[0], euler_step[1], euler_step[2], timestamp])
    # create new TINV
    TINV = np.linalg.inv(Transform)
    return TINV


class odom_lookup:
    # class initialization for data access between subscribers and publishers
    def __init__(self):
        # setup subscribers
        self.odom_sub = rospy.Subscriber(
            "/scan_to_map_extrapolated_10Hz", Odometry, self.odomCallback)

        # image callback function
    def odomCallback(self, odom):
        global TINV
        # get timestamp
        ts = odom.header.stamp
        print("Odom callback at timestamp:", ts)
        rot = np.array([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        trans = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y,
                          odom.pose.pose.position.z])
        TINV = storage(trans, rot, ts, TINV)

# ros node initializer and calls class constructors


def node_init():
    rospy.init_node("pcl_odometry_listener_node", anonymous=True)
    print(" --- pcl_odometry_listener_node initialized ---")
    OL = odom_lookup()
    rospy.spin()


if __name__ == '__main__':

    solution_complete = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_complete.csv", "w"
    )

    solution_steps = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_steps.csv", "w"
    )

    writer_complete = csv.writer(solution_complete)
    writer_steps = csv.writer(solution_steps)
    writer_steps.writerow(["x", "y", "z", "roll", "pitch", "yaw", "time"])
    writer_complete.writerow(["x", "y", "z", "roll", "pitch", "yaw", "time"])

    TINV = np.identity(4)

    node_init()

# class definition for keeping all variables together


class transform_lookup:
    # class initialization for data access between subscribers and publishers
    def __init__(self):
        # setup class vairables
        self.listener = tf.TransformListener()
        # setup subscribers
        self.pcl_sub = rospy.Subscriber(
            "/points_raw", PointCloud2, self.pclCallback)

        # image callback function
    def pclCallback(self, ros_pcl):
        global TINV
        # get timestamp
        ts = ros_pcl.header.stamp
        print("PCL callback at timestamp:", ts)
        # print("with TINV: ", TINV)
        # Lookup Tf
        self.listener.waitForTransform(
            'odom', 'base_link', rospy.Time(0), rospy.Duration(0.5))
        (trans, rot) = self.listener.lookupTransform(
            'odom', 'base_link', rospy.Time(0))
        TINV = storage(trans, rot, ts, TINV)
