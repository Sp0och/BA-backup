# Transformation imports
from scipy.spatial.transform import Rotation as Rot
from scipy.sparse import csr_matrix
from tf.transformations import euler_from_matrix
# python imports

from math import degrees
import math
import numpy as np
import pandas as pd
import csv


def euler_to_rotMat(roll, pitch, yaw):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0, 1]])
    Ry_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]])
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat


if __name__ == "__main__":

    odom_complete = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_complete.csv")

    odom_xc = pd.DataFrame.to_numpy(odom_complete["x"])
    odom_yc = pd.DataFrame.to_numpy(odom_complete["y"])
    odom_zc = pd.DataFrame.to_numpy(odom_complete["z"])
    odom_rollc = pd.DataFrame.to_numpy(odom_complete["roll"])
    odom_pitchc = pd.DataFrame.to_numpy(odom_complete["pitch"])
    odom_yawc = pd.DataFrame.to_numpy(odom_complete["yaw"])
    odom_time = pd.DataFrame.to_numpy(odom_complete["time"])

    odom_aligned = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/odom_aligned.csv", "w"
    )

    writer_aligned = csv.writer(odom_aligned)

    R_init = euler_to_rotMat(0.05771711247813663, -
                             0.26830801952133987, 1.4777457590079943)
    T_init = np.eye(4)
    T_init[0:3, 0:3] = R_init
    euler_init = euler_from_matrix(R_init, 'sxyz')
    print("euler init: ", euler_init)
    T_init[0:3, 3] = [0.17298460515469383,
                      1.3541736278090049, 0.42004325945497656]

    for i in range(len(odom_complete)):
        R = euler_to_rotMat(odom_rollc[i], odom_pitchc[i], odom_yawc[i])
        Transform = np.eye(4)
        Transform[0:3, 0:3] = R
        Transform[0:3, 3] = [odom_xc[i], odom_yc[i], odom_zc[i]]
        Transform = np.matmul(T_init, Transform)
        translation = Transform[0:3, 3]
        R = np.copy(Transform)
        R[0:3, 3] = 0
        euler = euler_from_matrix(R, 'sxyz')

        writer_aligned.writerow(
            [translation[0], translation[1], translation[2], euler[0], euler[1], euler[2], odom_time[i]])
