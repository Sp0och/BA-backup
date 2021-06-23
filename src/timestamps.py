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


if __name__ == "__main__":

    odom_complete = pd.read_csv(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/tst_GT_steps.csv")

    file_xc = pd.DataFrame.to_numpy(odom_complete["x"])
    file_yc = pd.DataFrame.to_numpy(odom_complete["y"])
    file_zc = pd.DataFrame.to_numpy(odom_complete["z"])
    file_rollc = pd.DataFrame.to_numpy(odom_complete["roll"])
    file_pitchc = pd.DataFrame.to_numpy(odom_complete["pitch"])
    file_yawc = pd.DataFrame.to_numpy(odom_complete["yaw"])
    file_time = pd.DataFrame.to_numpy(odom_complete["time"])
    odom_aligned_shifted = open(
        "/home/fierz/Downloads/catkin_tools/ros_catkin_ws/src/descriptor_and_image/output/GT_steps.csv", "w"
    )
    writer_aligned_shifted = csv.writer(odom_aligned_shifted)
    writer_aligned_shifted.writerow(
        ["x", "y", "z", "roll", "pitch", "yaw", "time"])
    for i in range(len(odom_complete)):
        # file_time[i] = float(file_time[i])
        timestamp = file_time[i] / 1.0e09
        print(timestamp)

        writer_aligned_shifted.writerow(
            [file_xc[i], file_yc[i], file_zc[i], file_rollc[i], file_pitchc[i], file_yawc[i], timestamp])
